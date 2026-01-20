"""
BLE Sensor Visualizer for nRF52840 Custom PCB

Receives sensor data over BLE Nordic UART Service (NUS) and displays:
- 3D device orientation from IMU (accelerometer + gyroscope)
- Battery percentage
- PPG signal with heart rate detection
- SpO2 (blood oxygen saturation) measurements

Requirements:
    pip install bleak numpy matplotlib scipy

Usage:
    python ble_visualizer.py
"""

import asyncio
import math
import struct
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:
    print("Please install bleak: pip install bleak")
    sys.exit(1)

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write to device
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Receive from device

# Packet constants
PACKET_SIZE = 32
SYNC1 = 0xAA
SYNC2 = 0x55

# Reconnection settings
RECONNECT_DELAY = 2.0  # seconds between reconnection attempts
MAX_RECONNECT_ATTEMPTS = 0  # 0 = infinite

@dataclass
class SensorData:
    """Parsed sensor packet data"""
    accel_x: float  # m/s^2
    accel_y: float
    accel_z: float
    gyro_x: float   # rad/s
    gyro_y: float
    gyro_z: float
    ppg_raw: int
    battery_percent: int
    charge_status: int
    timestamp_ms: int
    spo2_percent: int
    heart_rate_ppg: int
    signal_quality: int
    skin_temp_c: float  # Skin temperature in Celsius
    hr_mode: bool       # Heart rate mode enabled
    spo2_mode: bool     # SpO2 mode enabled

class MadgwickFilter:
    """Simple Madgwick filter for orientation estimation"""

    def __init__(self, beta=0.1):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        self.last_time = None

    def update(self, accel, gyro, dt):
        """Update orientation with IMU data"""
        if dt <= 0:
            return

        q = self.q
        ax, ay, az = accel
        gx, gy, gz = gyro

        # Normalize accelerometer
        norm = np.sqrt(ax*ax + ay*ay + az*az)
        if norm < 0.001:
            return
        ax, ay, az = ax/norm, ay/norm, az/norm

        # Gradient descent algorithm corrective step
        q0, q1, q2, q3 = q

        f1 = 2*(q1*q3 - q0*q2) - ax
        f2 = 2*(q0*q1 + q2*q3) - ay
        f3 = 2*(0.5 - q1*q1 - q2*q2) - az

        J_11or24 = 2*q2
        J_12or23 = 2*q3
        J_13or22 = 2*q0
        J_14or21 = 2*q1
        J_32 = 2*J_14or21
        J_33 = 2*J_11or24

        step0 = J_14or21*f2 - J_11or24*f1
        step1 = J_12or23*f1 + J_13or22*f2 - J_32*f3
        step2 = J_12or23*f2 - J_33*f3 - J_13or22*f1
        step3 = J_14or21*f1 + J_11or24*f2

        norm = np.sqrt(step0*step0 + step1*step1 + step2*step2 + step3*step3)
        if norm > 0:
            step0, step1, step2, step3 = step0/norm, step1/norm, step2/norm, step3/norm

        # Apply feedback step
        qDot0 = 0.5*(-q1*gx - q2*gy - q3*gz) - self.beta*step0
        qDot1 = 0.5*(q0*gx + q2*gz - q3*gy) - self.beta*step1
        qDot2 = 0.5*(q0*gy - q1*gz + q3*gx) - self.beta*step2
        qDot3 = 0.5*(q0*gz + q1*gy - q2*gx) - self.beta*step3

        # Integrate
        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt

        # Normalize quaternion
        norm = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q = np.array([q0/norm, q1/norm, q2/norm, q3/norm])

    def get_euler(self):
        """Get Euler angles (roll, pitch, yaw) in degrees"""
        q0, q1, q2, q3 = self.q

        roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
        pitch = np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1))
        yaw = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))

        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def get_rotation_matrix(self):
        """Get rotation matrix from quaternion"""
        q0, q1, q2, q3 = self.q

        R = np.array([
            [1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3), 1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2)]
        ])
        return R

class MotionDetector:
    """Motion detection using IMU data for artifact rejection"""

    def __init__(self, sample_rate=12):  # ~12.5Hz (25Hz / 2x nRF averaging)
        self.sample_rate = sample_rate
        self.buffer_size = sample_rate * 2  # 2-second buffer for motion analysis

        # IMU buffers
        self.accel_buffer = deque(maxlen=self.buffer_size)
        self.gyro_buffer = deque(maxlen=self.buffer_size)

        # Motion metrics
        self.motion_intensity = 0.0  # 0-1 scale
        self.is_moving = False

        # Thresholds (tuned for wrist-worn device)
        self.accel_threshold = 0.3   # m/s^2 variance threshold
        self.gyro_threshold = 0.15   # rad/s variance threshold
        self.motion_decay = 0.9      # Decay factor for motion intensity

        # High-motion detection (for complete rejection)
        self.high_motion_threshold = 0.6

    def add_sample(self, accel, gyro):
        """Add IMU sample (accel, gyro as tuples of x,y,z)"""
        # Calculate magnitude of acceleration deviation from 1g
        accel_mag = np.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
        accel_deviation = abs(accel_mag - 9.81)  # Deviation from gravity

        # Calculate angular velocity magnitude
        gyro_mag = np.sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2)

        self.accel_buffer.append(accel_deviation)
        self.gyro_buffer.append(gyro_mag)

        # Update motion intensity
        self._update_motion()

    def _update_motion(self):
        """Update motion intensity based on recent IMU data"""
        if len(self.accel_buffer) < 10:
            return

        # Calculate variance of recent samples (1-second window = sample_rate samples)
        accel_arr = np.array(list(self.accel_buffer))
        gyro_arr = np.array(list(self.gyro_buffer))
        window = self.sample_rate  # 1-second window

        accel_var = np.var(accel_arr[-window:]) if len(accel_arr) >= window else np.var(accel_arr)
        gyro_var = np.var(gyro_arr[-window:]) if len(gyro_arr) >= window else np.var(gyro_arr)

        # Also consider mean values (sustained motion)
        accel_mean = np.mean(accel_arr[-window:]) if len(accel_arr) >= window else np.mean(accel_arr)
        gyro_mean = np.mean(gyro_arr[-window:]) if len(gyro_arr) >= window else np.mean(gyro_arr)

        # Combine metrics into motion intensity (0-1)
        accel_score = min(1.0, (accel_var / self.accel_threshold + accel_mean / 2.0) / 2.0)
        gyro_score = min(1.0, (gyro_var / self.gyro_threshold + gyro_mean / 0.5) / 2.0)

        # Weighted combination (gyro is often more reliable for motion)
        new_intensity = 0.4 * accel_score + 0.6 * gyro_score

        # Smooth the motion intensity (fast attack, slow decay)
        if new_intensity > self.motion_intensity:
            self.motion_intensity = 0.7 * self.motion_intensity + 0.3 * new_intensity
        else:
            self.motion_intensity = self.motion_decay * self.motion_intensity + (1 - self.motion_decay) * new_intensity

        self.is_moving = self.motion_intensity > 0.3

    def should_reject_sample(self):
        """Returns True if current motion level is too high for reliable PPG"""
        return self.motion_intensity > self.high_motion_threshold

    def get_quality_factor(self):
        """Returns 0-1 quality factor (1 = no motion, 0 = high motion)"""
        return max(0.0, 1.0 - self.motion_intensity)


class HeartRateDetector:
    """Heart rate detection from PPG with IMU-based motion artifact removal"""

    def __init__(self, sample_rate=12):  # ~12.5Hz (25Hz sensor / 2x nRF averaging)
        self.sample_rate = sample_rate
        self.window_seconds = 12  # 12-second window for display + delay buffer
        self.buffer_size = sample_rate * self.window_seconds  # ~144 samples at 12Hz

        # PPG buffer (raw values)
        self.ppg_buffer = deque(maxlen=self.buffer_size)

        # Motion-aware quality tracking for each sample
        self.quality_buffer = deque(maxlen=self.buffer_size)

        # Motion detector
        self.motion_detector = MotionDetector(sample_rate)

        # Heart rate tracking
        self.last_hr = 0
        self.hr_confidence = 0.0  # 0-1 confidence in current HR
        self.hr_history = deque(maxlen=10)  # Recent HR estimates for trend
        self.hr_trend = 0  # -1 decreasing, 0 stable, 1 increasing

        # Filters
        self.filter_initialized = False
        self.sos_bp = None
        self.sos_hp = None  # High-pass for baseline removal
        self.sos_vpg = None  # Bandpass for VPG (clinical: 0.5-5 Hz)
        self.sos_apg = None  # Bandpass for APG (clinical: 0.5-8 Hz)

        # Enhanced artifact detection
        self.last_valid_ppg = None
        self.last_ppg_values = deque(maxlen=5)  # For derivative calculation
        self.artifact_threshold = 0.25  # Tighter threshold
        self.derivative_threshold = 0.15  # Max allowed derivative
        self.consecutive_artifacts = 0

        # Baseline tracking (for drift detection)
        self.baseline = None
        self.baseline_alpha = 0.005  # Very slow baseline adaptation

        # Running statistics
        self.running_mean = None
        self.running_std = None
        self.mean_alpha = 0.02
        self.std_alpha = 0.05

        # Beat segmentation for display
        self.beat_segments = deque(maxlen=5)  # Store last 5 beats
        self.last_peak_idx = 0

        # Noise floor detection - reject when nothing attached
        # Lowered thresholds: real signal typically has 200+ ADC variation
        # Noise floor (no finger) is typically < 10 ADC counts variation
        self.noise_floor_threshold = 8  # Minimum std dev (ADC counts) - lowered for weak signals
        self.signal_present = False
        self.min_ppg_range = 20  # Minimum peak-to-peak range for valid signal - lowered

        # Bad data segment tracking - skip artifacts in display
        self.last_artifact_idx = 0  # Index of last detected artifact
        self.valid_segment_start = 0  # Start of current valid data segment

        # Stable normalization - prevent amplitude "undulation"
        # Use slow-updating scale factors so waveforms don't jump around
        self.ppg_scale = None  # Running scale for PPG
        self.vpg_scale = None  # Running scale for VPG
        self.apg_scale = None  # Running scale for APG
        self.scale_alpha = 0.05  # How fast to adapt (0.05 = ~1sec settling, faster recovery)
        self.min_scale = 0.001  # Prevent division by zero
        self.scale_samples = 0  # Count samples since scale init

        # Peak-based BPM calculation from systolic peaks
        self.systolic_peak_times = deque(maxlen=20)  # Legacy - kept for compatibility
        self.peak_based_bpm = 0  # Current BPM calculated from peak intervals
        self.last_peak_sample_idx = -1  # Sample index of last detected peak
        self.hrv_rmssd = 0  # HRV metric (RMSSD in milliseconds)
        self.last_peak_count = 0  # Number of peaks in last analysis
        self.valid_interval_count = 0  # Number of valid intervals used for BPM

        # Initialize filters
        self._init_filters()

    def _init_filters(self):
        """Initialize signal processing filters with clinical-grade parameters"""
        try:
            # Bandpass filter: 0.8 - 3.0 Hz (48-180 BPM) - tighter range
            self.sos_bp = signal.butter(4, [0.8, 3.0], btype='band',
                                        fs=self.sample_rate, output='sos')

            # High-pass filter for baseline wander removal (0.5 Hz cutoff)
            self.sos_hp = signal.butter(2, 0.5, btype='high',
                                        fs=self.sample_rate, output='sos')

            # Clinical-grade VPG filter: 0.5-5 Hz
            # VPG (first derivative) shows blood flow velocity
            # Clinical standard range captures the primary cardiac components
            nyquist = self.sample_rate / 2
            vpg_high = min(5.0, nyquist * 0.9)  # Stay below Nyquist
            self.sos_vpg = signal.butter(3, [0.5, vpg_high], btype='band',
                                         fs=self.sample_rate, output='sos')

            # Clinical-grade APG filter: 0.5-6 Hz (slightly higher for acceleration)
            # APG (second derivative) shows a, b, c, d, e waves
            apg_high = min(6.0, nyquist * 0.9)
            self.sos_apg = signal.butter(3, [0.5, apg_high], btype='band',
                                         fs=self.sample_rate, output='sos')

            self.filter_initialized = True
        except Exception as e:
            print(f"Filter init error: {e}")

    def add_imu_sample(self, accel, gyro):
        """Add IMU sample for motion detection"""
        self.motion_detector.add_sample(accel, gyro)

    def _detect_spike(self, ppg_value):
        """Detect spikes using derivative analysis"""
        if len(self.last_ppg_values) < 3:
            return False

        # Calculate first derivative (rate of change)
        recent = list(self.last_ppg_values)
        if self.running_mean and self.running_mean > 0:
            # Normalize by running mean for relative derivative
            derivatives = []
            for i in range(1, len(recent)):
                d = abs(recent[i] - recent[i-1]) / self.running_mean
                derivatives.append(d)

            # Check current derivative
            current_deriv = abs(ppg_value - recent[-1]) / self.running_mean

            # Spike: sudden large derivative followed by reversal or plateau
            if current_deriv > self.derivative_threshold:
                # Check if previous derivatives were normal (sudden spike)
                if len(derivatives) >= 2:
                    avg_prev_deriv = np.mean(derivatives[-2:])
                    if current_deriv > 3 * avg_prev_deriv:
                        return True

        return False

    def _detect_drift(self, ppg_value):
        """Detect slow baseline drift"""
        if self.baseline is None:
            self.baseline = ppg_value
            return False

        # Update baseline very slowly
        self.baseline = (1 - self.baseline_alpha) * self.baseline + self.baseline_alpha * ppg_value

        # Check if value is drifting from expected baseline
        if self.running_std and self.running_std > 0:
            deviation = abs(ppg_value - self.baseline) / self.running_std
            # Flag as drift if consistently offset from baseline
            return deviation > 4.0  # More than 4 std deviations

        return False

    def _is_artifact(self, ppg_value):
        """Enhanced artifact detection with spike and drift detection"""
        if self.last_valid_ppg is None:
            return False

        # 1. Check for sudden spike
        if self._detect_spike(ppg_value):
            return True

        # 2. Check relative change (step artifact)
        relative_change = abs(ppg_value - self.last_valid_ppg) / max(self.last_valid_ppg, 1)
        motion_factor = 1.0 + self.motion_detector.motion_intensity
        threshold = self.artifact_threshold * motion_factor

        if relative_change > threshold:
            return True

        # 3. Check for drift (slower check)
        if self._detect_drift(ppg_value):
            return True

        return False

    def add_sample(self, ppg_value):
        """Add a new PPG sample with enhanced artifact rejection"""
        if ppg_value <= 0:
            return

        # Track last values for derivative calculation
        self.last_ppg_values.append(ppg_value)

        # Get current motion quality
        motion_quality = self.motion_detector.get_quality_factor()

        # Check for artifact
        is_artifact = self._is_artifact(ppg_value)

        if is_artifact:
            self.consecutive_artifacts += 1
            # Track artifact location for display skipping
            self.last_artifact_idx = len(self.ppg_buffer)

            # After several artifacts, accept new baseline and reset segment
            if self.consecutive_artifacts >= 5:
                self.last_valid_ppg = ppg_value
                self.running_mean = ppg_value
                self.baseline = ppg_value
                self.consecutive_artifacts = 0
                # Start new valid segment after artifacts clear
                self.valid_segment_start = len(self.ppg_buffer)
                # Reset scale factors for instant visual recovery
                self.ppg_scale = None
                self.vpg_scale = None
                self.apg_scale = None

            # Still add sample but with very low quality
            self.ppg_buffer.append(float(ppg_value))
            self.quality_buffer.append(0.1 * motion_quality)
            return

        # Valid sample
        self.consecutive_artifacts = 0
        self.last_valid_ppg = ppg_value

        # Decay valid_segment_start as good data accumulates
        # After ~1 second of clean data, reset the segment start (faster recovery)
        if self.valid_segment_start > 0:
            samples_since_artifact = len(self.ppg_buffer) - self.valid_segment_start
            if samples_since_artifact > self.sample_rate * 1:  # 1 second of good data
                self.valid_segment_start = 0  # Reset - all data is now valid

        # Update running statistics
        if self.running_mean is None:
            self.running_mean = ppg_value
            self.running_std = 100  # Initial estimate
        else:
            self.running_mean = (1 - self.mean_alpha) * self.running_mean + self.mean_alpha * ppg_value
            diff = abs(ppg_value - self.running_mean)
            self.running_std = (1 - self.std_alpha) * self.running_std + self.std_alpha * diff

        # Add to buffer with quality score
        self.ppg_buffer.append(float(ppg_value))

        # Quality is based on motion and signal characteristics
        signal_quality = 1.0
        if self.running_std is not None and self.running_std > 0:
            deviation = abs(ppg_value - self.running_mean) / (self.running_std + 1)
            signal_quality = max(0.3, 1.0 - 0.1 * deviation)

        total_quality = motion_quality * signal_quality
        self.quality_buffer.append(total_quality)

    def _remove_baseline_wander(self, data):
        """Remove baseline wander using polynomial detrending"""
        if len(data) < 20:
            return data - np.mean(data)

        # Fit a low-order polynomial to capture baseline drift
        x = np.arange(len(data))
        try:
            # Use robust polynomial fit (order 2 for slow drift)
            coeffs = np.polyfit(x, data, 2)
            baseline = np.polyval(coeffs, x)
            return data - baseline
        except Exception:
            return data - np.mean(data)

    def _reject_outliers(self, data, quality, threshold=2.5):
        """Replace outliers with interpolated values"""
        if len(data) < 10:
            return data

        # Calculate robust statistics
        median = np.median(data)
        mad = np.median(np.abs(data - median))  # Median absolute deviation
        if mad < 0.001:
            mad = np.std(data)

        # Find outliers
        outlier_mask = np.abs(data - median) > threshold * mad * 1.4826

        # Also mark low-quality samples as potential outliers
        if len(quality) == len(data):
            outlier_mask |= quality < 0.3

        # Interpolate outliers
        if np.any(outlier_mask):
            good_indices = np.where(~outlier_mask)[0]
            bad_indices = np.where(outlier_mask)[0]

            if len(good_indices) >= 2:
                # Linear interpolation for outliers
                data_clean = data.copy()
                data_clean[bad_indices] = np.interp(bad_indices, good_indices, data[good_indices])
                return data_clean

        return data

    def check_signal_present(self):
        """Check if a valid PPG signal is present (finger attached).

        Returns True if signal variation is above noise floor.
        Uses hysteresis to prevent flickering during artifacts.
        """
        if len(self.ppg_buffer) < 20:
            self.signal_present = False
            return False

        # Initialize hysteresis counter if needed
        if not hasattr(self, 'signal_absent_count'):
            self.signal_absent_count = 0

        # Get recent samples
        recent = np.array(list(self.ppg_buffer)[-30:], dtype=float)

        # Check peak-to-peak range
        ppg_range = np.max(recent) - np.min(recent)

        # Check standard deviation (more robust)
        ppg_std = np.std(recent)

        # Lower thresholds for detecting signal
        signal_detected = (ppg_range > self.min_ppg_range * 0.5 or
                          ppg_std > self.noise_floor_threshold * 0.5)

        was_present = self.signal_present

        # Hysteresis: require signal to be absent for several checks before declaring NO SIGNAL
        if signal_detected:
            self.signal_present = True
            self.signal_absent_count = 0
        else:
            self.signal_absent_count += 1
            # Only declare no signal after 15 consecutive absent readings (~0.5 sec at 30fps)
            if self.signal_absent_count > 15:
                self.signal_present = False

        # Reset scale factors when signal is regained (not when lost - preserves display)
        if not was_present and self.signal_present:
            self.ppg_scale = None
            self.vpg_scale = None
            self.apg_scale = None
            self.scale_samples = 0

        return self.signal_present

    def get_derivative_signals(self):
        """Get clinical-standard VPG and APG derivatives.

        VPG (Velocity PPG) - first derivative:
          - Clinical standard: bandpass 0.5-5 Hz
          - Shows blood flow velocity
          - Systolic peak = zero crossing (positive to negative)
          - More robust peak detection than raw PPG

        APG (Acceleration PPG) - second derivative:
          - Clinical standard: bandpass 0.5-8 Hz
          - Reveals a, b, c, d, e inflection waves:
            * a-wave: early systolic positive peak
            * b-wave: early systolic negative peak
            * c-wave: late systolic re-rise
            * d-wave: dicrotic notch
            * e-wave: early diastolic peak
          - APG aging index = (b-c-d-e)/a used for arterial stiffness

        Skips artifact segments to show only clean waveforms.
        """
        # Check if signal is present first
        if not self.check_signal_present():
            return np.array([]), np.array([]), np.array([])

        filtered = self.get_filtered_signal()
        if len(filtered) < 20:
            return np.array([]), np.array([]), np.array([])

        # Skip artifact portion - use only data after valid_segment_start
        buffer_len = len(self.ppg_buffer)
        if self.valid_segment_start > 0 and buffer_len > 0:
            skip_samples = max(0, self.valid_segment_start - (buffer_len - len(filtered)))
            if skip_samples > 0 and skip_samples < len(filtered) - 20:
                filtered = filtered[skip_samples:]

        if len(filtered) < 20:
            return np.array([]), np.array([]), np.array([])

        # First derivative (VPG) - central difference
        vpg_raw = np.gradient(filtered) * self.sample_rate

        # Apply clinical VPG bandpass filter (0.5-5 Hz)
        if self.filter_initialized and len(vpg_raw) >= 30:
            try:
                vpg = signal.sosfiltfilt(self.sos_vpg, vpg_raw)
            except Exception:
                vpg = vpg_raw
        else:
            vpg = vpg_raw

        # Light smoothing with Savitzky-Golay (reduced window for less over-smoothing)
        if len(vpg) >= 5:
            vpg = signal.savgol_filter(vpg, 5, 3)

        # Second derivative (APG) - derivative of VPG
        apg_raw = np.gradient(vpg) * self.sample_rate

        # Apply clinical APG bandpass filter (0.5-6 Hz)
        if self.filter_initialized and len(apg_raw) >= 30:
            try:
                apg = signal.sosfiltfilt(self.sos_apg, apg_raw)
            except Exception:
                apg = apg_raw
        else:
            apg = apg_raw

        # Light APG smoothing (reduced from 9 to 7 for less over-smoothing)
        if len(apg) >= 7:
            apg = signal.savgol_filter(apg, 7, 3)

        # Stable normalization for VPG - prevents amplitude undulation
        vpg_std = np.std(vpg)
        if vpg_std > self.min_scale:
            if self.vpg_scale is None:
                self.vpg_scale = vpg_std
            else:
                self.vpg_scale = (1 - self.scale_alpha) * self.vpg_scale + self.scale_alpha * vpg_std
            vpg = vpg / self.vpg_scale

        # Stable normalization for APG
        apg_std = np.std(apg)
        if apg_std > self.min_scale:
            if self.apg_scale is None:
                self.apg_scale = apg_std
            else:
                self.apg_scale = (1 - self.scale_alpha) * self.apg_scale + self.scale_alpha * apg_std
            apg = apg / self.apg_scale

        return filtered, vpg, apg

    def get_filtered_signal(self):
        """Get filtered PPG signal - minimal processing to preserve peaks"""
        if len(self.ppg_buffer) < 25:
            data = np.array(list(self.ppg_buffer), dtype=float)
            if len(data) > 0:
                return data - np.mean(data)
            return np.array([])

        data = np.array(list(self.ppg_buffer), dtype=float)

        # 1. Simple baseline removal (subtract mean) - preserves amplitude
        data = data - np.mean(data)

        # 2. Light bandpass filter only (0.5-4 Hz for heart rate range)
        if self.filter_initialized and len(data) >= 50:
            try:
                filtered = signal.sosfiltfilt(self.sos_bp, data)
                return filtered
            except Exception:
                pass

        return data

    def get_beat_aligned_signal(self):
        """Get signal segments aligned to heartbeat peaks for stacked display"""
        filtered = self.get_filtered_signal()
        if len(filtered) < 50:
            return None, []

        # Find peaks in filtered signal
        std = np.std(filtered)
        if std < 0.01:
            return filtered, []

        min_distance = int(self.sample_rate * 0.4)  # Min 0.4s between beats
        peaks, properties = signal.find_peaks(
            filtered,
            distance=min_distance,
            prominence=std * 0.3,
            height=0
        )

        if len(peaks) < 2:
            return filtered, []

        # Extract beat segments (centered on each peak)
        # Use median interval to determine segment width
        intervals = np.diff(peaks)
        valid_intervals = intervals[(intervals > self.sample_rate * 0.4) &
                                   (intervals < self.sample_rate * 1.5)]

        if len(valid_intervals) == 0:
            return filtered, []

        beat_width = int(np.median(valid_intervals))
        half_width = beat_width // 2

        beat_segments = []
        for peak in peaks:
            start = peak - half_width
            end = peak + half_width

            if start >= 0 and end < len(filtered):
                segment = filtered[start:end].copy()

                # Normalize each beat to consistent amplitude
                seg_max = np.max(np.abs(segment))
                if seg_max > 0.1:
                    segment = segment / seg_max

                beat_segments.append(segment)

        # Store for later access
        self.beat_segments = deque(beat_segments[-5:], maxlen=5)

        return filtered, beat_segments

    def get_stacked_beats(self):
        """Get the last N beats stacked and averaged for display"""
        _, beat_segments = self.get_beat_aligned_signal()

        if len(beat_segments) < 2:
            return None, None

        # Ensure all segments are same length
        min_len = min(len(s) for s in beat_segments)
        aligned = [s[:min_len] for s in beat_segments]

        # Stack individual beats
        stacked = np.array(aligned)

        # Calculate average beat template
        avg_beat = np.mean(stacked, axis=0)

        # Create time axis (centered on peak)
        time_axis = (np.arange(min_len) - min_len // 2) / self.sample_rate * 1000  # ms

        return time_axis, stacked, avg_beat

    def get_monitor_signal(self, num_beats=5):
        """Get a continuous signal showing the last N beats like an ER monitor.

        Returns normalized, concatenated beats for smooth scrolling display.
        Skips any artifact segments to show only clean waveforms.
        """
        filtered = self.get_filtered_signal()
        if len(filtered) < 50:
            return None, None

        # Skip data before valid_segment_start (after artifacts)
        # Map valid_segment_start to filtered signal index
        buffer_len = len(self.ppg_buffer)
        if self.valid_segment_start > 0 and buffer_len > 0:
            # Calculate how much of the filtered signal is valid
            skip_samples = max(0, self.valid_segment_start - (buffer_len - len(filtered)))
            if skip_samples > 0 and skip_samples < len(filtered):
                filtered = filtered[skip_samples:]

        if len(filtered) < 30:
            return None, None

        # Find peaks in filtered signal
        std = np.std(filtered)
        if std < 0.01:
            return None, None

        min_distance = int(self.sample_rate * 0.4)  # Min 0.4s between beats
        peaks, properties = signal.find_peaks(
            filtered,
            distance=min_distance,
            prominence=std * 0.3,
            height=0
        )

        if len(peaks) < 2:
            return None, None

        # Get the median beat interval for consistent display width
        intervals = np.diff(peaks)
        valid_intervals = intervals[(intervals > self.sample_rate * 0.4) &
                                   (intervals < self.sample_rate * 1.5)]
        if len(valid_intervals) == 0:
            return None, None

        beat_width = int(np.median(valid_intervals))

        # Extract and normalize each beat segment, then concatenate
        # Take the last N complete beats
        display_beats = []
        for i in range(len(peaks) - 1, -1, -1):
            if len(display_beats) >= num_beats:
                break

            peak = peaks[i]
            # Use consistent width centered on peak
            half_width = beat_width // 2
            start = peak - half_width
            end = peak + half_width

            if start >= 0 and end <= len(filtered):
                segment = filtered[start:end].copy()

                # Normalize amplitude to consistent range
                seg_min = np.min(segment)
                seg_max = np.max(segment)
                seg_range = seg_max - seg_min

                if seg_range > 0.1:
                    # Normalize to -1 to +1 range
                    segment = 2.0 * (segment - seg_min) / seg_range - 1.0

                display_beats.insert(0, segment)  # Insert at front to maintain order

        if len(display_beats) < 2:
            return None, None

        # Concatenate all beats into a continuous signal
        monitor_signal = np.concatenate(display_beats)

        # Create time axis (seconds, for display width)
        total_samples = len(monitor_signal)
        time_axis = np.arange(total_samples) / self.sample_rate

        return time_axis, monitor_signal

    def _find_peaks_autocorr(self, data):
        """Find heart rate using autocorrelation method"""
        if len(data) < self.sample_rate * 2:
            return None, 0

        # Autocorrelation
        autocorr = np.correlate(data, data, mode='full')
        autocorr = autocorr[len(autocorr)//2:]  # Take positive lags only

        # Normalize
        autocorr = autocorr / autocorr[0]

        # Look for peaks in valid HR range (40-180 BPM)
        min_lag = int(self.sample_rate * 60 / 180)  # 180 BPM
        max_lag = int(self.sample_rate * 60 / 40)   # 40 BPM

        if max_lag >= len(autocorr):
            max_lag = len(autocorr) - 1

        # Find peaks in autocorrelation
        search_region = autocorr[min_lag:max_lag+1]
        if len(search_region) < 5:
            return None, 0

        peaks, properties = signal.find_peaks(search_region, prominence=0.1)

        if len(peaks) == 0:
            return None, 0

        # Get the first (strongest) peak
        peak_idx = peaks[0] + min_lag
        hr = 60 * self.sample_rate / peak_idx

        # Confidence based on peak prominence
        confidence = min(1.0, autocorr[peak_idx] * 2)

        return hr, confidence

    def _find_peaks_direct(self, data):
        """Find heart rate using direct peak detection"""
        if len(data) < self.sample_rate * 2:
            return None, 0

        std = np.std(data)
        if std < 0.01:
            return None, 0

        # Find peaks with adaptive parameters
        min_distance = int(self.sample_rate * 0.33)  # Max ~180 BPM
        prominence = std * 0.3

        peaks, properties = signal.find_peaks(
            data,
            distance=min_distance,
            prominence=prominence,
            height=0  # Only positive peaks (after normalization)
        )

        if len(peaks) < 2:
            return None, 0

        # Calculate intervals
        intervals = np.diff(peaks)

        # Filter valid intervals (40-180 BPM range)
        min_interval = self.sample_rate * 60 / 180
        max_interval = self.sample_rate * 60 / 40
        valid_intervals = intervals[(intervals >= min_interval) & (intervals <= max_interval)]

        if len(valid_intervals) == 0:
            return None, 0

        # Use median for robustness
        median_interval = np.median(valid_intervals)
        hr = 60 * self.sample_rate / median_interval

        # Calculate confidence based on interval consistency
        if len(valid_intervals) >= 2:
            interval_std = np.std(valid_intervals)
            consistency = max(0, 1.0 - interval_std / median_interval)
            coverage = len(valid_intervals) / max(1, len(intervals))
            confidence = consistency * coverage
        else:
            confidence = 0.3

        return hr, confidence

    def _find_peaks_vpg(self, data):
        """Find heart rate using VPG (first derivative) zero crossings.

        VPG zero crossings from positive to negative correspond to systolic peaks
        in the PPG waveform. This method is often more robust than direct peak
        detection, especially with noisy signals.
        """
        if len(data) < self.sample_rate * 2:
            return None, 0

        # Calculate first derivative (VPG)
        vpg = np.gradient(data) * self.sample_rate

        # Smooth VPG to reduce noise
        if len(vpg) >= 7:
            vpg = signal.savgol_filter(vpg, 7, 3)

        # Find zero crossings (positive to negative = systolic peaks)
        zero_crossings = []
        for i in range(1, len(vpg)):
            if vpg[i-1] > 0 and vpg[i] <= 0:
                # Interpolate for more precise crossing point
                if vpg[i-1] != vpg[i]:
                    frac = vpg[i-1] / (vpg[i-1] - vpg[i])
                    zero_crossings.append(i - 1 + frac)
                else:
                    zero_crossings.append(float(i))

        if len(zero_crossings) < 2:
            return None, 0

        # Calculate intervals between zero crossings
        zero_crossings = np.array(zero_crossings)
        intervals = np.diff(zero_crossings)

        # Filter valid intervals (40-180 BPM range)
        min_interval = self.sample_rate * 60 / 180
        max_interval = self.sample_rate * 60 / 40
        valid_intervals = intervals[(intervals >= min_interval) & (intervals <= max_interval)]

        if len(valid_intervals) < 2:
            return None, 0

        # Use median for robustness
        median_interval = np.median(valid_intervals)
        hr = 60 * self.sample_rate / median_interval

        # Calculate confidence based on interval consistency
        interval_std = np.std(valid_intervals)
        consistency = max(0, 1.0 - interval_std / median_interval)
        coverage = len(valid_intervals) / max(1, len(intervals))
        confidence = consistency * coverage * 1.1  # Slight boost for VPG method

        return hr, min(1.0, confidence)

    def estimate_heart_rate(self):
        """Estimate heart rate with multi-method fusion and motion awareness"""
        if len(self.ppg_buffer) < self.sample_rate * 3:
            return self.last_hr

        # Check if valid signal is present (not just noise)
        if not self.check_signal_present():
            self.hr_confidence = 0.0
            self.last_hr = 0
            return 0

        # Check overall motion quality
        avg_quality = np.mean(list(self.quality_buffer)[-50:]) if len(self.quality_buffer) >= 50 else 0.5

        # If too much motion, return last known HR with reduced confidence
        if avg_quality < 0.3:
            self.hr_confidence = 0.2
            return self.last_hr

        try:
            # Get filtered signal
            filtered = self.get_filtered_signal()
            if len(filtered) < 50:
                return self.last_hr

            # Skip artifact portion for HR calculation
            buffer_len = len(self.ppg_buffer)
            if self.valid_segment_start > 0 and buffer_len > 0:
                skip_samples = max(0, self.valid_segment_start - (buffer_len - len(filtered)))
                if skip_samples > 0 and skip_samples < len(filtered) - 30:
                    filtered = filtered[skip_samples:]

            if len(filtered) < 30:
                return self.last_hr

            # Method 1: Direct peak detection
            hr_peaks, conf_peaks = self._find_peaks_direct(filtered)

            # Method 2: Autocorrelation
            hr_autocorr, conf_autocorr = self._find_peaks_autocorr(filtered)

            # Method 3: VPG zero-crossing (often most robust with noisy signals)
            hr_vpg, conf_vpg = self._find_peaks_vpg(filtered)

            # Fuse results from all three methods
            estimates = []
            weights = []
            methods = []  # Track which methods contributed

            if hr_peaks is not None and 40 <= hr_peaks <= 180:
                estimates.append(hr_peaks)
                weights.append(conf_peaks * avg_quality)
                methods.append('peak')

            if hr_autocorr is not None and 40 <= hr_autocorr <= 180:
                estimates.append(hr_autocorr)
                weights.append(conf_autocorr * avg_quality)
                methods.append('autocorr')

            if hr_vpg is not None and 40 <= hr_vpg <= 180:
                estimates.append(hr_vpg)
                weights.append(conf_vpg * avg_quality * 1.1)  # Slight VPG preference
                methods.append('vpg')

            if len(estimates) == 0:
                self.hr_confidence *= 0.9  # Decay confidence
                return self.last_hr

            # Multi-method fusion
            if len(estimates) >= 2:
                # Calculate pairwise agreements
                hr_array = np.array(estimates)
                hr_mean = np.mean(hr_array)
                hr_std = np.std(hr_array)

                # Agreement metric: low std relative to mean = good agreement
                if hr_mean > 0:
                    agreement = max(0, 1.0 - hr_std / hr_mean * 5)  # 5% std = 75% agreement
                else:
                    agreement = 0

                if agreement > 0.7:
                    # Methods agree well - use weighted average
                    total_weight = sum(weights)
                    hr_new = sum(e * w for e, w in zip(estimates, weights)) / total_weight
                    self.hr_confidence = min(1.0, agreement * max(weights) * 1.2)
                else:
                    # Methods disagree - prefer VPG if available, else highest confidence
                    if 'vpg' in methods and conf_vpg > 0.4:
                        vpg_idx = methods.index('vpg')
                        hr_new = estimates[vpg_idx]
                        self.hr_confidence = weights[vpg_idx] * 0.8
                    else:
                        best_idx = np.argmax(weights)
                        hr_new = estimates[best_idx]
                        self.hr_confidence = weights[best_idx] * 0.7
            else:
                hr_new = estimates[0]
                self.hr_confidence = weights[0]

            # Apply temporal smoothing (adaptive based on confidence)
            if self.last_hr > 0:
                # More smoothing when confidence is low
                alpha = 0.3 * self.hr_confidence + 0.1
                self.last_hr = int((1 - alpha) * self.last_hr + alpha * hr_new)
            else:
                self.last_hr = int(hr_new)

            # Update trend
            self.hr_history.append(self.last_hr)
            if len(self.hr_history) >= 5:
                recent = list(self.hr_history)[-5:]
                slope = (recent[-1] - recent[0]) / 4
                if slope > 2:
                    self.hr_trend = 1  # Increasing
                elif slope < -2:
                    self.hr_trend = -1  # Decreasing
                else:
                    self.hr_trend = 0  # Stable

        except Exception as e:
            # Silently handle errors
            pass

        return self.last_hr

    def get_hr_with_confidence(self):
        """Get heart rate along with confidence and trend"""
        hr = self.estimate_heart_rate()
        return hr, self.hr_confidence, self.hr_trend

    def get_motion_intensity(self):
        """Get current motion intensity for display"""
        return self.motion_detector.motion_intensity

    def record_systolic_peak(self, peak_sample_idx):
        """Legacy method - kept for compatibility. Use analyze_peaks() instead."""
        pass  # No longer used - we analyze full waveform now

    def analyze_peaks(self, peak_indices, sample_rate=12):
        """Analyze all detected peaks in the visible waveform to calculate BPM and HRV.

        This method takes all systolic peak indices found in the current display window
        and calculates heart rate metrics from the complete set of intervals.

        Args:
            peak_indices: Array of sample indices where systolic peaks were detected
            sample_rate: Samples per second (default 12 Hz for our PPG)
        """
        self.last_peak_count = len(peak_indices)

        if len(peak_indices) < 2:
            return

        # Calculate intervals between consecutive peaks (in seconds)
        intervals = []
        for i in range(1, len(peak_indices)):
            interval_samples = peak_indices[i] - peak_indices[i-1]
            interval_sec = interval_samples / sample_rate
            intervals.append(interval_sec)

        if len(intervals) == 0:
            return

        # Filter out anomalous intervals
        # Valid range: 300ms to 1500ms (40-200 BPM)
        min_interval = 0.3  # 300ms = 200 BPM max
        max_interval = 1.5  # 1500ms = 40 BPM min
        valid_intervals = [iv for iv in intervals if min_interval <= iv <= max_interval]

        self.valid_interval_count = len(valid_intervals)

        if len(valid_intervals) == 0:
            return

        # Calculate BPM from average interval
        avg_interval = sum(valid_intervals) / len(valid_intervals)
        if avg_interval > 0:
            bpm = 60.0 / avg_interval
            self.peak_based_bpm = int(max(40, min(200, bpm)))

        # Calculate HRV (RMSSD) from valid intervals
        self._update_hrv(valid_intervals)

    def _update_peak_based_bpm(self):
        """Calculate BPM from systolic peak intervals.

        Uses the last 10 valid intervals, ignoring anomalies (too short or too long).
        - Minimum interval: 300ms (200 BPM max)
        - Maximum interval: 2000ms (30 BPM min)
        """
        if len(self.systolic_peak_times) < 2:
            return

        # Calculate all intervals between peaks
        peak_times = list(self.systolic_peak_times)
        intervals = []
        for i in range(1, len(peak_times)):
            interval = peak_times[i] - peak_times[i-1]
            intervals.append(interval)

        if len(intervals) == 0:
            return

        # Filter out anomalous intervals
        # Valid range: 300ms to 2000ms (30-200 BPM)
        min_interval = 0.3  # 300ms = 200 BPM max
        max_interval = 2.0  # 2000ms = 30 BPM min
        valid_intervals = [iv for iv in intervals if min_interval <= iv <= max_interval]

        if len(valid_intervals) == 0:
            return

        # Use only the last 10 valid intervals for averaging
        recent_intervals = valid_intervals[-10:]

        # Calculate average interval and convert to BPM
        avg_interval = sum(recent_intervals) / len(recent_intervals)
        if avg_interval > 0:
            bpm = 60.0 / avg_interval
            # Clamp to reasonable range
            self.peak_based_bpm = int(max(30, min(200, bpm)))

        # Calculate HRV (RMSSD) from valid intervals
        self._update_hrv(valid_intervals)

    def _update_hrv(self, intervals):
        """Calculate HRV using RMSSD (Root Mean Square of Successive Differences).

        RMSSD is the most common time-domain HRV metric for short-term recordings.
        Normal resting RMSSD: 20-100ms (higher = better parasympathetic tone)
        """
        if len(intervals) < 3:
            return

        # Calculate successive differences
        successive_diffs = []
        for i in range(1, len(intervals)):
            diff = (intervals[i] - intervals[i-1]) * 1000  # Convert to ms
            successive_diffs.append(diff ** 2)

        if len(successive_diffs) > 0:
            # RMSSD = sqrt(mean of squared successive differences)
            mean_squared = sum(successive_diffs) / len(successive_diffs)
            self.hrv_rmssd = math.sqrt(mean_squared)

    def get_peak_based_bpm(self):
        """Get the current BPM calculated from systolic peak intervals."""
        return self.peak_based_bpm

    def get_hrv(self):
        """Get the current HRV (RMSSD in milliseconds)."""
        return self.hrv_rmssd


class StepCounter:
    """Step counter using accelerometer magnitude with adaptive threshold."""

    def __init__(self, sample_rate=25):
        self.sample_rate = sample_rate
        self.step_count = 0
        self.accel_buffer = deque(maxlen=75)  # ~3 seconds of data
        self.last_step_time = 0
        self.min_step_interval = 0.3  # Min 300ms between steps (max 200 steps/min)
        self.last_magnitude = 0
        self.baseline = 9.8  # Expected gravity magnitude in m/s²
        self.threshold_delta = 2.0  # Step detected when magnitude changes by this much
        self.peak_detected = False
        self.last_peak_mag = 0

    def add_sample(self, accel_x, accel_y, accel_z):
        """Add accelerometer sample and detect steps.

        Args:
            accel_x, accel_y, accel_z: Acceleration in m/s²
        """
        # Calculate magnitude
        magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        self.accel_buffer.append(magnitude)

        # Update baseline (running average)
        if len(self.accel_buffer) >= 10:
            self.baseline = sum(self.accel_buffer) / len(self.accel_buffer)

        # Step detection: look for peaks above baseline + threshold
        current_time = time.time()
        peak_threshold = self.baseline + self.threshold_delta

        # Detect rising edge above threshold
        if magnitude > peak_threshold and self.last_magnitude <= peak_threshold:
            self.peak_detected = True
            self.last_peak_mag = magnitude

        # Track the peak
        if self.peak_detected and magnitude > self.last_peak_mag:
            self.last_peak_mag = magnitude

        # Detect falling edge (step complete)
        if self.peak_detected and magnitude < peak_threshold:
            # Verify it was a significant peak
            if self.last_peak_mag > self.baseline + self.threshold_delta:
                if current_time - self.last_step_time >= self.min_step_interval:
                    self.step_count += 1
                    self.last_step_time = current_time
            self.peak_detected = False
            self.last_peak_mag = 0

        self.last_magnitude = magnitude

    def get_step_count(self):
        """Get total step count."""
        return self.step_count

    def reset(self):
        """Reset step counter."""
        self.step_count = 0
        self.accel_buffer.clear()
        self.baseline = 9.8


class BLESensorVisualizer:
    """Main visualizer class with auto-reconnection"""

    def __init__(self):
        self.client = None
        self.device_name = "nrf"
        self.device_address = None
        self.connected = False
        self.should_run = True
        self.reconnecting = False
        self.connection_status = "Disconnected"

        # Processing
        self.madgwick = MadgwickFilter(beta=0.1)
        self.hr_detector = HeartRateDetector(sample_rate=12)  # ~12.5Hz with nRF averaging
        self.step_counter = StepCounter(sample_rate=25)  # ~25Hz accelerometer data

        # Data buffers - use HR detector's buffer for PPG (single source of truth)
        self.time_buffer = deque(maxlen=250)  # ~20 seconds at 12.5Hz

        # Latest data
        self.latest_data = None
        self.battery_percent = 0
        self.charge_status = 0
        self.heart_rate = 0
        self.hr_confidence = 0.0
        self.hr_trend = 0  # -1 decreasing, 0 stable, 1 increasing
        self.spo2_percent = 0
        self.spo2_quality = 0
        self.last_valid_spo2 = 0  # Persist last valid SpO2 reading
        self.spo2_last_update = 0  # Timestamp of last valid reading
        self.skin_temp_c = 0.0
        self.hr_mode = True
        self.spo2_mode = False
        self.last_timestamp = 0

        # Stats
        self.packet_count = 0
        self.error_count = 0
        self.reconnect_count = 0
        self.last_packet_time = time.time()

        # Partial packet buffer for reassembly
        self.rx_buffer = bytearray()

        # Lock for thread safety
        self.data_lock = threading.Lock()

    def parse_packet(self, data: bytes) -> SensorData:
        """Parse a 32-byte sensor packet"""
        if len(data) != PACKET_SIZE:
            raise ValueError(f"Invalid packet size: {len(data)}")

        sync1, sync2 = data[0], data[1]
        if sync1 != SYNC1 or sync2 != SYNC2:
            raise ValueError(f"Invalid sync bytes: {sync1:02X} {sync2:02X}")

        # Verify checksum
        calc_checksum = sum(data[:-2]) & 0xFFFF
        pkt_checksum = struct.unpack('<H', data[30:32])[0]
        if calc_checksum != pkt_checksum:
            raise ValueError(f"Checksum mismatch: {calc_checksum} != {pkt_checksum}")

        # Parse fields (little-endian)
        accel_x, accel_y, accel_z = struct.unpack('<hhh', data[2:8])
        gyro_x, gyro_y, gyro_z = struct.unpack('<hhh', data[8:14])
        ppg_raw = struct.unpack('<I', data[14:18])[0]
        battery_percent = data[18]
        charge_status = data[19]
        timestamp_ms = struct.unpack('<I', data[20:24])[0]
        spo2_percent = data[24]
        heart_rate_ppg = data[25]
        signal_quality = data[26]
        skin_temp_c40 = data[27]  # Temperature + 40 offset
        mode_flags = data[28]     # bit0=HR, bit1=SpO2
        # byte 29 is reserved

        # Convert temperature: stored as C + 40 offset
        skin_temp_c = skin_temp_c40 - 40.0

        # Parse mode flags
        hr_mode = bool(mode_flags & 0x01)
        spo2_mode = bool(mode_flags & 0x02)

        return SensorData(
            accel_x=accel_x / 1000.0,  # Convert from milli-units
            accel_y=accel_y / 1000.0,
            accel_z=accel_z / 1000.0,
            gyro_x=gyro_x / 1000.0,
            gyro_y=gyro_y / 1000.0,
            gyro_z=gyro_z / 1000.0,
            ppg_raw=ppg_raw,
            battery_percent=battery_percent,
            charge_status=charge_status,
            timestamp_ms=timestamp_ms,
            spo2_percent=spo2_percent,
            heart_rate_ppg=heart_rate_ppg,
            signal_quality=signal_quality,
            skin_temp_c=skin_temp_c,
            hr_mode=hr_mode,
            spo2_mode=spo2_mode
        )

    def notification_handler(self, sender, data: bytearray):
        """Handle incoming BLE notifications"""
        with self.data_lock:
            # Add to buffer
            self.rx_buffer.extend(data)

            # Process complete packets
            while len(self.rx_buffer) >= PACKET_SIZE:
                # Look for sync bytes
                sync_idx = -1
                for i in range(len(self.rx_buffer) - 1):
                    if self.rx_buffer[i] == SYNC1 and self.rx_buffer[i+1] == SYNC2:
                        sync_idx = i
                        break

                if sync_idx < 0:
                    # No sync found, clear buffer
                    self.rx_buffer.clear()
                    break

                if sync_idx > 0:
                    # Discard bytes before sync
                    del self.rx_buffer[:sync_idx]

                if len(self.rx_buffer) < PACKET_SIZE:
                    break

                # Extract packet
                packet_data = bytes(self.rx_buffer[:PACKET_SIZE])
                del self.rx_buffer[:PACKET_SIZE]

                try:
                    sensor_data = self.parse_packet(packet_data)
                    self.process_data(sensor_data)
                    self.packet_count += 1
                    self.last_packet_time = time.time()
                except ValueError as e:
                    self.error_count += 1

    def process_data(self, data: SensorData):
        """Process incoming sensor data"""
        self.latest_data = data

        # Update battery
        self.battery_percent = data.battery_percent
        self.charge_status = data.charge_status

        # Update SpO2 with persistence
        self.spo2_percent = data.spo2_percent
        self.spo2_quality = data.signal_quality

        # Persist valid SpO2 readings (70-100% range indicates valid measurement)
        if 70 <= data.spo2_percent <= 100:
            self.last_valid_spo2 = data.spo2_percent
            self.spo2_last_update = time.time()

        # Update temperature and modes
        self.skin_temp_c = data.skin_temp_c
        self.hr_mode = data.hr_mode
        self.spo2_mode = data.spo2_mode

        # Update orientation filter
        accel = (data.accel_x, data.accel_y, data.accel_z)
        gyro = (data.gyro_x, data.gyro_y, data.gyro_z)

        if self.last_timestamp > 0:
            dt = (data.timestamp_ms - self.last_timestamp) / 1000.0
            if 0 < dt < 0.2:  # Sanity check
                self.madgwick.update(accel, gyro, dt)
        self.last_timestamp = data.timestamp_ms

        # Pass IMU data to HR detector for motion artifact detection
        self.hr_detector.add_imu_sample(accel, gyro)

        # Update step counter with accelerometer data
        self.step_counter.add_sample(data.accel_x, data.accel_y, data.accel_z)

        # Update PPG and heart rate (HR detector owns the PPG buffer)
        if data.ppg_raw > 0:
            self.time_buffer.append(data.timestamp_ms / 1000.0)
            self.hr_detector.add_sample(data.ppg_raw)
            self.heart_rate, self.hr_confidence, self.hr_trend = self.hr_detector.get_hr_with_confidence()

    def disconnected_callback(self, client):
        """Called when BLE disconnects"""
        print(f"\n[!] Disconnected from device!")
        self.connected = False
        self.connection_status = "Disconnected - Reconnecting..."

    async def find_device(self):
        """Scan for the nRF device"""
        print(f"Scanning for BLE device '{self.device_name}'...")

        try:
            devices = await BleakScanner.discover(timeout=5.0)
            for device in devices:
                name = device.name or ""
                if self.device_name.lower() in name.lower():
                    print(f"Found device: {device.name} ({device.address})")
                    return device
        except Exception as e:
            print(f"Scan error: {e}")

        return None

    async def connect(self):
        """Connect to the BLE device"""
        device = await self.find_device()
        if not device:
            return False

        self.device_address = device.address
        return await self.connect_to_address(device.address)

    async def connect_to_address(self, address):
        """Connect to a specific BLE address"""
        print(f"Connecting to {address}...")
        self.connection_status = f"Connecting to {address}..."

        try:
            self.client = BleakClient(
                address,
                disconnected_callback=self.disconnected_callback
            )

            await asyncio.wait_for(self.client.connect(), timeout=10.0)

            if not self.client.is_connected:
                print("Connection failed - not connected")
                return False

            print("Connected!")
            self.connection_status = "Connected"

            # Clear buffers on new connection
            self.rx_buffer.clear()

            # Subscribe to notifications
            await self.client.start_notify(NUS_TX_UUID, self.notification_handler)
            print("Subscribed to sensor data stream")

            # Send START command
            await self.client.write_gatt_char(NUS_RX_UUID, b"START")
            print("Sent START command")

            self.connected = True
            self.last_packet_time = time.time()
            return True

        except asyncio.TimeoutError:
            print("Connection timed out")
            self.connection_status = "Connection timed out"
        except BleakError as e:
            print(f"BLE error: {e}")
            self.connection_status = f"BLE error: {e}"
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connection_status = f"Error: {e}"

        return False

    async def reconnect(self):
        """Attempt to reconnect to the device"""
        if self.reconnecting:
            return False

        self.reconnecting = True
        self.reconnect_count += 1
        print(f"\n[*] Reconnection attempt #{self.reconnect_count}...")

        # Cleanup old connection
        if self.client:
            try:
                await self.client.disconnect()
            except Exception:
                pass
            self.client = None

        # Wait before reconnecting
        await asyncio.sleep(RECONNECT_DELAY)

        # Try to reconnect
        success = False
        if self.device_address:
            # Try last known address first
            success = await self.connect_to_address(self.device_address)

        if not success:
            # Scan for device again
            success = await self.connect()

        self.reconnecting = False
        return success

    async def disconnect(self):
        """Disconnect from device"""
        self.should_run = False
        if self.client and self.client.is_connected:
            try:
                await self.client.write_gatt_char(NUS_RX_UUID, b"STOP")
            except Exception:
                pass
            try:
                await self.client.disconnect()
            except Exception:
                pass
        self.connected = False

    def check_connection_health(self):
        """Check if connection is healthy (receiving data)"""
        if not self.connected:
            return False
        # If no packets for 3 seconds, connection might be stale
        return (time.time() - self.last_packet_time) < 3.0

    async def send_command(self, command: str):
        """Send a command to the device"""
        if self.client and self.client.is_connected:
            try:
                await self.client.write_gatt_char(NUS_RX_UUID, command.encode())
                print(f"Sent command: {command}")
                return True
            except Exception as e:
                print(f"Failed to send command: {e}")
        return False

    async def toggle_spo2(self):
        """Toggle SpO2 mode"""
        if self.spo2_mode:
            await self.send_command("SPO2_OFF")
        else:
            await self.send_command("SPO2_ON")

    async def toggle_hr(self):
        """Toggle HR mode"""
        if self.hr_mode:
            await self.send_command("HR_OFF")
        else:
            await self.send_command("HR_ON")

    @property
    def ppg_sample_count(self):
        """Get the number of PPG samples collected"""
        return len(self.hr_detector.ppg_buffer)

def create_box_vertices(size=1.0):
    """Create vertices for a 3D box"""
    s = size / 2
    vertices = np.array([
        [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
        [-s, -s, s], [s, -s, s], [s, s, s], [-s, s, s]
    ])
    return vertices

def create_box_faces(vertices):
    """Create faces for a 3D box"""
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # left
        [vertices[1], vertices[2], vertices[6], vertices[5]],  # right
    ]
    return faces

async def run_visualizer():
    """Main visualization loop with auto-reconnection"""
    visualizer = BLESensorVisualizer()

    # Initial connection
    if not await visualizer.connect():
        print("\nInitial connection failed. Will keep trying...")

    # Set up matplotlib with dark theme - Clean ProtoCentral-style layout
    plt.style.use('dark_background')
    plt.ion()
    fig = plt.figure(figsize=(14, 8), facecolor='#0d1117')
    fig.suptitle('PPG Heart Rate Monitor', fontsize=16, color='#58a6ff', y=0.98, fontweight='bold')

    # Clean layout: large PPG waveform on top, stats panel below
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.25, wspace=0.3,
                          height_ratios=[2.5, 1],
                          width_ratios=[0.8, 2, 0.8])

    # Dark theme colors (GitHub dark style)
    bg_color = '#0d1117'
    plot_bg = '#161b22'
    grid_color = '#21262d'
    text_color = '#c9d1d9'
    accent_cyan = '#58a6ff'
    accent_green = '#3fb950'
    accent_red = '#f85149'
    accent_orange = '#d29922'
    accent_yellow = '#e3b341'
    accent_magenta = '#bc8cff'
    ppg_color = '#39d353'  # Bright green for PPG waveform (like ECG monitors)

    # ===== MAIN PPG WAVEFORM (top, spans all columns) =====
    ax_ppg = fig.add_subplot(gs[0, :], facecolor=plot_bg)
    ax_ppg.set_title('PPG Waveform', color=text_color, fontsize=12, pad=10)
    ax_ppg.set_xlabel('Samples', color=text_color, fontsize=10)
    ax_ppg.set_ylabel('Amplitude', color=text_color, fontsize=10)
    ax_ppg.tick_params(colors=text_color, labelsize=9)
    ax_ppg.grid(True, alpha=0.15, color=grid_color, linewidth=0.5)
    ax_ppg.set_facecolor(plot_bg)
    # Clean border
    for spine in ax_ppg.spines.values():
        spine.set_color(grid_color)
        spine.set_linewidth(0.5)

    # 3D orientation plot (bottom left - smaller)
    ax_3d = fig.add_subplot(gs[1, 0], projection='3d', facecolor=plot_bg)
    ax_3d.set_xlim([-1.5, 1.5])
    ax_3d.set_ylim([-1.5, 1.5])
    ax_3d.set_zlim([-1.5, 1.5])
    ax_3d.tick_params(colors=text_color, labelsize=6)
    ax_3d.set_xlabel('', fontsize=1)
    ax_3d.set_ylabel('', fontsize=1)
    ax_3d.set_zlabel('', fontsize=1)

    # Status panel (bottom center)
    ax_status = fig.add_subplot(gs[1, 1], facecolor=plot_bg)
    ax_status.axis('off')
    status_text = ax_status.text(0.02, 0.95, '', transform=ax_status.transAxes,
                                  fontsize=11, verticalalignment='top',
                                  fontfamily='monospace', color=text_color,
                                  linespacing=1.4)

    # HR/SpO2 display panel (bottom right)
    ax_vitals = fig.add_subplot(gs[1, 2], facecolor=plot_bg)
    ax_vitals.axis('off')
    vitals_text = ax_vitals.text(0.5, 0.5, '', transform=ax_vitals.transAxes,
                                  fontsize=24, verticalalignment='center',
                                  horizontalalignment='center',
                                  fontfamily='monospace', color=accent_green,
                                  fontweight='bold')

    # ===== CLEAN PPG WAVEFORM SETUP =====
    # Main PPG line - bright green like medical monitors
    ppg_line, = ax_ppg.plot([], [], color=ppg_color, linewidth=1.5, antialiased=True)

    # Heartbeat peak markers - small red dots at systolic peaks
    peak_marker, = ax_ppg.plot([], [], 'o', color=accent_red, markersize=6,
                                markeredgecolor='white', markeredgewidth=0.5)

    # Display window: 100 samples (~8 sec at 12 Hz) - good balance of detail vs smoothness
    display_samples = 100
    ax_ppg.set_xlim(0, display_samples)
    # Fixed Y-axis: normalized signal typically stays within [-3, 3]
    # Fixed limits prevent jarring visual jumps when amplitude changes
    ax_ppg.set_ylim(-3.5, 3.5)

    # "No Signal" text overlay
    no_signal_text = ax_ppg.text(0.5, 0.5, '', transform=ax_ppg.transAxes,
                                  fontsize=20, ha='center', va='center',
                                  color=accent_red, fontweight='bold', alpha=0.9)

    # Control buttons
    btn_color = '#21262d'
    btn_hover = '#30363d'

    # SpO2 toggle button
    ax_btn_spo2 = fig.add_axes([0.02, 0.02, 0.08, 0.035])
    btn_spo2 = Button(ax_btn_spo2, 'SpO2', color=btn_color, hovercolor=btn_hover)
    btn_spo2.label.set_color(text_color)
    btn_spo2.label.set_fontsize(9)

    # HR toggle button
    ax_btn_hr = fig.add_axes([0.11, 0.02, 0.08, 0.035])
    btn_hr = Button(ax_btn_hr, 'HR', color=btn_color, hovercolor=btn_hover)
    btn_hr.label.set_color(text_color)
    btn_hr.label.set_fontsize(9)

    # Button click handlers
    def on_spo2_click(event):
        asyncio.create_task(visualizer.toggle_spo2())

    def on_hr_click(event):
        asyncio.create_task(visualizer.toggle_hr())

    btn_spo2.on_clicked(on_spo2_click)
    btn_hr.on_clicked(on_hr_click)

    # Box vertices for 3D orientation
    base_vertices = create_box_vertices(1.0)

    # Store theme colors for update function
    dark_theme = {
        'bg': plot_bg, 'grid': grid_color, 'text': text_color,
        'cyan': accent_cyan, 'green': accent_green, 'red': accent_red,
        'orange': accent_orange, 'ppg': ppg_color
    }

    def update_plot(frame):
        """Update all plots - clean ProtoCentral-style display"""
        with visualizer.data_lock:
            # ===== UPDATE 3D ORIENTATION (smaller, bottom left) =====
            ax_3d.cla()
            ax_3d.set_facecolor(dark_theme['bg'])
            ax_3d.set_xlim([-1.5, 1.5])
            ax_3d.set_ylim([-1.5, 1.5])
            ax_3d.set_zlim([-1.5, 1.5])
            ax_3d.tick_params(colors=dark_theme['text'], labelsize=6)
            ax_3d.xaxis.pane.fill = False
            ax_3d.yaxis.pane.fill = False
            ax_3d.zaxis.pane.fill = False
            ax_3d.xaxis.pane.set_edgecolor(dark_theme['grid'])
            ax_3d.yaxis.pane.set_edgecolor(dark_theme['grid'])
            ax_3d.zaxis.pane.set_edgecolor(dark_theme['grid'])

            # Rotate box by current orientation
            R = visualizer.madgwick.get_rotation_matrix()
            rotated_vertices = (R @ base_vertices.T).T
            faces = create_box_faces(rotated_vertices)

            # Draw box
            box_colors = [dark_theme['cyan'], dark_theme['cyan'],
                         '#0066aa', '#0066aa',
                         dark_theme['green'], dark_theme['red']]
            face_collection = Poly3DCollection(faces, alpha=0.7)
            face_collection.set_facecolors(box_colors)
            face_collection.set_edgecolor('#404060')
            ax_3d.add_collection3d(face_collection)

            # ===== REAL PPG WAVEFORM with Peak-Aligned Normalization =====
            ppg_data = visualizer.hr_detector.ppg_buffer
            signal_present = visualizer.hr_detector.check_signal_present()

            # Target Y values for normalized display (fixed axis)
            PEAK_Y = 2.5      # All peaks align here
            BASELINE_Y = 0.0  # Baseline/minimum

            # Display delay for stable processing
            delay_samples = 24  # 2 seconds at 12Hz

            if len(ppg_data) > display_samples + delay_samples and signal_present:
                try:
                    # 1. Filter the FULL buffer for stable results
                    raw_data = np.array(list(ppg_data), dtype=float)
                    centered = raw_data - np.mean(raw_data)

                    if len(centered) >= 30:
                        sos = signal.butter(2, [0.5, 5], btype='band', fs=12, output='sos')
                        filtered = signal.sosfiltfilt(sos, centered)
                    else:
                        filtered = centered

                    # 2. Detect peaks on full buffer
                    all_peaks, _ = signal.find_peaks(filtered, distance=5, prominence=0.05)

                    # 3. Beat-to-beat normalization on full buffer
                    if len(all_peaks) >= 2:
                        ppg_normalized = filtered.copy()

                        for i in range(len(all_peaks)):
                            seg_start = 0 if i == 0 else (all_peaks[i-1] + all_peaks[i]) // 2
                            seg_end = len(filtered) if i == len(all_peaks) - 1 else (all_peaks[i] + all_peaks[i+1]) // 2

                            segment = filtered[seg_start:seg_end]
                            if len(segment) > 0:
                                seg_min, seg_max = np.min(segment), np.max(segment)
                                seg_range = seg_max - seg_min
                                if seg_range > 0:
                                    ppg_normalized[seg_start:seg_end] = (segment - seg_min) / seg_range * (PEAK_Y - BASELINE_Y) + BASELINE_Y
                    else:
                        # Simple normalization if not enough peaks
                        pmin, pmax = np.min(filtered), np.max(filtered)
                        if pmax - pmin > 0:
                            ppg_normalized = (filtered - pmin) / (pmax - pmin) * (PEAK_Y - BASELINE_Y) + BASELINE_Y
                        else:
                            ppg_normalized = filtered + (PEAK_Y + BASELINE_Y) / 2

                    # 4. Extract display window WITH DELAY
                    end_idx = len(ppg_normalized) - delay_samples
                    start_idx = max(0, end_idx - display_samples)
                    ppg_display = ppg_normalized[start_idx:end_idx]

                    # Find peaks within display window
                    display_peaks = all_peaks[(all_peaks >= start_idx) & (all_peaks < end_idx)] - start_idx

                    # 5. Display (fixed axis)
                    samples = np.arange(len(ppg_display))
                    ppg_line.set_data(samples, ppg_display)
                    ax_ppg.set_xlim(0, display_samples)
                    ax_ppg.set_ylim(-0.5, 3.0)

                    # Mark peaks
                    if len(display_peaks) > 0:
                        peak_marker.set_data(display_peaks, ppg_display[display_peaks])
                        visualizer.hr_detector.analyze_peaks(all_peaks, sample_rate=12)
                    else:
                        peak_marker.set_data([], [])

                    no_signal_text.set_text('')

                except Exception:
                    ppg_line.set_data([], [])
                    peak_marker.set_data([], [])
            else:
                ppg_line.set_data([], [])
                peak_marker.set_data([], [])
                if not signal_present and len(ppg_data) > 20:
                    no_signal_text.set_text('NO SIGNAL')
                elif signal_present and len(ppg_data) > 0:
                    pct = int(len(ppg_data) * 100 / (display_samples + delay_samples))
                    no_signal_text.set_text(f'Buffering... {pct}%')
                else:
                    no_signal_text.set_text('Place finger on sensor')

            # ===== UPDATE STATUS PANEL =====
            roll, pitch, yaw = visualizer.madgwick.get_euler()
            charge_state = visualizer.charge_status & 0x03
            usb_present = (visualizer.charge_status & 0x04) != 0

            if usb_present:
                pwr_str = ['USB', 'Charging', 'Full', 'Err'][charge_state]
            else:
                pwr_str = 'Battery' if charge_state != 3 else 'Err'

            # Connection status
            if visualizer.connected and visualizer.check_connection_health():
                conn_str = "Connected"
                conn_color = dark_theme['green']
            elif visualizer.reconnecting:
                conn_str = "Reconnecting..."
                conn_color = dark_theme['orange']
            else:
                conn_str = "Disconnected"
                conn_color = dark_theme['red']

            # Heart rate
            peak_bpm = visualizer.hr_detector.get_peak_based_bpm()
            num_intervals = visualizer.hr_detector.valid_interval_count

            # HRV
            hrv = visualizer.hr_detector.get_hrv()
            hrv_str = f"{hrv:.0f}" if hrv > 0 else "--"

            # SpO2
            if visualizer.last_valid_spo2 >= 70:
                spo2_age = time.time() - visualizer.spo2_last_update
                if spo2_age < 30:
                    spo2_str = f"{visualizer.last_valid_spo2}%"
                else:
                    spo2_str = f"({visualizer.last_valid_spo2}%)"
            else:
                spo2_str = "--"

            # Temperature
            temp_str = f"{visualizer.skin_temp_c:.1f}" if visualizer.skin_temp_c > 0 else "--"

            # Steps
            steps = visualizer.step_counter.get_step_count()

            # Motion indicator
            motion = visualizer.hr_detector.get_motion_intensity()
            motion_bar = "=" * min(10, int(motion * 10))

            # Clean compact status
            status = f"""{conn_str}

Battery: {visualizer.battery_percent}% ({pwr_str})
Temp:    {temp_str}C
Steps:   {steps}
HRV:     {hrv_str} ms
SpO2:    {spo2_str}

Motion:  [{motion_bar:<10}]
Packets: {visualizer.packet_count}"""

            status_text.set_text(status)

            # ===== UPDATE VITALS DISPLAY (large HR number) =====
            if peak_bpm > 0 and num_intervals >= 2:
                vitals_text.set_text(f"{peak_bpm}\nBPM")
                vitals_text.set_color(dark_theme['green'])
            else:
                vitals_text.set_text("--\nBPM")
                vitals_text.set_color(dark_theme['text'])

            # Update button colors based on mode state
            if visualizer.spo2_mode:
                ax_btn_spo2.set_facecolor(dark_theme['green'])
                btn_spo2.label.set_text('SpO2 ON')
            else:
                ax_btn_spo2.set_facecolor(btn_color)
                btn_spo2.label.set_text('SpO2 OFF')

            if visualizer.hr_mode:
                ax_btn_hr.set_facecolor(dark_theme['green'])
                btn_hr.label.set_text('HR ON')
            else:
                ax_btn_hr.set_facecolor(btn_color)
                btn_hr.label.set_text('HR OFF')

        return [ppg_line, peak_marker, status_text, vitals_text]

    # Animation - 33ms interval for ~30 FPS (smoother scrolling waveform)
    ani = FuncAnimation(fig, update_plot, interval=33, blit=False, cache_frame_data=False)

    print("\nVisualization running! Close the window to stop.")
    print("Auto-reconnection is enabled.")
    print("Press Ctrl+C to exit.\n")

    try:
        while visualizer.should_run and plt.fignum_exists(fig.number):
            # Check connection health and reconnect if needed
            if not visualizer.connected or not visualizer.check_connection_health():
                if not visualizer.reconnecting:
                    # Start reconnection in background
                    asyncio.create_task(visualizer.reconnect())

            plt.pause(0.05)
            await asyncio.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        visualizer.should_run = False
        await visualizer.disconnect()
        plt.close('all')
        print("Disconnected.")

def main():
    """Entry point"""
    print("\033[36m" + "=" * 55 + "\033[0m")
    print("\033[36m  nRF52840 BLE Sensor Visualizer\033[0m")
    print("\033[90m  Dark Mode | IMU Motion Artifact Removal | Multi-Method HR\033[0m")
    print("\033[36m" + "=" * 55 + "\033[0m")
    print()

    # Run the async visualizer
    try:
        asyncio.run(run_visualizer())
    except KeyboardInterrupt:
        print("\n\033[33mExiting...\033[0m")

if __name__ == "__main__":
    main()
