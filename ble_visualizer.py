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

class HeartRateDetector:
    """Heart rate detection from PPG signal with continuous 5-second rolling buffer"""

    def __init__(self, sample_rate=25):
        self.sample_rate = sample_rate
        self.window_seconds = 5  # 5-second continuous rolling window
        self.buffer_size = sample_rate * self.window_seconds  # 125 samples
        self.ppg_buffer = deque(maxlen=self.buffer_size)
        self.last_hr = 0
        self.filter_initialized = False
        self.b = None
        self.a = None

        # Artifact detection - simple threshold-based
        self.last_valid_ppg = None
        self.artifact_threshold = 0.4  # 40% relative change threshold
        self.artifact_count = 0

        # Running mean for baseline (slow adaptation)
        self.running_mean = None
        self.mean_alpha = 0.02  # Very slow adaptation for baseline

        # Initialize bandpass filter for heart rate (0.8 - 3.0 Hz = 48-180 BPM)
        try:
            self.b, self.a = signal.butter(2, [0.8, 3.0], btype='band', fs=sample_rate)
            self.filter_initialized = True
        except Exception as e:
            print(f"Filter init error: {e}")

    def _is_artifact(self, ppg_value):
        """Detect if a sample is likely an artifact (large spike)"""
        if self.last_valid_ppg is None:
            return False

        # Simple relative change check
        relative_change = abs(ppg_value - self.last_valid_ppg) / max(self.last_valid_ppg, 1)
        return relative_change > self.artifact_threshold

    def add_sample(self, ppg_value):
        """Add a new PPG sample with simple artifact rejection"""
        if ppg_value <= 0:
            return

        # Check for artifact (large spike)
        if self._is_artifact(ppg_value):
            self.artifact_count += 1
            # After a few artifacts, accept new baseline
            if self.artifact_count >= 3:
                self.last_valid_ppg = ppg_value
                self.running_mean = ppg_value
                self.artifact_count = 0
            return  # Skip this sample

        # Valid sample - add to continuous buffer
        self.artifact_count = 0
        self.last_valid_ppg = ppg_value

        # Update running mean (slow adaptation)
        if self.running_mean is None:
            self.running_mean = ppg_value
        else:
            self.running_mean = (1 - self.mean_alpha) * self.running_mean + self.mean_alpha * ppg_value

        # Add to buffer (old samples automatically drop off due to maxlen)
        self.ppg_buffer.append(float(ppg_value))

    def get_filtered_signal(self):
        """Get bandpass filtered PPG signal with additional smoothing"""
        if len(self.ppg_buffer) < 25:
            # Return normalized raw data if not enough samples
            data = np.array(list(self.ppg_buffer), dtype=float)
            if len(data) > 0:
                return data - np.mean(data)
            return np.array([])

        data = np.array(list(self.ppg_buffer), dtype=float)

        # Remove DC component
        data = data - np.mean(data)

        # Normalize
        std = np.std(data)
        if std > 0:
            data = data / std

        # Apply bandpass filter if initialized and enough data
        if self.filter_initialized and len(data) >= 50:
            try:
                # Use filtfilt with padlen to avoid edge effects
                padlen = min(3 * max(len(self.b), len(self.a)), len(data) - 1)
                if padlen > 0:
                    filtered = signal.filtfilt(self.b, self.a, data, padlen=padlen)

                    # Additional smoothing with moving average (5-sample window)
                    # This reduces jitter in the display
                    window_size = 5
                    if len(filtered) >= window_size:
                        kernel = np.ones(window_size) / window_size
                        # Use 'same' mode to keep the same length
                        filtered = np.convolve(filtered, kernel, mode='same')

                    return filtered
            except Exception:
                pass

        return data

    def estimate_heart_rate(self):
        """Estimate heart rate from PPG buffer"""
        if len(self.ppg_buffer) < self.sample_rate * 3:  # Need at least 3 seconds
            return self.last_hr

        try:
            data = np.array(list(self.ppg_buffer), dtype=float)

            # Remove DC and normalize
            data = data - np.mean(data)
            std = np.std(data)
            if std > 0:
                data = data / std

            # Apply bandpass filter
            if self.filter_initialized and len(data) >= 50:
                padlen = min(3 * max(len(self.b), len(self.a)), len(data) - 1)
                if padlen > 0:
                    filtered = signal.filtfilt(self.b, self.a, data, padlen=padlen)
                else:
                    filtered = data
            else:
                filtered = data

            if len(filtered) < 50:
                return self.last_hr

            # Find peaks
            std = np.std(filtered)
            peaks, _ = signal.find_peaks(
                filtered,
                distance=int(self.sample_rate * 0.4),  # Min 0.4s between beats (~150 BPM max)
                prominence=std * 0.25 if std > 0 else 0.1
            )

            if len(peaks) >= 2:
                intervals = np.diff(peaks)
                # Keep reasonable intervals (0.4-1.5 seconds = 40-150 BPM)
                valid_intervals = intervals[(intervals > self.sample_rate * 0.4) &
                                            (intervals < self.sample_rate * 1.5)]

                if len(valid_intervals) >= 1:
                    avg_interval = np.median(valid_intervals)
                    hr = 60 * self.sample_rate / avg_interval

                    if 40 <= hr <= 150:
                        # Smooth the reading
                        if self.last_hr > 0:
                            self.last_hr = int(0.8 * self.last_hr + 0.2 * hr)
                        else:
                            self.last_hr = int(hr)
        except Exception:
            pass

        return self.last_hr

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
        self.hr_detector = HeartRateDetector(sample_rate=25)

        # Data buffers - use HR detector's buffer for PPG (single source of truth)
        self.time_buffer = deque(maxlen=500)  # ~20 seconds at 25Hz

        # Latest data
        self.latest_data = None
        self.battery_percent = 0
        self.charge_status = 0
        self.heart_rate = 0
        self.spo2_percent = 0
        self.spo2_quality = 0
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

        # Update SpO2
        self.spo2_percent = data.spo2_percent
        self.spo2_quality = data.signal_quality

        # Update temperature and modes
        self.skin_temp_c = data.skin_temp_c
        self.hr_mode = data.hr_mode
        self.spo2_mode = data.spo2_mode

        # Update orientation filter
        if self.last_timestamp > 0:
            dt = (data.timestamp_ms - self.last_timestamp) / 1000.0
            if 0 < dt < 0.2:  # Sanity check
                accel = (data.accel_x, data.accel_y, data.accel_z)
                gyro = (data.gyro_x, data.gyro_y, data.gyro_z)
                self.madgwick.update(accel, gyro, dt)
        self.last_timestamp = data.timestamp_ms

        # Update PPG and heart rate (HR detector owns the PPG buffer)
        if data.ppg_raw > 0:
            self.time_buffer.append(data.timestamp_ms / 1000.0)
            self.hr_detector.add_sample(data.ppg_raw)
            self.heart_rate = self.hr_detector.estimate_heart_rate()

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

    # Set up matplotlib with dark theme
    plt.style.use('dark_background')
    plt.ion()
    fig = plt.figure(figsize=(14, 8), facecolor='#1a1a2e')
    fig.suptitle('nRF52840 Sensor Visualization', fontsize=14, color='#00d4ff')

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)

    # Dark theme colors
    bg_color = '#1a1a2e'
    grid_color = '#2a2a4e'
    text_color = '#e0e0e0'
    accent_cyan = '#00d4ff'
    accent_green = '#00ff88'
    accent_red = '#ff4466'
    accent_orange = '#ffaa00'

    # 3D orientation plot
    ax_3d = fig.add_subplot(gs[0, 0], projection='3d', facecolor=bg_color)
    ax_3d.set_title('Device Orientation', color=text_color)
    ax_3d.set_xlim([-1.5, 1.5])
    ax_3d.set_ylim([-1.5, 1.5])
    ax_3d.set_zlim([-1.5, 1.5])
    ax_3d.set_xlabel('X', color=text_color)
    ax_3d.set_ylabel('Y', color=text_color)
    ax_3d.set_zlabel('Z', color=text_color)
    ax_3d.tick_params(colors=text_color)

    # PPG raw signal
    ax_ppg_raw = fig.add_subplot(gs[0, 1:], facecolor=bg_color)
    ax_ppg_raw.set_title('PPG Raw Signal', color=text_color)
    ax_ppg_raw.set_xlabel('Time (s)', color=text_color)
    ax_ppg_raw.set_ylabel('ADC Value', color=text_color)
    ax_ppg_raw.tick_params(colors=text_color)
    ax_ppg_raw.grid(True, alpha=0.3, color=grid_color)
    ppg_raw_line, = ax_ppg_raw.plot([], [], color=accent_cyan, linewidth=1.0)

    # PPG filtered signal
    ax_ppg_filt = fig.add_subplot(gs[1, 1:], facecolor=bg_color)
    ax_ppg_filt.set_title('PPG Filtered (Heartbeat)', color=text_color)
    ax_ppg_filt.set_xlabel('Time (s)', color=text_color)
    ax_ppg_filt.set_ylabel('Amplitude', color=text_color)
    ax_ppg_filt.tick_params(colors=text_color)
    ax_ppg_filt.grid(True, alpha=0.3, color=grid_color)
    ppg_filt_line, = ax_ppg_filt.plot([], [], color=accent_red, linewidth=1.5)

    # Status panel
    ax_status = fig.add_subplot(gs[1, 0], facecolor=bg_color)
    ax_status.axis('off')
    status_text = ax_status.text(0.1, 0.95, '', transform=ax_status.transAxes,
                                  fontsize=10, verticalalignment='top',
                                  fontfamily='monospace', color=text_color)

    # Box vertices
    base_vertices = create_box_vertices(1.0)

    # Store dark theme colors for update function
    dark_theme = {
        'bg': bg_color, 'grid': grid_color, 'text': text_color,
        'cyan': accent_cyan, 'green': accent_green, 'red': accent_red, 'orange': accent_orange
    }

    def update_plot(frame):
        """Update all plots"""
        nonlocal ppg_raw_line, ppg_filt_line

        with visualizer.data_lock:
            # Update 3D orientation with dark theme
            ax_3d.cla()
            ax_3d.set_facecolor(dark_theme['bg'])
            ax_3d.set_title('Device Orientation', color=dark_theme['text'])
            ax_3d.set_xlim([-1.5, 1.5])
            ax_3d.set_ylim([-1.5, 1.5])
            ax_3d.set_zlim([-1.5, 1.5])
            ax_3d.set_xlabel('X', color=dark_theme['text'])
            ax_3d.set_ylabel('Y', color=dark_theme['text'])
            ax_3d.set_zlabel('Z', color=dark_theme['text'])
            ax_3d.tick_params(colors=dark_theme['text'])
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

            # Draw box with dark theme colors
            box_colors = [dark_theme['cyan'], dark_theme['cyan'],
                         '#0066aa', '#0066aa',
                         dark_theme['green'], dark_theme['red']]
            face_collection = Poly3DCollection(faces, alpha=0.7)
            face_collection.set_facecolors(box_colors)
            face_collection.set_edgecolor('#404060')
            ax_3d.add_collection3d(face_collection)

            # Draw coordinate axes
            axis_len = 1.2
            ax_3d.quiver(0, 0, 0, axis_len, 0, 0, color=dark_theme['red'], arrow_length_ratio=0.1)
            ax_3d.quiver(0, 0, 0, 0, axis_len, 0, color=dark_theme['green'], arrow_length_ratio=0.1)
            ax_3d.quiver(0, 0, 0, 0, 0, axis_len, color=dark_theme['cyan'], arrow_length_ratio=0.1)

            # Update PPG plots (continuous 5-second rolling window)
            ppg_data = visualizer.hr_detector.ppg_buffer
            if len(ppg_data) > 10:
                # Get raw PPG data
                ppg_raw = np.array(list(ppg_data), dtype=float)

                # Create time axis - shows last N seconds of data
                times = np.arange(len(ppg_raw)) / 25.0  # 25Hz sample rate

                ppg_raw_line.set_data(times, ppg_raw)
                # Fixed 5-second x-axis, data fills in from left
                ax_ppg_raw.set_xlim(0, 5.0)
                ax_ppg_raw.relim()
                ax_ppg_raw.autoscale_view(scalex=False)

                # Filtered signal
                try:
                    filtered = visualizer.hr_detector.get_filtered_signal()
                    if len(filtered) > 0:
                        filt_times = np.arange(len(filtered)) / 25.0
                        ppg_filt_line.set_data(filt_times, filtered)
                        ax_ppg_filt.set_xlim(0, 5.0)
                        ax_ppg_filt.relim()
                        ax_ppg_filt.autoscale_view(scalex=False)
                except Exception:
                    pass

            # Update status text
            roll, pitch, yaw = visualizer.madgwick.get_euler()
            charge_status_str = ['Ready', 'Charging', 'Done', 'Fault'][visualizer.charge_status % 4]

            # Connection status indicator
            if visualizer.connected and visualizer.check_connection_health():
                conn_indicator = "CONNECTED"
                conn_color = dark_theme['green']
            elif visualizer.reconnecting:
                conn_indicator = "RECONNECTING..."
                conn_color = dark_theme['orange']
            else:
                conn_indicator = "DISCONNECTED"
                conn_color = dark_theme['red']

            # SpO2 quality indicator
            if visualizer.spo2_quality >= 80:
                quality_str = "Good"
            elif visualizer.spo2_quality >= 50:
                quality_str = "Fair"
            elif visualizer.spo2_quality > 0:
                quality_str = "Poor"
            else:
                quality_str = "No Signal"

            # SpO2 display with validity check
            if visualizer.spo2_percent >= 70 and visualizer.spo2_quality > 0:
                spo2_str = f"{visualizer.spo2_percent}%"
            else:
                spo2_str = "---"

            # Mode indicators
            hr_mode_str = "ON" if visualizer.hr_mode else "OFF"
            spo2_mode_str = "ON" if visualizer.spo2_mode else "OFF"

            # Temperature display
            temp_str = f"{visualizer.skin_temp_c:.1f}C" if visualizer.skin_temp_c > 0 else "---"

            # Buffer fill (0-5 seconds)
            buffer_secs = len(visualizer.hr_detector.ppg_buffer) / 25.0

            status = f"""[{conn_indicator}]

Battery: {visualizer.battery_percent}%
Status:  {charge_status_str}

Heart Rate: {visualizer.heart_rate} BPM
SpO2: {spo2_str} ({quality_str})
Skin Temp: {temp_str}

Modes: HR={hr_mode_str} SpO2={spo2_mode_str}

Orientation:
  Roll:  {roll:6.1f}
  Pitch: {pitch:6.1f}
  Yaw:   {yaw:6.1f}

Buffer: {buffer_secs:.1f}s
Packets: {visualizer.packet_count}
"""
            status_text.set_text(status)
            status_text.set_color(conn_color if conn_indicator != "CONNECTED" else dark_theme['text'])

        return [ppg_raw_line, ppg_filt_line, status_text]

    # Animation
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)

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
    print("\033[36m" + "=" * 50 + "\033[0m")
    print("\033[36m  nRF52840 BLE Sensor Visualizer\033[0m")
    print("\033[90m  Dark Mode | Auto-reconnection | Artifact Rejection\033[0m")
    print("\033[36m" + "=" * 50 + "\033[0m")
    print()

    # Run the async visualizer
    try:
        asyncio.run(run_visualizer())
    except KeyboardInterrupt:
        print("\n\033[33mExiting...\033[0m")

if __name__ == "__main__":
    main()
