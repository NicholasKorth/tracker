#!/usr/bin/env python3
"""
Bluetooth LE UART Service (NUS) Listener
Connects to nRF device and receives/sends data over BLE UART
"""

import asyncio
import sys
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# Nordic UART Service (NUS) UUIDs
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write to this
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Receive notifications from this


class BLEUARTClient:
    def __init__(self, device_name="Nordic_UART_Service"):
        self.device_name = device_name
        self.client = None
        self.device = None

    async def find_device(self, timeout=10.0):
        """Scan for BLE devices and find the target device"""
        print(f"Scanning for '{self.device_name}' device...")

        devices = await BleakScanner.discover(timeout=timeout)

        for device in devices:
            if device.name and self.device_name in device.name:
                print(f"\nFound device: {device.name}")
                print(f"  Address: {device.address}")
                self.device = device
                return device

        # If not found by name, list all available devices
        print("\nDevice not found. Available devices:")
        for device in devices:
            if device.name:
                print(f"  {device.name} ({device.address})")

        return None

    def notification_handler(self, sender, data):
        """Handle incoming data from BLE UART"""
        try:
            message = data.decode('utf-8').strip()
            print(f"[RX] {message}")
        except UnicodeDecodeError:
            print(f"[RX] (binary): {data.hex()}")

    async def connect(self):
        """Connect to the BLE device"""
        if not self.device:
            print("Error: No device found to connect to")
            return False

        try:
            print(f"\nConnecting to {self.device.address}...")
            self.client = BleakClient(self.device.address)
            await self.client.connect()
            print("Connected successfully!")

            # Subscribe to notifications
            await self.client.start_notify(UART_TX_CHAR_UUID, self.notification_handler)
            print("Subscribed to UART TX notifications")

            return True
        except BleakError as e:
            print(f"Connection failed: {e}")
            return False

    async def send_data(self, data):
        """Send data to the BLE device"""
        if not self.client or not self.client.is_connected:
            print("Error: Not connected to device")
            return False

        try:
            # Ensure data is bytes
            if isinstance(data, str):
                data = data.encode('utf-8')

            await self.client.write_gatt_char(UART_RX_CHAR_UUID, data)
            print(f"[TX] {data.decode('utf-8').strip()}")
            return True
        except Exception as e:
            print(f"Send failed: {e}")
            return False

    async def disconnect(self):
        """Disconnect from the BLE device"""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("\nDisconnected")

    async def interactive_session(self):
        """Run an interactive session to send/receive data"""
        print("\n=== Interactive Mode ===")
        print("Type messages to send to the device")
        print("Press Ctrl+C to exit\n")

        try:
            while self.client.is_connected:
                # Wait for user input (non-blocking approach)
                await asyncio.sleep(0.1)

        except KeyboardInterrupt:
            print("\nExiting...")


async def read_stdin_async():
    """Helper to read from stdin asynchronously"""
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, sys.stdin.readline)


async def main():
    # You can change this to match your device name
    # Common names: "Nordic_UART_Service", "nRF UART", etc.
    device_name = "nrf"

    client = BLEUARTClient(device_name)

    # Find the device
    device = await client.find_device(timeout=10.0)
    if not device:
        print(f"\nFailed to find device with name '{device_name}'")
        print("Make sure your nRF device is powered on and advertising")
        return

    # Connect to the device
    if not await client.connect():
        return

    try:
        print("\n=== Listening for data ===")
        print("The device will send 'Hello' messages every 2 seconds")
        print("\nTo send a test message, type below and press Enter:")
        print("(Press Ctrl+C to exit)\n")

        # Keep the connection alive and allow sending messages
        while client.client.is_connected:
            # Check if there's input available (Windows compatible)
            try:
                # Use a timeout to check for input periodically
                user_input = await asyncio.wait_for(
                    asyncio.get_event_loop().run_in_executor(None, input),
                    timeout=0.5
                )

                if user_input.strip():
                    # Add newline if not present
                    if not user_input.endswith('\n'):
                        user_input += '\r\n'
                    await client.send_data(user_input)

            except asyncio.TimeoutError:
                # No input, continue listening
                pass
            except EOFError:
                # Handle EOF gracefully
                break

    except KeyboardInterrupt:
        print("\n\nReceived exit signal...")
    finally:
        await client.disconnect()


if __name__ == "__main__":
    print("=" * 50)
    print("BLE UART Service Listener")
    print("=" * 50)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
