#!/usr/bin/env python3
"""
Simple BLE Scanner - Lists all discoverable BLE devices
"""

import asyncio
from bleak import BleakScanner


async def scan_devices():
    print("Scanning for BLE devices for 10 seconds...")
    print("=" * 60)

    devices = await BleakScanner.discover(timeout=10.0)

    if not devices:
        print("No BLE devices found!")
        return

    print(f"\nFound {len(devices)} device(s):\n")

    for device in devices:
        print(f"Name: {device.name if device.name else '(Unknown)'}")
        print(f"Address: {device.address}")
        print(f"Details: {device.details}")
        print("-" * 60)


if __name__ == "__main__":
    print("BLE Device Scanner")
    print("=" * 60)
    asyncio.run(scan_devices())
