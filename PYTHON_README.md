# BLE UART Python Listener

This Python script connects to your nRF device via Bluetooth LE and allows you to receive and send data over the Nordic UART Service (NUS).

## Setup

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Usage

1. Flash the modified firmware to your nRF device
2. Run the Python listener:
```bash
python ble_listener.py
```

3. The script will:
   - Scan for BLE devices named "Nordic_UART_Service"
   - Connect to your nRF device
   - Display incoming "Hello" messages every 2 seconds
   - Allow you to send test messages back to the device

## What Changed in the nRF Code

The modified [main.c](src/main.c) now:
- Sends "Hello #N" messages every 2 seconds when connected
- Each message has an incrementing counter
- Still accepts incoming data from the Python client
- Logs sent messages for debugging

## Testing Communication

Once connected, you can:

1. **Receive data**: The nRF device automatically sends "Hello #N" every 2 seconds
2. **Send data**: Type a message in the Python terminal and press Enter
   - Example: `Test message from Python`
   - The nRF device will receive it and can echo it back

## Troubleshooting

**Device not found:**
- Make sure your nRF device is powered on
- Check that Bluetooth is enabled on your computer
- The device should be advertising (check LED status)
- Try adjusting the device name in the Python script if your device uses a different name

**Connection fails:**
- On Windows, you may need to pair the device first via Bluetooth settings
- Try running the script with administrator privileges
- Make sure no other application is connected to the device

**Permission errors (Linux):**
```bash
sudo setcap 'cap_net_raw,cap_net_admin+eip' $(which python3)
```

## Device Name Configuration

If your device uses a different name, edit line 124 in `ble_listener.py`:
```python
device_name = "YourDeviceName"  # Change this
```
