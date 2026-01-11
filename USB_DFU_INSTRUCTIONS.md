# USB DFU Flashing Instructions for nRF52840

Your project is now configured with MCUboot USB DFU support for easy USB-C flashing!

## Initial Setup (One-Time)

1. **First flash with programmer** (J-Link, etc.):
   ```bash
   west build -p
   west flash
   ```
   This flashes both the MCUboot bootloader and your application.

## Regular Development Workflow

### Method 1: Using nRF Connect Programmer (Recommended)

1. Connect your nRF52840 via USB-C
2. Press the RESET button while holding BOOT/DFU button to enter DFU mode
3. Open nRF Connect for Desktop → Programmer
4. Select your device
5. Load the `build/zephyr/app_update.bin` file
6. Click "Write"

### Method 2: Using nrfutil (Command Line)

1. Install nrfutil:
   ```bash
   pip install nrfutil
   ```

2. Enter DFU mode on your device (reset while holding DFU button)

3. Flash via USB:
   ```bash
   nrfutil dfu usb-serial -pkg build/zephyr/dfu_application.zip -p <PORT>
   ```

### Method 3: Using dfu-util (Cross-platform CLI)

1. Install dfu-util:
   - Windows: `choco install dfu-util`
   - macOS: `brew install dfu-util`
   - Linux: `sudo apt-get install dfu-util`

2. Enter DFU mode

3. Flash:
   ```bash
   dfu-util -a 0 -D build/zephyr/app_update.bin
   ```

## Entering DFU Mode

### Hardware Method:
- Connect USB-C cable
- Hold the DFU/BOOT button (if available on your board)
- Press and release RESET button
- Release DFU button
- Device should appear as DFU device

### Software Method (if implemented):
- Send a special BLE command to trigger DFU mode
- Or add a button press pattern in your application

## Build Outputs

After building with MCUboot enabled, you'll get:

- `build/zephyr/zephyr.hex` - Full image (bootloader + app) for initial programming
- `build/zephyr/app_update.bin` - Application only for DFU updates
- `build/mcuboot/zephyr/zephyr.hex` - Bootloader only

## Troubleshooting

**Device not appearing in DFU mode:**
- Make sure USB drivers are installed (Windows may need Zadig)
- Check that MCUboot was flashed correctly
- Verify USB-C cable supports data (not charge-only)
- Try different USB ports

**DFU flash fails:**
- Ensure device is in DFU mode
- Check that the .bin file is the correct app_update.bin
- Verify flash partitions have enough space

**Application doesn't run after DFU:**
- Make sure you're flashing app_update.bin, not the full zephyr.hex
- Check that partition sizes in pm_static.yml are correct
- Verify signing keys match between bootloader and application

## Partition Layout

```
0x00000 - 0x10000: MCUboot bootloader (64KB)
0x10000 - 0x10200: Padding (512B)
0x10200 - 0x80200: Slot 0 - Active application (448KB)
0x80200 - 0xF0200: Slot 1 - Update application (448KB)
0xF0000 - 0xFFFFF: Settings storage (64KB)
```

## Security Note

The current configuration uses a default signing key for development.
**For production**, you must:
1. Generate your own RSA key pair
2. Update `CONFIG_BOOT_SIGNATURE_KEY_FILE` in child_image/mcuboot.conf
3. Never share your private key
