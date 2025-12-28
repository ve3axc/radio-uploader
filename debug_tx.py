#!/usr/bin/env python3
"""
IC-7300 TX Detection Debug Script

This script monitors raw CI-V responses to help debug TX detection.
Press your mic PTT or CW key while watching the output to see what changes.

Usage:
  python3 debug_tx.py [serial_port]

  Default port: /dev/tty.usbserial-14130
"""

import serial
import time
import sys

# CI-V addresses
CIV_ADDR_RADIO = 0x94       # IC-7300 default
CIV_ADDR_CONTROLLER = 0xE0  # Controller (us)

def find_serial_port():
    """Find the IC-7300 serial port."""
    import glob
    patterns = [
        '/dev/tty.usbserial-*',
        '/dev/ttyUSB*',
        '/dev/tty.SLAB_USBtoUART*',
    ]
    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]
    return None

def read_until_fd(ser, deadline_s=0.3):
    """Read bytes until 0xFD or timeout."""
    buf = bytearray()
    start = time.time()
    while time.time() - start < deadline_s:
        if ser.in_waiting:
            b = ser.read(1)
            buf.extend(b)
            if b == b'\xfd':
                break
        else:
            time.sleep(0.01)
    return bytes(buf)

def send_civ_command(ser, cmd, subcmd=None):
    """Send a CI-V command and return raw response."""
    frame = bytearray([0xFE, 0xFE, CIV_ADDR_RADIO, CIV_ADDR_CONTROLLER, cmd])
    if subcmd is not None:
        frame.append(subcmd)
    frame.append(0xFD)

    # Clear input buffer
    ser.reset_input_buffer()

    # Send command
    ser.write(frame)

    # Read response
    resp = read_until_fd(ser, deadline_s=0.2)
    return resp

def hex_dump(data, label=""):
    """Format bytes as hex string."""
    if not data:
        return f"{label}: (no response)"
    hex_str = ' '.join(f'{b:02X}' for b in data)
    return f"{label}: {hex_str}"

def main():
    # Get serial port
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_serial_port()
        if not port:
            print("ERROR: No serial port found!")
            print("Usage: python3 debug_tx.py /dev/tty.usbserial-XXXXX")
            sys.exit(1)

    print("=" * 60)
    print("  IC-7300 TX Detection Debug")
    print("=" * 60)
    print(f"Serial port: {port}")
    print()
    print("Commands being tested:")
    print("  0x15 0x11 - Power Output meter (current approach)")
    print("  0x15 0x12 - SWR meter")
    print("  0x1C 0x00 - TX/RX status query")
    print("  0x15 0x02 - S-meter")
    print()
    print("Press Ctrl+C to stop")
    print("Press your PTT/CW key and watch for changes!")
    print("-" * 60)
    print()

    try:
        ser = serial.Serial(port, 19200, timeout=0.1)
        time.sleep(0.5)  # Let port settle

        iteration = 0
        while True:
            iteration += 1
            print(f"--- Poll #{iteration} ---")

            # Test each command
            commands = [
                (0x15, 0x11, "Power Output (0x15 0x11)"),
                (0x15, 0x12, "SWR Meter   (0x15 0x12)"),
                (0x1C, 0x00, "TX Status   (0x1C 0x00)"),
                (0x15, 0x02, "S-Meter     (0x15 0x02)"),
            ]

            for cmd, subcmd, label in commands:
                resp = send_civ_command(ser, cmd, subcmd)
                print(hex_dump(resp, f"  {label}"))

            print()
            time.sleep(0.5)  # Poll every 500ms

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)

if __name__ == "__main__":
    main()
