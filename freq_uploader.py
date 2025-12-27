#!/usr/bin/env python3
"""
Ham Radio Cloud Frequency Uploader

Reads frequency from IC-7300 and uploads to Firebase in real-time.
Designed for one-command setup and reliable 24/7 operation.

Usage:
    python3 freq_uploader.py --station-id YOUR_STATION_ID
    python3 freq_uploader.py --station-id f7k9x2m4 --callsign VE3AXC
"""

import time
import argparse
import signal
import sys
import subprocess
import os
import json
import logging
import termios
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler

import requests
import serial
from serial.serialutil import SerialException
from serial.tools import list_ports

# ============================================================================
# Logging Setup
# ============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(SCRIPT_DIR, "freq_uploader.log")

logger = logging.getLogger("freq_uploader")
logger.setLevel(logging.INFO)

log_handler = RotatingFileHandler(LOG_PATH, maxBytes=1_000_000, backupCount=5)
log_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
logger.addHandler(log_handler)

# ============================================================================
# Configuration
# ============================================================================

# Firebase Realtime Database (Europe West region)
FIREBASE_URL = "https://ve3axc-online-logger-default-rtdb.europe-west1.firebasedatabase.app"

# IC-7300 CP2102 USB identifiers
VID = 0x10C4  # Silicon Labs
PID = 0xEA60  # CP210x
BAUD = 19200

# CI-V Protocol
CIV_ADDR_RADIO = 0x94  # IC-7300 default address
CIV_ADDR_CONTROLLER = 0xE0

MODE_MAP = {
    0x00: "LSB", 0x01: "USB", 0x02: "AM", 0x03: "CW",
    0x04: "RTTY", 0x05: "FM", 0x07: "CW-R", 0x08: "RTTY-R"
}

# No-response timeout (force reconnect after this many seconds)
NO_RESPONSE_TIMEOUT = 10.0

# Pause check interval (how often to poll Firebase for pause flag)
PAUSE_CHECK_INTERVAL = 2.0

# Global stop flag
stop_requested = False


class PlannedReconnect(Exception):
    """Raised for expected reconnects (e.g., no response timeout) vs real USB errors."""
    pass


# ============================================================================
# Signal Handling
# ============================================================================

def on_signal(signum, frame):
    """Handle Ctrl+C and SIGTERM gracefully."""
    global stop_requested
    stop_requested = True
    print("\n[STOP] Shutdown requested...")


# ============================================================================
# Serial Port Functions
# ============================================================================

def find_ic7300_port():
    """
    Auto-detect IC-7300 serial port by USB VID/PID.
    Returns /dev/cu.* path (preferred on macOS).
    """
    # First pass: look for IC-7300 specifically
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            if p.serial_number and p.serial_number.startswith("IC-7300"):
                dev = p.device.replace("/dev/tty.", "/dev/cu.")
                return dev

    # Fallback: any CP210x device
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            dev = p.device.replace("/dev/tty.", "/dev/cu.")
            return dev

    return None


def open_serial(port_override=None):
    """Open serial connection to IC-7300."""
    if port_override:
        dev = port_override.replace("/dev/tty.", "/dev/cu.")
    else:
        dev = find_ic7300_port()
        if not dev:
            raise FileNotFoundError("IC-7300 not found. Is it connected and powered on?")

    ser = serial.Serial(
        dev,
        baudrate=BAUD,
        timeout=0.2,
        write_timeout=0.5,
        rtscts=False,
        dsrdtr=False
    )
    return ser


def drain_rx(ser, seconds=0.25):
    """Drain pending RX data before closing."""
    end = time.time() + seconds
    while time.time() < end:
        try:
            n = ser.in_waiting
            if n:
                ser.read(n)
                end = time.time() + seconds
            else:
                time.sleep(0.01)
        except Exception:
            break


def safe_close(ser):
    """Close serial port cleanly."""
    if ser is None:
        return
    try:
        try:
            ser.flush()
        except Exception:
            pass
        try:
            drain_rx(ser, 0.25)
        except Exception:
            pass
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
    finally:
        try:
            ser.close()
        except Exception:
            pass


def safe_reset_input_buffer(ser):
    """
    Best-effort buffer reset. Returns False if device dropped.
    Uses this instead of raw reset_input_buffer() to avoid crashes.
    """
    try:
        ser.reset_input_buffer()
        return True
    except (termios.error, OSError):
        return False  # Caller should trigger reconnect


def snapshot_ports():
    """Snapshot of all matching USB ports for debugging."""
    out = []
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            out.append({
                "device": p.device,
                "vid": hex(p.vid) if p.vid else None,
                "pid": hex(p.pid) if p.pid else None,
                "serial_number": p.serial_number,
                "manufacturer": p.manufacturer,
                "product": p.product,
                "location": getattr(p, "location", None),
            })
    return out


def port_still_listed(port_path: str):
    """Check if port is still visible in system (vs re-enumerated/dropped)."""
    if not port_path:
        return False
    for p in list_ports.comports():
        if p.device == port_path:
            return True
    return False


# ============================================================================
# CI-V Protocol Functions
# ============================================================================

def read_until_fd(ser, deadline_s=0.2, max_bytes=256):
    """
    Read from serial until FD terminator or timeout.
    More reliable than fixed sleep + read.
    """
    end = time.monotonic() + deadline_s
    buf = bytearray()
    while time.monotonic() < end and len(buf) < max_bytes:
        chunk = ser.read(32)
        if chunk:
            buf += chunk
            if 0xFD in chunk:
                break
    return bytes(buf)


def find_civ_frame(buf: bytes, expect_cmd: int = None):
    """
    Find and parse a CI-V frame in the buffer.
    CI-V frames are not guaranteed to start at byte 0 - there can be
    partial frames, multiple frames, echo of command, or wrong response.

    If expect_cmd is specified, scans through buffer until finding a frame
    with that command byte (skips echoes and other frames).

    Returns (to, frm, cmd, payload_bytes) or None if no valid frame found.
    """
    i = 0
    while True:
        i = buf.find(b"\xFE\xFE", i)
        if i < 0:
            return None
        # Find terminator
        j = buf.find(b"\xFD", i + 2)
        if j < 0:
            return None
        frame = buf[i:j + 1]
        if len(frame) >= 6:
            to_, frm, cmd = frame[2], frame[3], frame[4]
            payload = frame[5:-1]
            # If we're looking for a specific command, check it
            if expect_cmd is None or cmd == expect_cmd:
                return to_, frm, cmd, payload
        # Not the frame we want, keep scanning
        i = j + 1

def decode_freq(bcd_bytes):
    """Decode BCD frequency bytes to Hz."""
    freq = 0
    for i, byte in enumerate(bcd_bytes):
        freq += (byte & 0x0F) * (10 ** (i * 2))
        freq += ((byte >> 4) & 0x0F) * (10 ** (i * 2 + 1))
    return freq


def read_frequency(ser):
    """Read frequency from radio via CI-V."""
    # CI-V: FE FE [to] [from] [cmd] FD
    # 0x94 = IC-7300, 0xE0 = controller, 0x03 = read freq
    ser.write(bytes([0xFE, 0xFE, CIV_ADDR_RADIO, CIV_ADDR_CONTROLLER, 0x03, 0xFD]))
    resp = read_until_fd(ser, deadline_s=0.2)

    parsed = find_civ_frame(resp, expect_cmd=0x03)
    if not parsed:
        return None
    to_, frm, cmd, payload = parsed
    if len(payload) >= 5:
        return decode_freq(payload[:5])
    return None


def read_mode(ser):
    """Read mode from radio via CI-V."""
    # 0x04 = read mode
    ser.write(bytes([0xFE, 0xFE, CIV_ADDR_RADIO, CIV_ADDR_CONTROLLER, 0x04, 0xFD]))
    resp = read_until_fd(ser, deadline_s=0.2)

    parsed = find_civ_frame(resp, expect_cmd=0x04)
    if not parsed:
        return None
    to_, frm, cmd, payload = parsed
    if len(payload) >= 1:
        return MODE_MAP.get(payload[0], f"?{payload[0]}")
    return None


# ============================================================================
# Firebase Functions
# ============================================================================

def upload_to_firebase(station_id, data):
    """
    Upload station data to Firebase Realtime Database.
    Uses REST API with PUT to update the station's data.
    """
    url = f"{FIREBASE_URL}/stations/{station_id}.json"
    try:
        response = requests.put(url, json=data, timeout=5)
        response.raise_for_status()
        return True
    except requests.exceptions.RequestException as e:
        print(f"[UPLOAD] Error: {e}")
        return False


def set_offline(station_id):
    """Mark station as offline in Firebase."""
    url = f"{FIREBASE_URL}/stations/{station_id}/online.json"
    try:
        requests.put(url, json=False, timeout=5)
    except Exception:
        pass  # Best effort


def check_paused(station_id):
    """
    Check if station is paused (user clicked Disconnect in web app).
    Returns True if paused, False otherwise.
    """
    url = f"{FIREBASE_URL}/stations/{station_id}/paused.json"
    try:
        response = requests.get(url, timeout=3)
        if response.status_code == 200:
            result = response.json()
            return result is True
    except Exception:
        pass  # Network error - assume not paused
    return False


def set_paused_status(station_id, paused):
    """Set the paused status in Firebase (used on startup to clear any stale pause)."""
    url = f"{FIREBASE_URL}/stations/{station_id}/paused.json"
    try:
        requests.put(url, json=paused, timeout=3)
    except Exception:
        pass  # Best effort


# ============================================================================
# Main
# ============================================================================

def main():
    global stop_requested

    parser = argparse.ArgumentParser(
        description="Ham Radio Cloud Frequency Uploader",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 freq_uploader.py --station-id f7k9x2m4
    python3 freq_uploader.py --station-id f7k9x2m4 --callsign VE3AXC
    python3 freq_uploader.py --station-id f7k9x2m4 --port /dev/cu.usbserial-2110
        """
    )
    parser.add_argument("--station-id", required=True,
                        help="Your unique station ID from the web app")
    parser.add_argument("--callsign", default=None,
                        help="Your callsign (optional, shown in data)")
    parser.add_argument("--port", default=None,
                        help="Serial port (default: auto-detect)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Poll interval in seconds (default: 1.0)")
    parser.add_argument("--upload-interval", type=float, default=5.0,
                        help="Min seconds between uploads if no change (default: 5.0)")
    args = parser.parse_args()

    # Validate station ID
    if not args.station_id or len(args.station_id) < 4:
        print("[ERROR] Invalid station ID. Get yours from the web app.")
        sys.exit(1)

    # Set up signal handlers
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Start caffeinate to prevent system sleep (macOS)
    caffeinate_proc = None
    if sys.platform == "darwin":
        try:
            caffeinate_proc = subprocess.Popen(
                ['caffeinate', '-i', '-w', str(os.getpid())],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("[INIT] Sleep prevention: ACTIVE")
        except Exception:
            print("[INIT] Sleep prevention: FAILED (caffeinate not found)")

    print(f"[INIT] Station ID: {args.station_id}")
    print(f"[INIT] Logging to: {LOG_PATH}")
    if args.callsign:
        print(f"[INIT] Callsign: {args.callsign}")
    logger.info("START pid=%s station_id=%s callsign=%s argv=%s",
                os.getpid(), args.station_id, args.callsign, sys.argv)
    print("-" * 60)

    ser = None
    backoff = 1.0
    last_freq = 0
    last_mode = ""
    last_upload_time = 0
    upload_count = 0
    reconnect_count = 0
    last_good_time = time.monotonic()  # Track time since last valid response

    # Pause/resume state (controlled by web app)
    is_paused = False
    last_pause_check = 0

    # Clear any stale pause flag on startup
    set_paused_status(args.station_id, False)

    while not stop_requested:
        try:
            # Check if paused by web app (every PAUSE_CHECK_INTERVAL seconds)
            now = time.time()
            if now - last_pause_check >= PAUSE_CHECK_INTERVAL:
                last_pause_check = now
                was_paused = is_paused
                is_paused = check_paused(args.station_id)

                if is_paused and not was_paused:
                    print("[PAUSE] Paused by web app. Waiting for resume...")
                    set_offline(args.station_id)
                    if ser:
                        safe_close(ser)
                        ser = None
                elif not is_paused and was_paused:
                    print("[RESUME] Resumed by web app. Reconnecting...")

            # If paused, just sleep and continue
            if is_paused:
                time.sleep(1.0)
                continue

            # Connect/reconnect if needed
            if ser is None or not ser.is_open:
                if args.port:
                    print(f"[CONN] Connecting to {args.port}...")
                else:
                    print("[CONN] Scanning for IC-7300...")

                ser = open_serial(args.port)

                # Safe buffer flush on connect (may fail if device drops immediately)
                if not safe_reset_input_buffer(ser):
                    raise OSError("Device dropped immediately after connect")

                backoff = 1.0  # Reset backoff on success
                last_good_time = time.monotonic()  # Reset timeout on new connection
                reconnect_count += 1
                conn_msg = f"[CONN] Connected to {ser.port}"
                if reconnect_count > 1:
                    conn_msg += f" (reconnect #{reconnect_count})"
                print(conn_msg)
                logger.info("CONNECTED port=%s snapshot=%s", ser.port,
                            json.dumps(snapshot_ports(), ensure_ascii=False))
                print("-" * 60)

            # Poll frequency and mode (with small gap to avoid response overlap)
            freq = read_frequency(ser)
            time.sleep(0.01)  # Small gap between commands
            mode = read_mode(ser)

            # Count as success if we got EITHER freq or mode
            got_any = freq is not None or mode is not None

            if got_any:
                last_good_time = time.monotonic()  # Reset timeout on valid response

            if freq is not None:
                freq_mhz = freq / 1e6
                timestamp = datetime.now().strftime("%H:%M:%S")
                now = time.time()

                # Decide if we should upload
                freq_changed = (freq != last_freq or mode != last_mode)
                time_to_heartbeat = (now - last_upload_time) >= args.upload_interval

                if freq_changed or time_to_heartbeat:
                    # Build data payload
                    data = {
                        "frequency_hz": freq,
                        "frequency_mhz": round(freq_mhz, 6),
                        "mode": mode or "UNKNOWN",
                        "radio_model": "IC-7300",
                        "last_updated": datetime.now(timezone.utc).isoformat(),
                        "online": True
                    }
                    if args.callsign:
                        data["callsign"] = args.callsign.upper()

                    # Upload to Firebase
                    if upload_to_firebase(args.station_id, data):
                        upload_count += 1
                        status = "-> uploaded" if freq_changed else "[heartbeat]"
                        print(f"[{timestamp}] {freq_mhz:12.6f} MHz  {mode or '':5} {status}")
                        last_upload_time = now
                    else:
                        print(f"[{timestamp}] {freq_mhz:12.6f} MHz  {mode or '':5} [upload failed]")

                    last_freq = freq
                    last_mode = mode

            elif not got_any:
                # No response from radio
                no_resp_seconds = time.monotonic() - last_good_time

                # If port is open but radio not responding for too long, force reconnect
                if no_resp_seconds >= NO_RESPONSE_TIMEOUT:
                    logger.warning("NO_RESPONSE timeout=%.1fs port=%s is_open=%s snapshot=%s",
                                   no_resp_seconds, getattr(ser, "port", None),
                                   getattr(ser, "is_open", None),
                                   json.dumps(snapshot_ports(), ensure_ascii=False))
                    raise PlannedReconnect(f"No CI-V response for {no_resp_seconds:.1f}s")

            if stop_requested:
                break

            time.sleep(args.interval)

        except PlannedReconnect as e:
            # Expected reconnect (no response timeout) - log as WARNING, not ERROR
            error_time = datetime.now().strftime("%H:%M:%S")
            print(f"[{error_time}] RECONNECT: {e}")

            if ser:
                safe_close(ser)
                ser = None

            if not stop_requested:
                print(f"[WAIT] Reconnecting in {backoff:.1f}s...")
                logger.info("RECONNECT reason=%r backoff=%.1fs", str(e), backoff)

                wait_end = time.time() + backoff
                while time.time() < wait_end and not stop_requested:
                    time.sleep(0.1)

                backoff = min(backoff * 2, 10.0)

        except (OSError, SerialException, FileNotFoundError, termios.error) as e:
            # USB disconnect, port gone, device not found, or termios failure
            error_time = datetime.now().strftime("%H:%M:%S")
            print(f"[{error_time}] ERROR: {type(e).__name__}: {e}")

            # Log detailed exception info
            errno = getattr(e, "errno", None)
            port_path = getattr(ser, "port", None)
            logger.error("EXCEPTION type=%s errno=%s msg=%r port=%s is_open=%s port_still_listed=%s snapshot=%s",
                         type(e).__name__, errno, str(e),
                         port_path, getattr(ser, "is_open", None),
                         port_still_listed(port_path),
                         json.dumps(snapshot_ports(), ensure_ascii=False))

            if ser:
                safe_close(ser)
                ser = None

            if not stop_requested:
                print(f"[WAIT] Reconnecting in {backoff:.1f}s...")
                logger.info("RECONNECT backoff=%.1fs", backoff)

                # Sleep in small chunks so we can respond to Ctrl+C
                wait_end = time.time() + backoff
                while time.time() < wait_end and not stop_requested:
                    time.sleep(0.1)

                backoff = min(backoff * 2, 10.0)  # Exponential backoff, max 10s

    # Clean shutdown
    print("-" * 60)
    print("[EXIT] Setting station offline...")
    set_offline(args.station_id)

    if ser:
        print("[EXIT] Closing serial port...")
        safe_close(ser)

    # Terminate caffeinate (will auto-exit via -w, but explicit is cleaner)
    if caffeinate_proc:
        try:
            caffeinate_proc.terminate()
        except Exception:
            pass

    print(f"[EXIT] Done. Uploads: {upload_count}, Reconnects: {reconnect_count}")
    logger.info("EXIT uploads=%s reconnects=%s", upload_count, reconnect_count)


if __name__ == "__main__":
    main()
