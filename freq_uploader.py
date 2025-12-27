#!/usr/bin/env python3
"""
Ham Radio Cloud Frequency Uploader v3 (WebSocket)

Reads frequency from IC-7300 and publishes to WebSocket server in real-time.
No Firebase dependency - direct WebSocket for <100ms latency.

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
import threading
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler

import serial
from serial.serialutil import SerialException
from serial.tools import list_ports

try:
    import websocket
except ImportError:
    print("[ERROR] websocket-client not installed. Run: pip3 install websocket-client")
    sys.exit(1)

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

# WebSocket Server
WEBSOCKET_URL = "wss://ws.ve3axc.com"

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
    """Auto-detect IC-7300 serial port by USB VID/PID."""
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            if p.serial_number and p.serial_number.startswith("IC-7300"):
                dev = p.device.replace("/dev/tty.", "/dev/cu.")
                return dev

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
    """Best-effort buffer reset. Returns False if device dropped."""
    try:
        ser.reset_input_buffer()
        return True
    except (termios.error, OSError):
        return False


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
            })
    return out


def port_still_listed(port_path: str):
    """Check if port is still visible in system."""
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
    """Read from serial until FD terminator or timeout."""
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
    """Find and parse a CI-V frame in the buffer."""
    i = 0
    while True:
        i = buf.find(b"\xFE\xFE", i)
        if i < 0:
            return None
        j = buf.find(b"\xFD", i + 2)
        if j < 0:
            return None
        frame = buf[i:j + 1]
        if len(frame) >= 6:
            to_, frm, cmd = frame[2], frame[3], frame[4]
            payload = frame[5:-1]
            if expect_cmd is None or cmd == expect_cmd:
                return to_, frm, cmd, payload
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
# WebSocket Publisher
# ============================================================================

class FrequencyPublisher:
    """WebSocket client for publishing frequency updates."""

    def __init__(self, station_id):
        self.station_id = station_id
        self.ws = None
        self.connected = False
        self.reconnect_delay = 1.0
        self.max_reconnect_delay = 30.0
        self.lock = threading.Lock()
        self.should_run = True
        self.ws_thread = None

    def start(self):
        """Start the WebSocket connection in a background thread."""
        self.should_run = True
        self.ws_thread = threading.Thread(target=self._run_forever, daemon=True)
        self.ws_thread.start()

    def stop(self):
        """Stop the WebSocket connection."""
        self.should_run = False
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass

    def _run_forever(self):
        """Run WebSocket with auto-reconnection."""
        while self.should_run:
            try:
                self._connect()
            except Exception as e:
                logger.error("WebSocket error: %s", e)

            if self.should_run:
                print(f"[WS] Reconnecting in {self.reconnect_delay:.1f}s...")
                time.sleep(self.reconnect_delay)
                self.reconnect_delay = min(self.reconnect_delay * 2, self.max_reconnect_delay)

    def _connect(self):
        """Establish WebSocket connection."""
        self.ws = websocket.WebSocketApp(
            WEBSOCKET_URL,
            on_open=self._on_open,
            on_close=self._on_close,
            on_error=self._on_error,
            on_message=self._on_message
        )
        self.ws.run_forever(ping_interval=30, ping_timeout=10)

    def _on_open(self, ws):
        """Handle connection opened."""
        with self.lock:
            self.connected = True
            self.reconnect_delay = 1.0
        print("[WS] Connected to server")
        logger.info("WebSocket connected")

    def _on_close(self, ws, close_status_code, close_msg):
        """Handle connection closed."""
        with self.lock:
            self.connected = False
        print(f"[WS] Disconnected (code={close_status_code})")
        logger.info("WebSocket disconnected: %s", close_msg)

    def _on_error(self, ws, error):
        """Handle connection error."""
        with self.lock:
            self.connected = False
        print(f"[WS] Error: {error}")
        logger.error("WebSocket error: %s", error)

    def _on_message(self, ws, message):
        """Handle incoming messages (not expected, but log them)."""
        try:
            data = json.loads(message)
            if data.get("type") == "error":
                print(f"[WS] Server error: {data.get('message')}")
        except Exception:
            pass

    def publish(self, freq_hz, mode):
        """Publish frequency update to server."""
        with self.lock:
            if not self.connected or not self.ws:
                return False

        try:
            message = json.dumps({
                "type": "publish",
                "station_id": self.station_id,
                "data": {
                    "frequency_hz": freq_hz,
                    "mode": mode or "UNKNOWN"
                }
            })
            self.ws.send(message)
            return True
        except Exception as e:
            logger.error("Publish error: %s", e)
            return False

    def is_connected(self):
        """Check if currently connected."""
        with self.lock:
            return self.connected


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
                        help="Your callsign (optional)")
    parser.add_argument("--port", default=None,
                        help="Serial port (default: auto-detect)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Poll interval in seconds (default: 1.0)")
    parser.add_argument("--upload-interval", type=float, default=5.0,
                        help="Min seconds between uploads if no change (default: 5.0)")
    args = parser.parse_args()

    if not args.station_id or len(args.station_id) < 4:
        print("[ERROR] Invalid station ID. Get yours from the web app.")
        sys.exit(1)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Start caffeinate (macOS)
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
            print("[INIT] Sleep prevention: FAILED")

    print(f"[INIT] Station ID: {args.station_id}")
    print(f"[INIT] WebSocket: {WEBSOCKET_URL}")
    print(f"[INIT] Logging to: {LOG_PATH}")
    if args.callsign:
        print(f"[INIT] Callsign: {args.callsign}")
    logger.info("START pid=%s station_id=%s callsign=%s",
                os.getpid(), args.station_id, args.callsign)
    print("-" * 60)

    # Start WebSocket publisher
    publisher = FrequencyPublisher(args.station_id)
    publisher.start()
    print("[INIT] WebSocket publisher started")

    # Wait briefly for initial connection
    time.sleep(1.0)

    ser = None
    backoff = 1.0
    last_freq = 0
    last_mode = ""
    last_publish_time = 0
    publish_count = 0
    reconnect_count = 0
    last_good_time = time.monotonic()

    while not stop_requested:
        try:
            # Connect/reconnect if needed
            if ser is None or not ser.is_open:
                if args.port:
                    print(f"[CONN] Connecting to {args.port}...")
                else:
                    print("[CONN] Scanning for IC-7300...")

                ser = open_serial(args.port)

                if not safe_reset_input_buffer(ser):
                    raise OSError("Device dropped immediately after connect")

                backoff = 1.0
                last_good_time = time.monotonic()
                reconnect_count += 1
                conn_msg = f"[CONN] Connected to {ser.port}"
                if reconnect_count > 1:
                    conn_msg += f" (reconnect #{reconnect_count})"
                print(conn_msg)
                logger.info("CONNECTED port=%s", ser.port)
                print("-" * 60)

            # Poll frequency and mode
            freq = read_frequency(ser)
            time.sleep(0.01)
            mode = read_mode(ser)

            got_any = freq is not None or mode is not None

            if got_any:
                last_good_time = time.monotonic()

            if freq is not None:
                freq_mhz = freq / 1e6
                timestamp = datetime.now().strftime("%H:%M:%S")
                now = time.time()

                freq_changed = (freq != last_freq or mode != last_mode)
                # Publish on change, or every 5 seconds as heartbeat
                time_to_heartbeat = (now - last_publish_time) >= 5.0

                if freq_changed or time_to_heartbeat:
                    if publisher.publish(freq, mode):
                        publish_count += 1
                        ws_status = "WS" if publisher.is_connected() else "queued"
                        change_indicator = "->" if freq_changed else "  "
                        print(f"[{timestamp}] {freq_mhz:12.6f} MHz  {mode or '':5} {change_indicator} [{ws_status}]")
                        last_publish_time = now
                    else:
                        print(f"[{timestamp}] {freq_mhz:12.6f} MHz  {mode or '':5}    [offline]")

                    last_freq = freq
                    last_mode = mode

            elif not got_any:
                no_resp_seconds = time.monotonic() - last_good_time
                if no_resp_seconds >= NO_RESPONSE_TIMEOUT:
                    logger.warning("NO_RESPONSE timeout=%.1fs", no_resp_seconds)
                    raise PlannedReconnect(f"No CI-V response for {no_resp_seconds:.1f}s")

            if stop_requested:
                break

            time.sleep(args.interval)

        except PlannedReconnect as e:
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
            error_time = datetime.now().strftime("%H:%M:%S")
            print(f"[{error_time}] ERROR: {type(e).__name__}: {e}")
            logger.error("EXCEPTION type=%s msg=%r", type(e).__name__, str(e))

            if ser:
                safe_close(ser)
                ser = None

            if not stop_requested:
                print(f"[WAIT] Reconnecting in {backoff:.1f}s...")
                wait_end = time.time() + backoff
                while time.time() < wait_end and not stop_requested:
                    time.sleep(0.1)
                backoff = min(backoff * 2, 10.0)

    # Clean shutdown
    print("-" * 60)
    print("[EXIT] Stopping WebSocket publisher...")
    publisher.stop()

    if ser:
        print("[EXIT] Closing serial port...")
        safe_close(ser)

    if caffeinate_proc:
        try:
            caffeinate_proc.terminate()
        except Exception:
            pass

    print(f"[EXIT] Done. Published: {publish_count}, Reconnects: {reconnect_count}")
    logger.info("EXIT publishes=%s reconnects=%s", publish_count, reconnect_count)


if __name__ == "__main__":
    main()
