# Ham Radio Cloud Frequency Uploader

Stream your radio's frequency to the cloud in real-time.

## Quick Start

```bash
curl -sL https://ve3axc.com/radio | bash -s YOUR_STATION_ID
```

Get your Station ID from the [QSO Logger](https://qso.ve3axc.com) app.

## What It Does

- Auto-detects your IC-7300 radio via USB
- Reads frequency and mode via CI-V protocol
- Uploads to Firebase in real-time
- Auto-reconnects on USB disconnect
- Runs until you press Ctrl+C

## Requirements

- macOS or Linux
- Python 3.8+
- IC-7300 connected via USB

## Manual Installation

```bash
# Clone the repo
git clone https://github.com/ve3axc/radio-uploader.git
cd radio-uploader

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run
python3 freq_uploader.py --station-id YOUR_STATION_ID --callsign YOUR_CALL
```

## Options

```
--station-id    Your unique station ID (required)
--callsign      Your callsign (optional)
--port          Serial port override (default: auto-detect)
--interval      Poll interval in seconds (default: 1.0)
```

## How It Works

```
IC-7300 (USB) → freq_uploader.py → Firebase → QSO Logger App
```

The uploader reads your radio's frequency via CI-V protocol and pushes it to Firebase. Any app subscribed to your station ID sees the updates in real-time.

## License

MIT
