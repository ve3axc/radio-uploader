#!/bin/bash
# ============================================================================
# Ham Radio Cloud Frequency Uploader - One-Command Installer
# ============================================================================
#
# Usage:
#   curl -sL https://ve3axc.com/radio | bash -s YOUR_STATION_ID
#   curl -sL https://ve3axc.com/radio | bash -s YOUR_STATION_ID YOUR_CALLSIGN
#
# What this script does:
#   1. Checks/installs Python 3
#   2. Creates ~/.radio-uploader/ directory
#   3. Downloads freq_uploader.py
#   4. Installs dependencies (pyserial, websocket-client, certifi)
#   5. Starts uploading your frequency to the cloud
#
# ============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
INSTALL_DIR="$HOME/.radio-uploader"
SCRIPT_URL="https://raw.githubusercontent.com/ve3axc/radio-uploader/main/freq_uploader.py"

# Arguments
STATION_ID="${1:-}"
CALLSIGN="${2:-}"

# ============================================================================
# Helper Functions
# ============================================================================

print_header() {
    echo ""
    echo -e "${BLUE}============================================================${NC}"
    echo -e "${BLUE}  Ham Radio Cloud Frequency Uploader${NC}"
    echo -e "${BLUE}============================================================${NC}"
    echo ""
}

print_step() {
    echo -e "${YELLOW}[SETUP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ============================================================================
# Validation
# ============================================================================

print_header

if [ -z "$STATION_ID" ]; then
    print_error "Missing station ID!"
    echo ""
    echo "Usage:"
    echo "  curl -sL https://ve3axc.com/radio | bash -s YOUR_STATION_ID"
    echo "  curl -sL https://ve3axc.com/radio | bash -s YOUR_STATION_ID YOUR_CALLSIGN"
    echo ""
    echo "Get your station ID from the web app."
    exit 1
fi

echo "Station ID: $STATION_ID"
if [ -n "$CALLSIGN" ]; then
    echo "Callsign: $CALLSIGN"
fi
echo ""

# ============================================================================
# Check/Install Python
# ============================================================================

print_step "Checking for Python 3..."

if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1)
    print_success "Found $PYTHON_VERSION"
else
    print_step "Python 3 not found. Installing..."

    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS - try Homebrew first
        if command -v brew &> /dev/null; then
            brew install python3
        else
            print_error "Please install Python 3 from https://python.org"
            exit 1
        fi
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux
        if command -v apt-get &> /dev/null; then
            sudo apt-get update && sudo apt-get install -y python3 python3-pip python3-venv
        elif command -v yum &> /dev/null; then
            sudo yum install -y python3 python3-pip
        else
            print_error "Please install Python 3 manually"
            exit 1
        fi
    else
        print_error "Unsupported OS. Please install Python 3 manually."
        exit 1
    fi

    print_success "Python 3 installed"
fi

# ============================================================================
# Create Install Directory
# ============================================================================

print_step "Creating install directory..."

mkdir -p "$INSTALL_DIR"
print_success "Created $INSTALL_DIR"

# ============================================================================
# Create Virtual Environment
# ============================================================================

print_step "Setting up Python environment..."

if [ ! -d "$INSTALL_DIR/venv" ]; then
    python3 -m venv "$INSTALL_DIR/venv"
fi
print_success "Virtual environment ready"

# Activate venv
source "$INSTALL_DIR/venv/bin/activate"

# ============================================================================
# Download/Copy Script
# ============================================================================

print_step "Downloading uploader script..."

curl -sL "$SCRIPT_URL" -o "$INSTALL_DIR/freq_uploader.py"
print_success "Downloaded freq_uploader.py"

# ============================================================================
# Install Dependencies
# ============================================================================

print_step "Installing dependencies..."

pip install --quiet --upgrade pip
pip install --quiet pyserial websocket-client certifi

print_success "Dependencies installed"

# ============================================================================
# Create Start/Stop Scripts
# ============================================================================

print_step "Creating helper scripts..."

# Create start script
cat > "$INSTALL_DIR/start" << 'STARTSCRIPT'
#!/bin/bash
INSTALL_DIR="$HOME/.radio-uploader"
source "$INSTALL_DIR/venv/bin/activate"

STATION_ID="${1:-}"
CALLSIGN="${2:-}"

if [ -z "$STATION_ID" ]; then
    echo "Usage: $0 STATION_ID [CALLSIGN]"
    exit 1
fi

ARGS="--station-id $STATION_ID"
if [ -n "$CALLSIGN" ]; then
    ARGS="$ARGS --callsign $CALLSIGN"
fi

python3 "$INSTALL_DIR/freq_uploader.py" $ARGS
STARTSCRIPT

chmod +x "$INSTALL_DIR/start"

print_success "Created ~/.radio-uploader/start"

# ============================================================================
# Run the Uploader
# ============================================================================

echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  Setup Complete!${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo "Starting frequency uploader..."
echo ""
echo "To stop: Press Ctrl+C"
echo "To restart later: ~/.radio-uploader/start $STATION_ID $CALLSIGN"
echo ""
echo "------------------------------------------------------------"

# Build command
ARGS="--station-id $STATION_ID"
if [ -n "$CALLSIGN" ]; then
    ARGS="$ARGS --callsign $CALLSIGN"
fi

# Run the uploader
python3 "$INSTALL_DIR/freq_uploader.py" $ARGS
