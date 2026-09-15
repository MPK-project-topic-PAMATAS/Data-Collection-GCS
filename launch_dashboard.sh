#!/bin/bash
# OUSTER LIDAR DASHBOARD LAUNCHER

cd "$(dirname "$0")"

if [ $# -eq 0 ]; then
    echo "OUSTER LIDAR DASHBOARD"
    echo ""
    echo "Usage: ./launch_dashboard.sh [OPTION] [VALUE]"
    echo ""
    echo "OPTIONS:"
    echo "  --live <IP>              Connect to live Ouster sensor"
    echo "  --pcap <FILE>            Load PCAP recording"
    echo "  --metadata <JSON>        Metadata file for PCAP"
    echo ""
    echo "EXAMPLES:"
    echo "  ./launch_dashboard.sh --live 169.254.197.185"
    echo "  ./launch_dashboard.sh --pcap data.pcap --metadata config/sensor.json"
    echo ""
    exit 0
fi

# Check if Python dependencies are installed
python3 -c "import matplotlib" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing Python dependencies..."
    pip install matplotlib numpy 2>/dev/null
fi

# Run dashboard
echo "Launching Ouster LiDAR Dashboard..."
python3 dashboard.py "$@"
