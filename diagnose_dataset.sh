#!/usr/bin/env bash

set -u

REPORT="diagnostic_report_$(date +%Y%m%d_%H%M%S).txt"

exec > >(tee "$REPORT") 2>&1

echo "============================================================"
echo "DATA ACQUISITION PIPELINE DIAGNOSTIC REPORT"
echo "============================================================"
echo "Date: $(date)"
echo "PWD:  $(pwd)"
echo

echo "============================================================"
echo "1. SYSTEM INFORMATION"
echo "============================================================"

uname -a
echo
cat /etc/os-release 2>/dev/null || true
echo
echo "Python:"
python3 --version 2>/dev/null || true
echo
echo "ROS:"
ros2 --version 2>/dev/null || true
echo
echo "Git:"
git rev-parse --show-toplevel 2>/dev/null || true
git branch --show-current 2>/dev/null || true
git log -1 --oneline 2>/dev/null || true
echo

echo "============================================================"
echo "2. PROJECT STRUCTURE"
echo "============================================================"

echo "Top-level files:"
find . -maxdepth 2 -type f \
    ! -path './.git/*' \
    | sort \
    | head -300

echo

echo "Python files:"
find . -type f -name "*.py" \
    ! -path './.git/*' \
    | sort

echo

echo "Configuration files:"
find . -type f \( \
    -name "*.yaml" -o \
    -name "*.yml" -o \
    -name "*.json" -o \
    -name "*.toml" -o \
    -name "*.ini" -o \
    -name "*.cfg" -o \
    -name "*.conf" \
    \) \
    ! -path './.git/*' \
    | sort

echo

echo "============================================================"
echo "3. SEARCH FOR ACQUISITION / RECORDING CODE"
echo "============================================================"

grep -RniE \
    "AcquisitionThread|Synchronizer|SyncedFrame|DatasetWriter|writer|record|recording|start_recording|stop_recording|begin_session|end_session" \
    . \
    --include="*.py" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -1000

echo

echo "============================================================"
echo "4. SEARCH FOR SENSOR IMPLEMENTATIONS"
echo "============================================================"

echo "--- CAMERA / ZED ---"
grep -RniE \
    "ZED|zed|SVO|SVO2|camera|resolution|fps|FPS|quality|recording" \
    . \
    --include="*.py" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -1000

echo

echo "--- MTI / XSENS / IMU ---"
grep -RniE \
    "MTi|MTI|Xsens|xsens|xbus|imu|IMU|start_raw_log|timestamp" \
    . \
    --include="*.py" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -1000

echo

echo "--- LIDAR / OUSTER / PCAP ---"
grep -RniE \
    "Ouster|ouster|lidar|LiDAR|PCAP|pcap|start_pcap|rosbag|bag" \
    . \
    --include="*.py" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -1000

echo

echo "--- RADAR / AWR1843 / TI ---"
grep -RniE \
    "radar|Radar|AWR1843|awr1843|TI|ti_|heatmap|range_profile|doppler|PointCloud2|ADC|IQ|raw" \
    . \
    --include="*.py" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -1500

echo

echo "--- GNSS / GPS / PPS / PTP ---"
grep -RniE \
    "GNSS|gnss|GPS|gps|PPS|pps|PTP|ptp|NTP|ntp|clock|monotonic|CLOCK_REALTIME|CLOCK_MONOTONIC|time_ns" \
    . \
    --include="*.py" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -1500

echo

echo "============================================================"
echo "5. ROS2 NODES"
echo "============================================================"

timeout 10 ros2 node list 2>&1 || true

echo

echo "============================================================"
echo "6. ROS2 TOPICS"
echo "============================================================"

timeout 10 ros2 topic list -t 2>&1 || true

echo

echo "============================================================"
echo "7. ROS2 TOPIC RATES"
echo "============================================================"

TOPICS=$(timeout 10 ros2 topic list 2>/dev/null || true)

while IFS= read -r topic; do
    [ -z "$topic" ] && continue

    case "$topic" in
        *camera*|*zed*|*image*|*imu*|*lidar*|*points*|*radar*|*gnss*|*gps*|*clock*)
            echo
            echo "------------------------------------------------------------"
            echo "TOPIC: $topic"
            timeout 5 ros2 topic hz "$topic" 2>&1 || true
            ;;
    esac
done <<< "$TOPICS"

echo

echo "============================================================"
echo "8. ROS2 TOPIC TYPES AND DETAILS"
echo "============================================================"

while IFS= read -r line; do
    topic=$(echo "$line" | awk '{print $1}')
    [ -z "$topic" ] && continue

    case "$topic" in
        *camera*|*zed*|*image*|*imu*|*lidar*|*points*|*radar*|*gnss*|*gps*)
            echo
            echo "------------------------------------------------------------"
            echo "TOPIC: $topic"
            timeout 5 ros2 topic info "$topic" -v 2>&1 || true
            ;;
    esac
done <<< "$TOPICS"

echo

echo "============================================================"
echo "9. ONE SAMPLE FROM IMPORTANT ROS2 TOPICS"
echo "============================================================"

while IFS= read -r line; do
    topic=$(echo "$line" | awk '{print $1}')
    [ -z "$topic" ] && continue

    case "$topic" in
        *camera*|*zed*|*image*|*imu*|*lidar*|*points*|*radar*|*gnss*|*gps*)
            echo
            echo "------------------------------------------------------------"
            echo "TOPIC: $topic"
            timeout 5 ros2 topic echo "$topic" --once 2>&1 | head -150 || true
            ;;
    esac
done <<< "$TOPICS"

echo

echo "============================================================"
echo "10. TIMESTAMP IMPLEMENTATION"
echo "============================================================"

grep -RniE \
    "timestamp_ns|time_ns|monotonic_ns|time\.time_ns|time\.monotonic_ns|header\.stamp|builtin_interfaces|Time\(" \
    . \
    --include="*.py" \
    --include="*.cpp" \
    --include="*.hpp" \
    --include="*.h" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -2000

echo

echo "============================================================"
echo "11. SYNCHRONIZATION IMPLEMENTATION"
echo "============================================================"

grep -RniE \
    "get_nearest|nearest|sync\(|Synchronizer|SyncedFrame|MAX_DELTA|DELTA_NS|buffer|deque|ring|interpol|synchron" \
    . \
    --include="*.py" \
    --include="*.cpp" \
    --include="*.hpp" \
    --include="*.h" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -2000

echo

echo "============================================================"
echo "12. RECORDING FORMATS"
echo "============================================================"

grep -RniE \
    "\.svo2|\.svo|\.pcap|\.bag|\.xbus|\.h5|\.hdf5|\.json|\.npy|\.parquet|rosbag|PointCloud2" \
    . \
    --include="*.py" \
    --include="*.cpp" \
    --include="*.hpp" \
    --include="*.h" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -2000

echo

echo "============================================================"
echo "13. RADAR RAW DATA INVESTIGATION"
echo "============================================================"

grep -RniE \
    "adc|ADC|raw_data|rawData|iq|IQ|complex|chirp|chirps|sample|samples|frame|packet|LVDS|DCA1000|capture|record" \
    . \
    --include="*.py" \
    --include="*.cpp" \
    --include="*.hpp" \
    --include="*.h" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -2500

echo

echo "============================================================"
echo "14. CONFIGURATION VALUES"
echo "============================================================"

grep -RniE \
    "SAVE_|FPS|fps|resolution|quality|MAX_DELTA|BUFFER|buffer|RATE|rate|RECORD|record|PCAP|pcap|RADAR|radar|LIDAR|lidar|IMU|imu|GPS|gps|PPS|pps|PTP|ptp" \
    . \
    --include="*.py" \
    --include="*.yaml" \
    --include="*.yml" \
    --include="*.json" \
    --include="*.toml" \
    --exclude-dir=.git \
    2>/dev/null \
    | head -2500

echo

echo "============================================================"
echo "15. DATASET DIRECTORIES"
echo "============================================================"

for d in \
    raw \
    radar \
    lidar \
    depth \
    disparity \
    confidence \
    sensors \
    metadata \
    config
do
    if [ -d "$d" ]; then
        echo
        echo "DIRECTORY: $d"
        find "$d" -maxdepth 3 -type f 2>/dev/null \
            | sort \
            | head -300
    fi
done

echo

echo "============================================================"
echo "16. EXISTING RECORDED DATA"
echo "============================================================"

find . -type f \( \
    -name "*.svo2" -o \
    -name "*.svo" -o \
    -name "*.pcap" -o \
    -name "*.bag" -o \
    -name "*.xbus" -o \
    -name "*.h5" -o \
    -name "*.hdf5" \
    \) \
    ! -path './.git/*' \
    -exec ls -lh {} \; \
    2>/dev/null

echo

echo "============================================================"
echo "17. TIMESTAMP / FILE SAMPLE ANALYSIS"
echo "============================================================"

find . -type f \( \
    -name "*.json" -o \
    -name "*.parquet" \
    \) \
    ! -path './.git/*' \
    | sort \
    | head -50

echo

echo "============================================================"
echo "18. GIT STATUS / UNCOMMITTED IMPLEMENTATION"
echo "============================================================"

git status --short 2>/dev/null || true

echo

echo "============================================================"
echo "19. HARDWARE / USB DEVICES"
echo "============================================================"

lsusb 2>/dev/null || true

echo

echo "============================================================"
echo "20. SERIAL DEVICES"
echo "============================================================"

ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

echo

echo "============================================================"
echo "21. NETWORK INTERFACES"
echo "============================================================"

ip -br addr 2>/dev/null || true

echo

echo "============================================================"
echo "22. NTP / PTP / GPS CLOCK STATUS"
echo "============================================================"

echo "--- timedatectl ---"
timedatectl 2>/dev/null || true

echo
echo "--- chrony ---"
chronyc tracking 2>/dev/null || true
chronyc sources -v 2>/dev/null || true

echo
echo "--- PTP ---"
which ptp4l >/dev/null 2>&1 && ptp4l -v 2>&1 | head -20 || true
which pmc >/dev/null 2>&1 && pmc -u -b 0 'GET TIME_STATUS_NP' 2>&1 || true

echo

echo "============================================================"
echo "23. CURRENT CLOCK VALUES"
echo "============================================================"

python3 - <<'PY'
import time
import datetime

print("time.time_ns():       ", time.time_ns())
print("time.monotonic_ns():  ", time.monotonic_ns())
print("UTC:", datetime.datetime.now(datetime.timezone.utc).isoformat())
PY

echo

echo "============================================================"
echo "24. PROCESS INFORMATION"
echo "============================================================"

ps aux | grep -Ei \
    "ros|zed|ouster|radar|mti|xsens|dataset|record|gcs" \
    | grep -v grep \
    || true

echo

echo "============================================================"
echo "END OF REPORT"
echo "============================================================"

echo
echo "Report saved to:"
realpath "$REPORT"
