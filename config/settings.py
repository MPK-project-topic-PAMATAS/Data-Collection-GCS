from pyzed import sl

# Camera configuration
CAMERA_RESOLUTION = sl.RESOLUTION.HD720
CAMERA_FPS = 30

# Depth configuration
# NEURAL_LIGHT preserves substantially more detail than PERFORMANCE while
# reducing the GPU cost of full NEURAL for continuous 30 FPS acquisition.
DEPTH_MODE = sl.DEPTH_MODE.NEURAL_LIGHT
UNIT = sl.UNIT.METER
COORDINATE_SYSTEM = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP

# Depth range (meters)
DEPTH_MIN_M = 0.3
DEPTH_MAX_M = 30.0

# Depth stabilization: 1 = enabled (SDK default), 0 = disabled
DEPTH_STABILIZATION = 1

# Runtime parameters (applied every grab())
CONFIDENCE_THRESHOLD = 100          # 0–255, lower = more confident pixels kept
TEXTURE_CONFIDENCE_THRESHOLD = 100  # 0–255, lower = more texture-confidence kept

# Camera orientation: set to True when ZED is mounted upside-down on vehicle
# Applies a 180° rotation to left, right, depth, and disparity immediately
# after acquisition so the entire pipeline works with upright images.
CAMERA_FLIP_180 = False

# MTi-8
MTI_PORT = "/dev/ttyUSB0"
MTI_BAUDRATE = 2000000
MTI_TIMEOUT = 0.01

# ── MTi-8 yaw error correction ────────────────────────────────────────
# MTi-8 heading is gyro-integrated with no absolute yaw reference, so it
# drifts and carries any startup mount offset. When GNSS course-over-ground
# is valid and the vehicle moves faster than MIN_SPEED, the dashboard pulls
# the displayed yaw toward GNSS course. Recording is unaffected — raw MTi
# data is always written unmodified.
MTI_YAW_GNSS_CORRECTION_ENABLED = True
MTI_YAW_GNSS_MIN_SPEED_MPS = 1.5
MTI_YAW_CORRECTION_GAIN = 0.05   # bias convergence: deg/s per deg of error

# Recording
MAX_FRAMES = 1800          # 1 minute at 30 FPS (60 seconds)
DATASET_DIR = "output"

# Dataset format
# All ZED2 video and IMU is recorded inside the SVO2 file; no per-frame
# images are written.  Dense depth maps are saved per frame.

# Depth is the canonical dense measurement.  Disparity is derivable from it,
# and confidence is only needed for dedicated depth-quality sessions.  Keeping
# both off by default avoids writing 7.0 MiB of extra float maps per HD720 frame.
SAVE_DEPTH = False         # Disable to save ~210 MB per minute
SAVE_DISPARITY = False
SAVE_CONFIDENCE = False

# Refreshing full-frame reductions is expensive and the dashboard does not need
# them at camera rate.
DASHBOARD_METRICS_INTERVAL_S = 1.0
HDF5_WRITE_BATCH_SIZE = 256
HDF5_BATCH_MAX_LATENCY_S = 0.05
# SVO2 compression. Must be one of the sl.SVO_COMPRESSION_MODE members that
# the installed ZED SDK actually exposes — validated eagerly in
# camera/zed_camera.py.start_recording() (raises ValueError, never silently
# falls back). "LOSSLESS" here means H264_LOSSLESS, not raw/uncompressed.
SVO_COMPRESSION_MODE = "H264_LOSSLESS"
SAVE_SVO = True            # save ZED SVO2 recording (~200 MB per minute)
SAVE_RAW_IMU = True        # save raw MTi-8 Xbus binary (~5 MB per minute)
SAVE_RADAR_BAG = True      # save radar as a ROS1 .bag (~50 MB per minute)
CALIBRATION_VERSION = "v1.0"

# TI AWR1843BOOST radar
RADAR_CFG_PORT = "/dev/ttyACM0"
RADAR_CFG_BAUD = 115200
RADAR_DATA_PORT = "/dev/ttyACM1"
RADAR_DATA_BAUD = 921600
RADAR_BUFFER_SIZE = 60
# These must match the active AWR1843 profile in radar/receiver.py.
RADAR_NUM_RANGE_BINS = 256   # 256 ADC samples (heatmap output is off in 3TX)
# Range per bin for profileCfg in radar/receiver.py:
# c * fs / (2 * slope * N) = c * 5.209e6 / (2 * 70e12 * 256) ~= 0.0436 m
RADAR_RANGE_RES_M = 0.0436
RADAR_NUM_VIRT_ANT = 12  # 4 RX x 3 TX (elevation-capable config)
# frameCfg uses 16 loops (3TX: TX0/TX1/TX2). No range-Doppler/azimuth heatmap
# TLV is produced; this constant is dormant.
RADAR_NUM_DOPPLER_BINS = 16
MAX_DELTA_NS = 20_000_000       # max camera↔IMU sync delta (20 ms)
# frameCfg uses a 100 ms frame period (3TX config); nearest-frame matching
# must accept its 50 ms max half-period separation. 80 ms keeps margin.
RADAR_MAX_DELTA_NS = 80_000_000
# A serial port can remain open while no complete UART frames arrive.  Treat
# the radar as offline only after this liveness interval, not after one camera
# frame fails to find a timestamp match.
RADAR_STREAM_TIMEOUT_NS = 1_000_000_000
# If the DATA port stays open but no frames arrive for this long, the sensor
# has likely reset (e.g. a transient USB brownout on the shared CDC-ACM link)
# without the driver knowing — the port reconnects but the chip forgot its
# config and never got sensorStart again. The watchdog in RadarReceiver
# reapplies _SENSOR_CONFIG_CMDS + sensorStart once idle exceeds this.
# Set well above RADAR_STREAM_TIMEOUT_NS so it never fights RadarSerialReader's
# own port-reconnect retries (which back off up to 5 s).
RADAR_RECONFIG_TIMEOUT_NS = 3_000_000_000

# Radar->camera extrinsic (lever arm), in the radar's own frame (x=right,
# y=forward, z=up — see radar/parser.py's TLV docstring), meters, measured
# from the AWR1843's antenna array phase center (the patch-antenna region
# on the BOOST board, not the board edge or connector) to the ZED's LEFT
# lens optical center, since camera.calib's fx/fy/cx/cy are the left eye's.
# From a hand sketch/ruler measurement on the physical rig: camera left
# lens sits ~5.5cm to the radar's left (dx), ~1.5cm below it (dz), and
# ~0.4cm forward of it (dy) — the ZED's lens barrel visibly protrudes
# ahead of the radar PCB's front face on the shared bracket. dy's exact
# sign wasn't independently verified (magnitude only: 0.3-0.5cm), but its
# effect is small next to dx/dz at any range this rig operates at.
# Update all three after every physical remount — see radar/projection.py
# and docs/RADAR_CAMERA_OVERLAY.md.
RADAR_TO_CAMERA_OFFSET_M = (-0.055, 0.004, -0.015)

# ── Ouster OS-0-64-UO2 lidar ─────────────────────────────────────────
# Hostname/IP of the lidar on the local network (via its HTTP API / UDP).
# The ouster SDK connects to the sensor from this address on the same subnet.
LIDAR_HOSTNAME = "169.254.197.185"  # OS-0-64's IP (USB-Ethernet link-local peer)
# Nearest-frame sync tolerance. The OS-0 at 10 Hz yields a 100 ms frame
# period, so accept up to half that (50 ms) when matching to a camera frame.
LIDAR_MAX_DELTA_NS = 50_000_000
# How long without a live frame before the dashboard reports the lidar
# as offline (avoids false alarms between 10 Hz frames).
LIDAR_STREAM_TIMEOUT_NS = 1_000_000_000

# Ouster timestamp source.  None leaves the sensor's current setting alone.
# "TIME_FROM_INTERNAL_OSC" free-runs from power-on (cannot be related to the
# host clock except by host-arrival stamps).  "TIME_FROM_PTP_1588" slaves the
# sensor to a PTP grandmaster (run ptp4l on the laptop NIC - see
# docs/SYNCHRONIZATION.md).  "TIME_FROM_SYNC_PULSE_IN" uses a 1 PPS + NMEA
# input.  Applying a mode re-initialises the sensor once at startup.
LIDAR_TIMESTAMP_MODE = None

# GNSS fixes arrive at ~1-10 Hz (device/mountpoint dependent) and are not
# buffered with a strict on-device timestamp the way MTi/radar/lidar are;
# this tolerance is only used by the *offline* synchronizer
# (fusion/offline_sync.py), never to gate recording.
GNSS_MAX_DELTA_NS = 200_000_000

# ── IMU source configuration ─────────────────────────────────────────
# ZED IMU is optional diagnostic logging only. MTi-8 is the sole
# navigation/orientation sensor. ZED IMU alignment with MTi-8 failed
# (40° RMS residual) — never load alignment matrices in production.
# All ZED2 video and IMU is captured inside the SVO2 file itself.
ENABLE_ZED_IMU_LOGGING = False      # enable only for diagnostic sessions
