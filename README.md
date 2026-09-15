# Stereo Dataset

Multi-sensor acquisition pipeline for a ZED 2 stereo camera, MTi-8 IMU,
AWR1843 radar, Ouster OS-0-64 lidar, and a GNSS receiver. Every sensor is
recorded independently at its own native rate — a missing camera frame never
removes an IMU/radar/lidar/GNSS sample. Synchronization is a separate,
derived, offline step (see `docs/SYNCHRONIZATION.md`), not a recording
precondition.

## Layout

| Path              | Purpose                                              |
|-------------------|------------------------------------------------------|
| `camera/`         | ZED 2 capture (`ZEDCamera`)                           |
| `mti8/`           | MTi-8 Xbus parser, serial reader, receiver            |
| `radar/`          | AWR1843 TLV parser, serial reader, receiver           |
| `lidar/`          | Ouster OS-0-64 receiver (native pcap + JSON recording)|
| `gnss/`           | NMEA parser, NTRIP client, receiver                   |
| `fusion/`         | Live preview matching (`Synchronizer`) + offline sync |
| `dataset/`        | Writer, reader, PyTorch dataset, HDF5/Parquet writers |
| `dashboard/`      | PySide6 live monitoring GUI                           |
| `calibration/`    | ZED calibration export/validation, Kalibr conversion  |
| `tools/`          | CLI utilities (CSV export, dataset migration)         |
| `tests/`          | Pytest tests                                          |
| `docs/`           | Dataset format specification                          |

Recordings are written under a single dataset root (default `output/`, see
`config.settings.DATASET_DIR`), and every recording session gets its own
self-contained subfolder within it, named by its session_id — a complete,
isolated copy of the whole directory tree (raw files, sensors, radar,
metadata, calibration, manifests), so no two sessions ever share or
overwrite a file. See `docs/DATASET_FORMAT.md` for the full on-disk layout.

## Setup

Python 3.10+ is required. For live ZED capture, install the
[ZED SDK](https://www.stereolabs.com) first; it provides `pyzed`, which is not
available from PyPI. The dashboard demo does not require ZED hardware.

```bash
# Download the project
git clone https://github.com/MPK-project-topic-PAMATAS/Data-Collection-GCS.git
cd Data-Collection-GCS

# Create .venv and install the project and its dependencies
./install.sh
```

If `python3` is not Python 3.10 or newer, choose a suitable interpreter:

```bash
PYTHON=python3.12 ./install.sh
```

If the repository was downloaded as a ZIP and the installer is not executable:

```bash
chmod +x install.sh
./install.sh
```

**Windows** (or any platform, since it's plain Python and does the same thing
as `install.sh` — no shell-specific script needed): run this from the
repository directory in PowerShell or cmd.exe (`py` on Windows, `python3`
elsewhere if `py` isn't on PATH):

```powershell
py install.py
```

### ZED SDK (required for live capture, not for the demo)

`pyzed` isn't on PyPI, so neither installer can pull it in automatically —
it has to come from the ZED SDK installer itself:

1. Install the [ZED SDK](https://www.stereolabs.com/developers/release/) for
   your platform (Linux, Windows, or Jetson) and matching CUDA version.
2. Activate the venv this project's installer created
   (`source .venv/bin/activate` on Linux/macOS,
   `.venv\Scripts\activate` on Windows).
3. Run the SDK's own installer script, which detects your Python version and
   installs the matching `pyzed` wheel into whichever Python is currently
   active:
   - Linux: `python /usr/local/zed/get_python_api.py`
   - Windows: `python "C:\Program Files (x86)\ZED SDK\get_python_api.py"`
     (adjust the path if you installed the SDK elsewhere)
4. Verify with `python -c "import pyzed"` — both `install.sh` and
   `install.py` print a reminder at the end if it's still missing.

Everything except live ZED capture (`gcs --demo`, `playback`,
`validate-dataset`, dataset tooling) works without the ZED SDK installed.

### Ouster OS-0-64 lidar

No vendor driver needed — `ouster-sdk` is on PyPI and installs cross-platform
via the optional extra:

```bash
pip install -e ".[lidar]"
```

The sensor connects over its own USB-Ethernet link (not a serial port); it
must be reachable at the link-local address in
`config/settings.py`'s `LIDAR_HOSTNAME` (default `169.254.197.185`). Windows
and Linux both auto-assign a link-local address to the adapter (APIPA);
if the sensor doesn't show up, set a static IP on that network adapter in
the `169.254.x.x` range rather than relying on auto-config.

### TI AWR1843 radar

No TI SDK install needed for this project — `radar/receiver.py` talks the
mmWave SDK's CLI/TLV protocol directly over two USB CDC-ACM serial ports (CFG
+ DATA); no Radar Studio, mmWave SDK, or UniFlash install required.

- **Linux**: works out of the box — the kernel's built-in `cdc_acm` driver
  enumerates the board as `/dev/ttyACM0`/`/dev/ttyACM1`, and
  `radar/ports.py` auto-detects which is which by USB VID:PID (`0451:bef3`).
- **Windows**: if the board's COM ports don't show up in Device Manager,
  install the XDS110 debug-probe driver (TI's "XDS Emulation Software
  Package" — also referenced in the AWR1843BOOST User's Guide §2.4). Once
  the ports enumerate, **`radar/ports.py`'s auto-detection is Linux-only**
  (it reads `/sys/class/tty`, which doesn't exist on Windows) — set
  `RADAR_CFG_PORT`/`RADAR_DATA_PORT` in `config/settings.py` to the actual
  `COMn` ports by hand (check Device Manager; the two ports enumerate
  adjacently, e.g. `COM5`/`COM6`, and the lower-numbered one is usually CFG).

### MTi-8 (Xsens)

No Xsens SDK/XDA needed either — `mti8/serial_reader.py` parses the raw Xbus
protocol directly over a plain serial port; the Xsens Device API mentioned in
some internal notes is unused by this Python pipeline.

- **Linux**: works out of the box (`/dev/ttyUSB*`).
- **Windows**: if the device doesn't enumerate as a COM port, install the
  Xsens USB driver (bundled with the Xsens MT Software Suite — the driver
  only, MT Manager itself isn't needed). Port auto-detection
  (`_auto_detect_port()`) does work cross-platform once the device shows up
  (it matches "motion"/"mti"/"xsens" in the port description), but if it
  still doesn't pick the right port, set `MTI_PORT` in `config/settings.py`
  to the actual `COMn` port.

## Start the dashboard

After installation, run this from the repository directory:

```bash
.venv/bin/record
```

This opens the Ground Control Station dashboard. Connect the ZED camera and
MTi-8 before starting it for live capture; GNSS is optional. Press **REC** in
the dashboard to begin recording. Recording requires at least 8 GB of free
space on the dataset output drive.

Alternatively, activate the virtual environment once per terminal session and
use the shorter commands:

```bash
source .venv/bin/activate
record
```

## Demo mode

```bash
# Same dashboard with simulated data (no hardware needed)
.venv/bin/gcs --demo
```

With the virtual environment activated, this is equivalent to `gcs --demo`.

## Other commands

```bash
# Validate a recorded dataset
.venv/bin/validate-dataset output/

# Playback a recorded dataset
.venv/bin/playback output/

# Build the derived synchronized index (offline, after recording)
.venv/bin/synchronize-dataset output/

# Headless recording (no dashboard)
.venv/bin/python main.py

# Run the test suite
.venv/bin/python -m pytest tests/
```
