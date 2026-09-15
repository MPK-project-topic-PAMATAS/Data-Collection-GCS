"""Launch the Ground Control Station with live sensor integration.

Architecture:
  AcquisitionThread  →  SharedFrame  →  Dashboard (read-only)
  (own thread)              ↑                (Qt main thread)

Usage:
    python run_gcs.py              # GCS with live sensors
    python run_gcs.py --demo       # Demo mode with simulated data
    python run_gcs.py --no-gnss    # Skip GNSS receiver
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from PySide6.QtWidgets import QApplication

_SAFE_FAILED = object()  # sentinel: distinguishable from any real return value


def _safe(log, description, fn, *args, **kwargs):
    """Run fn, logging and swallowing any exception instead of letting it
    escape a Qt slot. An uncaught Python exception inside a PySide6 slot
    (e.g. camera.start_recording(), lidar.start_pcap()) can abort the whole
    process rather than just failing that call — every individual recorder
    start/stop must be isolated like this so one sensor failing (a bad SVO
    compression setting, a disconnected lidar, a full disk) never takes
    down recording for every other sensor, or the app itself.

    Returns fn's return value, or the _SAFE_FAILED sentinel if it raised
    (never None on failure, since fn's own successful return value may
    legitimately be None).
    """
    try:
        return fn(*args, **kwargs)
    except Exception as e:
        log(f"{description} failed: {e}", "error")
        return _SAFE_FAILED


def build_python_sources(window, use_gnss, use_radar, use_lidar):
    """Original per-sensor Python adapters (fallback path)."""
    from camera.zed_camera import ZEDCamera
    from mti8.receiver import MTiReceiver
    from fusion.synchronizer import Synchronizer
    from dataset.writer import DatasetWriter
    from config.settings import DATASET_DIR

    camera = ZEDCamera()
    mti = MTiReceiver()
    radar = None
    lidar = None
    gnss = None

    if use_gnss:
        try:
            from gnss.receiver import GNSSReceiver
            from config.ntrip_credentials import (
                NTRIP_CASTERS, DEFAULT_MOUNTPOINT,
            )
            caster = NTRIP_CASTERS[0]
            gnss = GNSSReceiver(
                serial_port="/dev/ttyUSB1",
                baudrate=115200,
                ntrip_host=caster["host"],
                ntrip_port=caster["port"],
                ntrip_user=caster["username"],
                ntrip_pass=caster["password"],
                mountpoint=DEFAULT_MOUNTPOINT,
            )
        except Exception as e:
            window.log_panel.log(f"GNSS config error: {e}", "warning")
            gnss = None

    if use_radar:
        try:
            from radar.receiver import RadarReceiver
            radar = RadarReceiver()
            radar.start()
            window.log_panel.log("AWR1843 radar connected", "success")
        except Exception as e:
            window.log_panel.log(f"Radar connection failed: {e}", "error")
            radar = None

    if use_lidar:
        try:
            from lidar.receiver import LidarReceiver
            lidar = LidarReceiver()
            lidar.start()
            if lidar.sensor_name:
                window.log_panel.log(
                    f"Ouster {lidar.sensor_name} lidar connected", "success"
                )
            else:
                window.log_panel.log("Ouster lidar connected", "success")
        except Exception as e:
            window.log_panel.log(f"Lidar connection failed: {e}", "error")
            lidar = None

    sync = Synchronizer(mti, gnss, radar, lidar)
    writer = DatasetWriter(root=DATASET_DIR,
                           enable_radar=use_radar)

    try:
        camera.open()
        window.log_panel.log("ZED 2 camera connected", "success")
    except Exception as e:
        window.log_panel.log(f"ZED connection failed: {e}", "error")

    try:
        mti.start()
        window.log_panel.log("MTi-8 IMU connected", "success")
        if mti.firmware:
            writer.set_mti_firmware(mti.firmware)
    except Exception as e:
        window.log_panel.log(f"MTi connection failed: {e}", "error")

    if gnss:
        try:
            gnss.start()
            if gnss.ntrip_connected:
                window.log_panel.log("GNSS NTRIP connected", "success")
            else:
                window.log_panel.log(
                    "GNSS serial, NTRIP pending", "warning"
                )
        except Exception as e:
            window.log_panel.log(f"GNSS connection failed: {e}", "error")

    try:
        writer.save_calibration(camera.calib)
        if camera.calib.get("sdk_version"):
            writer.set_zed_sdk(camera.calib["sdk_version"])
    except Exception:
        pass

    return _Sources(camera=camera, mti=mti, gnss=gnss, radar=radar, lidar=lidar,
                    sync=sync, writer=writer)


def build_native_sources(window, use_gnss, use_radar, use_lidar):
    """Native C++ acquisition backends, mirroring the Python interface."""
    import sensor_backend
    from fusion.synchronizer import Synchronizer
    from dataset.writer import DatasetWriter
    from config.settings import DATASET_DIR

    if not sensor_backend.available():
        raise RuntimeError("native backend unavailable")

    from nativeadapter import NativeAcquisition

    acq = NativeAcquisition(
        enable_zed=True,
        enable_mti8=True,
        enable_radar=use_radar,
        enable_ouster=use_lidar,
        enable_gnss=use_gnss,
    )
    if not acq.start():
        window.log_panel.log(
            "Native acquisition start returned failure (sensors may be offline)",
            "warning",
        )

    camera, mti, radar, lidar, gnss = acq.camera, acq.mti, acq.radar, acq.lidar, acq.gnss
    if use_radar:
        radar.start()
    if use_lidar:
        lidar.start()
    gnss.start()
    mti.start()

    try:
        camera.open()
        window.log_panel.log("ZED 2 camera connected (native)", "success")
    except Exception as e:
        window.log_panel.log(f"ZED connection failed (native): {e}", "error")

    window.log_panel.log("MTi-8 IMU connected (native)", "success")
    if gnss:
        window.log_panel.log("GNSS connected (native)", "success")

    sync = Synchronizer(mti, gnss, radar, lidar)
    writer = DatasetWriter(root=DATASET_DIR,
                           enable_radar=use_radar)
    try:
        if getattr(mti, "firmware", None):
            writer.set_mti_firmware(mti.firmware)
    except Exception:
        pass
    try:
        writer.save_calibration(camera.calib)
        if camera.calib.get("sdk_version"):
            writer.set_zed_sdk(camera.calib["sdk_version"])
    except Exception:
        pass

    return _Sources(camera=camera, mti=mti, gnss=gnss, radar=radar, lidar=lidar,
                    sync=sync, writer=writer)


class _Sources:
    """Uniform sensor/backend container consumed by the shared GCS loop."""

    __slots__ = ("camera", "mti", "gnss", "radar", "lidar", "sync", "writer")

    def __init__(self, camera, mti, gnss, radar, lidar, sync, writer):
        self.camera = camera
        self.mti = mti
        self.gnss = gnss
        self.radar = radar
        self.lidar = lidar
        self.sync = sync
        self.writer = writer


def run_live(use_gnss=True, use_radar=True, use_lidar=True):
    from dashboard.main_window import MainWindow
    from dashboard.acquisition import AcquisitionThread, SharedFrame
    from config.settings import SAVE_SVO, SAVE_RAW_IMU, SAVE_RADAR_BAG

    app = QApplication(sys.argv)

    shared = SharedFrame()
    window = MainWindow(shared)
    window.show()

    # Native C++ acquisition when available; original Python adapters otherwise.
    try:
        import sensor_backend
        if sensor_backend.available():
            src = build_native_sources(window, use_gnss, use_radar, use_lidar)
            window.log_panel.log("Using native C++ acquisition backend", "info")
        else:
            src = build_python_sources(window, use_gnss, use_radar, use_lidar)
    except Exception as e:
        window.log_panel.log(
            f"Native backend init failed ({e}); using Python adapters",
            "warning",
        )
        src = build_python_sources(window, use_gnss, use_radar, use_lidar)

    camera, mti, radar, lidar, gnss = src.camera, src.mti, src.radar, src.lidar, src.gnss
    sync, writer = src.sync, src.writer

    # Radar panel projects radar points onto the camera image using the
    # camera's own intrinsics (see dashboard/radar_panel.py's
    # RadarCameraOverlay) — needs camera.calib, only available once open().
    if getattr(camera, "calib", None):
        window.radar_panel.set_camera_calib(camera.calib)

    # Start acquisition in dedicated thread
    acq = AcquisitionThread(camera, sync, writer, shared)
    acq.start()

    window.log_panel.log("Acquisition thread started", "success")
    window.log_panel.log("GCS ready — use REC to start recording", "info")

    # ── Recording hooks ─────────────────────────────────────────────

    # Radar bag recorder + poller (independent of camera sync).
    radar_rec = {"poller": None}
    # Native-rate structured recorders (MTi HDF5, radar JSON/heatmap, GNSS
    # fixes) — independent of camera sync, unlike the old write(SyncedFrame)
    # path. See dataset/*_structured_recorder.py.
    struct_pollers = {"mti": None, "radar": None, "gnss": None}
    # Actual per-session output paths, for the session manifest (on_stop).
    session_files = {
        "start_ns": 0, "svo": None, "xbus": None,
        "pcap": None, "pcap_json": None, "bag": None, "nmea": None,
    }

    def on_start():
        from dashboard.gate import run_health_check
        import time
        log = window.log_panel.log

        result = _safe(log, "Health check", run_health_check, camera, mti,
                       gnss, writer, sync, radar=radar, lidar=lidar)
        if result is _SAFE_FAILED:
            log("Recording blocked — health check itself failed", "error")
            return
        all_ok, checks = result
        for c in checks:
            level = "success" if c.ok else ("error" if c.critical else "warning")
            icon = "✓" if c.ok else "✗"
            log(f"{icon} {c.name}: {c.message}", level)
        if not all_ok:
            log("Recording blocked — fix critical issues first", "error")
            return
        start_ns = time.time_ns()
        session_files["start_ns"] = start_ns
        if _safe(log, "Begin session", writer.begin_session, start_ns) is _SAFE_FAILED:
            log("Recording blocked — could not open a session", "error")
            return
        acq.set_recording(True)
        log(f"Session {writer.session_id} started", "info")

        # Immutable per-session config snapshot (never includes secrets).
        def _save_config_snapshot():
            from dataset.config_snapshot import build_config_snapshot
            snapshot = build_config_snapshot(
                camera.calib, mti, radar, lidar, gnss, writer.session_id,
            )
            writer.save_config_snapshot(snapshot)
        _safe(log, "Config snapshot", _save_config_snapshot)

        # Raw filenames no longer need a session_id suffix — begin_session()
        # (above) already switched writer.raw_dir to a fresh, dedicated
        # <dataset_root>/<session_id>/raw/ subfolder, so plain names can
        # never collide with another session's (an external dataset audit
        # flagged the earlier design, where all sessions shared one flat
        # raw/ folder, as a real risk — this is the folder-per-session fix).

        # SVO recording
        if SAVE_SVO:
            svo_path = str(writer.raw_dir / "recording.svo2")
            if _safe(log, "SVO recording", camera.start_recording, svo_path) is not _SAFE_FAILED:
                session_files["svo"] = svo_path
                log(f"SVO: {svo_path}", "info")
        # Raw IMU recording
        if SAVE_RAW_IMU:
            raw_path = str(writer.raw_dir / "mti.xbus")
            if _safe(log, "Raw IMU log", mti.start_raw_log, raw_path) is not _SAFE_FAILED:
                session_files["xbus"] = raw_path
                log(f"Raw IMU: {raw_path}", "info")
        # Lidar pcap recording — exactly one pcap + one config JSON per
        # session, produced entirely by LidarReceiver at native rate,
        # independent of the camera and of Synchronizer.
        if lidar is not None:
            pcap_path = str(writer.raw_dir / "lidar.pcap")
            if _safe(log, "Lidar pcap recording", lidar.start_pcap, pcap_path) is not _SAFE_FAILED:
                session_files["pcap"] = pcap_path
                session_files["pcap_json"] = getattr(lidar, "config_json_path", None)
                log(f"Lidar pcap: {pcap_path}", "info")
        # Radar ROS1 .bag recording (every frame, independent of camera sync)
        if radar is not None and SAVE_RADAR_BAG:
            def _start_radar_bag():
                from dataset.radar_bag_recorder import (
                    RadarBagRecorder, RadarBagPoller,
                )
                bag_path = str(writer.raw_dir / "radar.bag")
                rec = RadarBagRecorder()
                rec.open(bag_path)
                poller = RadarBagPoller(radar, rec)
                poller.start()
                radar_rec["poller"] = poller
                session_files["bag"] = bag_path
                log(f"Radar bag: {bag_path}", "info")
            _safe(log, "Radar bag recording", _start_radar_bag)

        # Native-rate structured recorders — independent of camera sync.
        def _start_mti_poller():
            from dataset.mti_structured_recorder import MTiStructuredPoller
            mti_poller = MTiStructuredPoller(mti, writer)
            mti_poller.start()
            struct_pollers["mti"] = mti_poller
        _safe(log, "MTi structured recorder", _start_mti_poller)

        if radar is not None:
            def _start_radar_poller():
                from dataset.radar_structured_recorder import RadarStructuredPoller
                radar_poller = RadarStructuredPoller(radar, writer)
                radar_poller.start()
                struct_pollers["radar"] = radar_poller
            _safe(log, "Radar structured recorder", _start_radar_poller)

        if gnss is not None:
            def _start_gnss_poller():
                from dataset.gnss_structured_recorder import GNSSStructuredPoller
                nmea_path = str(writer.raw_dir / "gnss.nmea.log")
                if hasattr(gnss, "start_raw_log"):
                    gnss.start_raw_log(nmea_path)
                    session_files["nmea"] = nmea_path
                    log(f"Raw NMEA: {nmea_path}", "info")
                gnss_poller = GNSSStructuredPoller(gnss, writer)
                gnss_poller.start()
                struct_pollers["gnss"] = gnss_poller
            _safe(log, "GNSS structured recorder", _start_gnss_poller)

    def on_stop():
        import time
        log = window.log_panel.log
        acq.set_recording(False)

        # Stop structured pollers before drop-stat counters are read so the
        # final numbers reflect everything that was actually written. Each
        # stop is isolated — one poller failing to stop cleanly must not
        # skip stopping the rest, or crash the app on STOP.
        for key in ("mti", "radar", "gnss"):
            poller = struct_pollers.get(key)
            if poller is not None:
                _safe(log, f"Stop {key} structured recorder", poller.stop)
                struct_pollers[key] = None
        if gnss is not None and hasattr(gnss, "stop_raw_log"):
            _safe(log, "Stop GNSS raw log", gnss.stop_raw_log)

        # Drop/gap statistics — recorded as events, not silently discarded.
        try:
            if hasattr(mti, "counters"):
                total, valid, invalid, dropped, suspicious = mti.counters
                writer.add_event(
                    time.time_ns(), "MTI_COUNTERS",
                    f"total={total} valid={valid} invalid={invalid} "
                    f"dropped={dropped} suspicious={suspicious}",
                )
            if radar is not None and hasattr(radar, "dropped_frames"):
                writer.add_event(
                    time.time_ns(), "RADAR_DROPPED_FRAMES",
                    f"dropped={radar.dropped_frames}",
                )
            if lidar is not None and hasattr(lidar, "dropped_frames"):
                writer.add_event(
                    time.time_ns(), "LIDAR_DROPPED_FRAMES",
                    f"dropped={lidar.dropped_frames}",
                )
        except Exception:
            pass

        stop_ns = time.time_ns()
        _safe(log, "End session", writer.end_session, stop_ns)
        log(f"Session {writer.session_id} ended", "info")
        if SAVE_SVO:
            _safe(log, "Stop SVO recording", camera.stop_recording)
        if SAVE_RAW_IMU:
            _safe(log, "Stop raw IMU log", mti.stop_raw_log)
        if lidar is not None:
            _safe(log, "Stop lidar pcap", lidar.stop_pcap)

            def _ensure_pcap_config_json():
                # Exactly one config JSON must exist alongside the one pcap
                # — if LidarReceiver.start_pcap() couldn't write it (e.g.
                # sensor info was never available), generate a minimal
                # fallback so the pcap is still replayable. Never per-frame.
                if session_files.get("pcap_json"):
                    return
                import json
                from pathlib import Path

                pcap_file = Path(session_files["pcap"]) if session_files.get("pcap") else None
                if not (pcap_file and pcap_file.exists()):
                    return
                md_file = pcap_file.with_name(f"{pcap_file.stem}_0.json")
                if not md_file.exists():
                    metadata = {
                        "prod_line": "OS-0-64-U02",
                        "lidar_mode": "1024x10",
                        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    }
                    with open(md_file, "w", encoding="utf-8") as f:
                        json.dump(metadata, f, indent=2)
                    log(f"Generated PCAP metadata: {md_file.name}", "info")
                session_files["pcap_json"] = str(md_file)
            _safe(log, "Generate fallback lidar config JSON", _ensure_pcap_config_json)

        poller = radar_rec.get("poller")
        if poller is not None:
            _safe(log, "Close radar bag", poller.close)
            radar_rec["poller"] = None

        # Per-session manifest — the single pcap + single config JSON
        # (not per-frame files), plus every other recorder's output path.
        def _save_session_manifest():
            from dataset.session_manifest import build_session_manifest
            manifest = build_session_manifest(
                root=writer.root,
                session_id=writer.session_id,
                start_timestamp_ns=session_files["start_ns"],
                stop_timestamp_ns=stop_ns,
                camera_svo2=session_files["svo"],
                mti_raw_xbus=session_files["xbus"],
                ouster_pcap=session_files["pcap"],
                ouster_config_json=session_files["pcap_json"],
                radar_bag=session_files["bag"],
                gnss_nmea_log=session_files["nmea"],
            )
            writer.save_session_manifest(manifest)
        _safe(log, "Session manifest", _save_session_manifest)

    window.recorder_panel.sig_start.connect(on_start)
    window.recorder_panel.sig_stop.connect(on_stop)

    app.exec()

    acq.stop()
    acq.join(timeout=5)
    try:
        writer.close()
    except Exception:
        pass
    try:
        camera.close()
    except Exception:
        pass
    try:
        mti.stop()
    except Exception:
        pass
    if gnss:
        try:
            gnss.stop()
        except Exception:
            pass
    if radar:
        try:
            radar.stop()
        except Exception:
            pass
    if lidar:
        try:
            lidar.stop()
        except Exception:
            pass


def run_demo():
    from dashboard.main_window import run_demo as _run_demo
    _run_demo()


def main():
    parser = argparse.ArgumentParser(
        description="Ground Control Station"
    )
    parser.add_argument(
        "--demo", action="store_true", help="Run with simulated data"
    )
    parser.add_argument(
        "--no-gnss", action="store_true", help="Skip GNSS receiver"
    )
    parser.add_argument(
        "--no-radar", action="store_true", help="Skip AWR1843 radar receiver"
    )
    parser.add_argument(
        "--no-lidar", action="store_true", help="Skip Ouster OS-0 lidar"
    )
    args = parser.parse_args()

    if args.demo:
        run_demo()
    else:
        run_live(use_gnss=not args.no_gnss, use_radar=not args.no_radar,
                 use_lidar=not args.no_lidar)


if __name__ == "__main__":
    main()
