"""Per-session manifest: must reference the single pcap + single config
JSON (never per-frame files), the other recorders' actual output paths,
start/stop timing, and synchronization metadata — without ever claiming
synchronization was applied during recording."""

import sys
from types import SimpleNamespace, ModuleType

if not hasattr(sys.modules.get("pyzed"), "sl"):
    zed_module = ModuleType("pyzed")
    zed_module.sl = SimpleNamespace(
        RESOLUTION=SimpleNamespace(HD720=object()),
        DEPTH_MODE=SimpleNamespace(NEURAL_LIGHT=object()),
        UNIT=SimpleNamespace(METER=object()),
        COORDINATE_SYSTEM=SimpleNamespace(RIGHT_HANDED_Z_UP=object()),
    )
    sys.modules["pyzed"] = zed_module

from dataset.session_manifest import build_session_manifest
from dataset.writer import DatasetWriter


def test_session_manifest_references_single_pcap_and_json(tmp_path):
    manifest = build_session_manifest(
        root=tmp_path,
        session_id="20260101_000000",
        start_timestamp_ns=1_000_000_000,
        stop_timestamp_ns=2_000_000_000,
        camera_svo2=str(tmp_path / "raw" / "recording.svo2"),
        mti_raw_xbus=str(tmp_path / "raw" / "mti.xbus"),
        ouster_pcap=str(tmp_path / "raw" / "lidar.pcap"),
        ouster_config_json=str(tmp_path / "raw" / "lidar_0.json"),
        radar_bag=str(tmp_path / "raw" / "radar.bag"),
        gnss_nmea_log=str(tmp_path / "raw" / "gnss.nmea.log"),
    )

    assert manifest["session_id"] == "20260101_000000"
    assert manifest["start_timestamp_ns"] == 1_000_000_000
    assert manifest["stop_timestamp_ns"] == 2_000_000_000
    assert manifest["duration_s"] == 1.0

    files = manifest["files"]
    assert files["ouster_pcap"] == "raw/lidar.pcap"
    assert files["ouster_config_json"] == "raw/lidar_0.json"
    # Exactly one pcap path and one config JSON path — not a list, not a dir.
    assert isinstance(files["ouster_pcap"], str)
    assert isinstance(files["ouster_config_json"], str)
    assert files["camera_svo2"] == "raw/recording.svo2"
    assert files["mti_raw_xbus"] == "raw/mti.xbus"
    assert files["radar_bag"] == "raw/radar.bag"
    assert files["gnss_nmea_log"] == "raw/gnss.nmea.log"

    assert manifest["synchronization"]["applied_during_recording"] is False


def test_session_manifest_omits_unrecorded_sensors(tmp_path):
    manifest = build_session_manifest(
        root=tmp_path,
        session_id="s1",
        start_timestamp_ns=0,
        stop_timestamp_ns=0,
    )
    files = manifest["files"]
    assert files["ouster_pcap"] is None
    assert files["ouster_config_json"] is None
    assert files["radar_bag"] is None
    assert files["radar_native_structured"] is None
    assert files["gnss_nmea_log"] is None
    assert manifest["gnss_structured"] is None


def test_writer_save_session_manifest_writes_json(tmp_path):
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=False)
    manifest = build_session_manifest(
        root=tmp_path, session_id=writer.session_id,
        start_timestamp_ns=1, stop_timestamp_ns=2,
        ouster_pcap=str(tmp_path / "raw" / "lidar.pcap"),
        ouster_config_json=str(tmp_path / "raw" / "lidar_0.json"),
    )
    writer.save_session_manifest(manifest)
    writer.close()

    path = tmp_path / "metadata" / f"session_manifest_{writer.session_id}.json"
    assert path.exists()
    import json
    with open(path) as f:
        saved = json.load(f)
    assert saved["files"]["ouster_pcap"] == "raw/lidar.pcap"
