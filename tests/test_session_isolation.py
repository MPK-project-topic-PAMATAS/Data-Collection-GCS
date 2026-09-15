"""Regression for two real bugs found live:

1. Raw filenames (recording.svo2, mti.xbus, lidar.pcap, radar.bag,
   gnss.nmea.log) were fixed and shared across the whole continuous
   dataset root — a second REC in one app run silently overwrote (or, for
   the radar bag, refused to overwrite) the first session's raw files.
2. The interim fix (suffixing filenames with session_id) only worked if
   session_id actually changed per session — it didn't (see
   test_multi_session_ids.py) — and even fixed, suffixed-but-flat files
   are harder to work with than one folder per session.

The final design: every session gets its own subfolder,
<dataset_root>/<session_id>/, self-contained (depth/, sensors/, radar/,
metadata/, calibration/, manifests/, raw/) — see
DatasetWriter.begin_session(). This covers dataset.reader.resolve_lidar_paths()
and list_sessions(), the pieces that let readers find a given (or the most
recent) session's files back again.
"""

import sys
import time
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

from dataset.reader import resolve_lidar_paths, list_sessions


def _touch(path, content=b"x"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


def _make_session(root, session_id, with_lidar=True):
    session_dir = root / session_id
    _touch(session_dir / "raw" / "placeholder.xbus")  # any file — raw/ must exist
    if with_lidar:
        _touch(session_dir / "raw" / "lidar.pcap")
        _touch(session_dir / "raw" / "lidar_0.json")
    return session_dir


def test_two_sessions_have_separate_folders_and_lidar_files(tmp_path):
    _make_session(tmp_path, "20260101_100000")
    _make_session(tmp_path, "20260101_110000")

    pcap1, json1 = resolve_lidar_paths(tmp_path, session_id="20260101_100000")
    pcap2, json2 = resolve_lidar_paths(tmp_path, session_id="20260101_110000")

    assert pcap1 != pcap2, "two sessions resolved to the same pcap file"
    assert pcap1.parent.parent.name == "20260101_100000"
    assert pcap2.parent.parent.name == "20260101_110000"
    assert pcap1.name == pcap2.name == "lidar.pcap"  # same filename, different folders
    assert json1.name == json2.name == "lidar_0.json"


def test_missing_session_id_returns_none_not_a_wrong_session(tmp_path):
    _make_session(tmp_path, "20260101_100000")

    pcap, meta = resolve_lidar_paths(tmp_path, session_id="does_not_exist")
    assert pcap is None
    assert meta is None


def test_no_session_id_picks_most_recently_modified_session(tmp_path):
    _make_session(tmp_path, "20260101_100000")
    time.sleep(0.01)
    _make_session(tmp_path, "20260101_110000")

    pcap, meta = resolve_lidar_paths(tmp_path)
    assert pcap.parent.parent.name == "20260101_110000"


def test_list_sessions_orders_most_recent_first(tmp_path):
    _make_session(tmp_path, "20260101_100000")
    time.sleep(0.01)
    _make_session(tmp_path, "20260101_110000")
    time.sleep(0.01)
    _make_session(tmp_path, "20260101_120000")

    sessions = list_sessions(tmp_path)
    assert sessions == ["20260101_120000", "20260101_110000", "20260101_100000"]


def test_legacy_flat_filename_still_readable(tmp_path):
    """Datasets recorded before session-scoped folders existed must still
    be readable."""
    raw = tmp_path / "raw"
    _touch(raw / "lidar.pcap")
    _touch(raw / "lidar_0.json")

    pcap, meta = resolve_lidar_paths(tmp_path)
    assert pcap.name == "lidar.pcap"
    assert meta.name == "lidar_0.json"


def test_legacy_flat_suffixed_filename_still_readable(tmp_path):
    """Datasets recorded during the brief interim (filename-suffixed, not
    folder-per-session) design must still be readable when session_id is
    known."""
    raw = tmp_path / "raw"
    _touch(raw / "lidar_20260101_100000.pcap")
    _touch(raw / "lidar_20260101_100000_0.json")

    pcap, meta = resolve_lidar_paths(tmp_path, session_id="20260101_100000")
    assert pcap.name == "lidar_20260101_100000.pcap"
    assert meta.name == "lidar_20260101_100000_0.json"


def test_session_without_lidar_is_skipped_by_auto_pick(tmp_path):
    """A session that recorded no lidar must not make auto-pick (no
    session_id given) return nothing just because it's the newest folder."""
    _make_session(tmp_path, "20260101_100000", with_lidar=True)
    time.sleep(0.01)
    _make_session(tmp_path, "20260101_110000", with_lidar=False)  # no lidar this session

    pcap, meta = resolve_lidar_paths(tmp_path)
    assert pcap is not None
    assert pcap.parent.parent.name == "20260101_100000"
