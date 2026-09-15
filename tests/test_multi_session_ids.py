"""Regression for a real bug observed live: pressing REC three times in one
app run produced three sessions all sharing the SAME session_id, because
DatasetWriter.session_id was computed once at __init__ (the writer lives
for the whole app lifetime — continuous dataset root design) and
begin_session() never regenerated it. Every session-scoped raw filename
(recording_<id>.svo2, radar_<id>.bag, ...) collided with the first
session's, and the radar bag recorder correctly refused to overwrite it —
which is what actually surfaced the bug ("radar_<id>.bag exists already,
not overwriting").

begin_session() must produce a fresh, unique session_id on every call.
"""

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

from dataset.writer import DatasetWriter


def test_repeated_begin_session_produces_distinct_ids(tmp_path):
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=False)

    # Same app run, same DatasetWriter instance — exactly the observed
    # scenario (three REC presses without restarting the app).
    base_ns = 1_700_000_000_000_000_000
    writer.begin_session(base_ns)
    sid1 = writer.session_id
    writer.end_session(base_ns + 5_000_000_000)

    writer.begin_session(base_ns + 20_000_000_000)
    sid2 = writer.session_id
    writer.end_session(base_ns + 25_000_000_000)

    writer.begin_session(base_ns + 40_000_000_000)
    sid3 = writer.session_id
    writer.end_session(base_ns + 45_000_000_000)

    writer.close()

    assert len({sid1, sid2, sid3}) == 3, (
        f"sessions must have distinct IDs, got {sid1!r}, {sid2!r}, {sid3!r}"
    )


def test_begin_session_within_the_same_second_still_disambiguates(tmp_path):
    """Two REC presses landing in the same wall-clock second (rapid
    restart, or a fast test) must still get different IDs, not silently
    collide because the YYYYMMDD_HHMMSS format only has second resolution.
    """
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=False)
    same_second_ns = 1_700_000_000_000_000_000

    writer.begin_session(same_second_ns)
    sid1 = writer.session_id
    writer.end_session(same_second_ns + 1_000_000)

    writer.begin_session(same_second_ns + 2_000_000)  # +2ms, same second
    sid2 = writer.session_id
    writer.end_session(same_second_ns + 3_000_000)

    writer.close()

    assert sid1 != sid2
