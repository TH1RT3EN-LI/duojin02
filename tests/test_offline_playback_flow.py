import pytest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/duojin02_slam_tools"))
from duojin02_slam_tools.offline import queue_drained
from duojin02_slam_tools.playback_flow import PlaybackFlow


def status(accepted=0, completed=0, processing=False, capacity=16, pending=0, **extra):
    return dict(schema=PlaybackFlow.SCHEMA, capacity=capacity, physical_pending=pending,
                received=accepted, accepted=accepted, completed=completed, queued=accepted-completed-int(processing),
                processing=processing, outstanding=accepted-completed, invalid=0,
                clock_active=True, clock_nanoseconds=1000000000, **extra)


def test_hysteresis_preserves_a_single_pause_interval():
    flow = PlaybackFlow()
    assert flow.request(status(3)) is None
    assert flow.request(status(4)) == "pause_high_watermark"
    assert flow.acknowledged("pause_high_watermark", status(5), 10) is None
    assert flow.request(status(5, 3)) is None
    assert flow.request(status(5, 4)) == "resume_low_watermark"
    assert flow.acknowledged("resume_low_watermark", status(5, 4), 13) is None
    report = flow.report(20)
    assert report["pause_count"] == 1 and report["paused_wall_sec"] == 3
    assert report["peak_pause_overshoot"] == 1
    assert report["high_watermark"] == 4 and report["low_watermark"] == 1


def test_future_dependency_is_not_held_behind_a_paused_producer():
    flow = PlaybackFlow()
    assert flow.request(status(4, pending=1)) is None
    assert flow.request(status(4)) == "pause_high_watermark"
    # A message already in middleware can open a future-TF dependency after ACK.
    assert flow.acknowledged("pause_high_watermark", status(5, pending=1), 10) == "resume_dependencies"
    assert flow.acknowledged("resume_dependencies", status(5, pending=1), 10.1) is None
    assert not flow.paused and flow.report(11)["dependency_reopens"] == 1


@pytest.mark.parametrize("change", [
    {"schema": "legacy"}, {"capacity": 1}, {"clock_active": False},
    {"clock_nanoseconds": -1}, {"outstanding": 9}, {"queued": 1},
    {"accepted": True}, {"invalid": 1}, {"received": -1},
    {"accepted": 1, "received": 0, "outstanding": 1, "queued": 1},
])
def test_unverifiable_or_rejected_input_fails_explicitly(change):
    current = status()
    current.update(change)
    with pytest.raises(RuntimeError):
        PlaybackFlow().request(current)


def test_headroom_exhaustion_is_not_silently_a_larger_queue():
    with pytest.raises(RuntimeError, match="headroom"):
        PlaybackFlow().request(status(16))


def test_reset_or_capacity_change_cannot_hide_in_the_same_replay():
    flow = PlaybackFlow()
    flow.request(status(3, 1))
    with pytest.raises(RuntimeError, match="backwards"):
        flow.request(status(2, 1))
    with pytest.raises(RuntimeError, match="capacity changed"):
        flow.request(status(3, 1, capacity=32))


def test_pending_geometry_blocks_terminal_drain_even_with_empty_sync_queue():
    assert not queue_drained(status(2, 2, pending=1), {"input_accounting_closed": True})
    assert queue_drained(status(2, 2), {"input_accounting_closed": True})


def test_eof_closes_an_acknowledged_pause_without_counting_save_time():
    flow = PlaybackFlow()
    flow.acknowledged("pause_high_watermark", status(4), 10)
    flow.finished(11)
    first = flow.report(12)
    assert first == flow.report(20)
    assert first["paused_wall_sec"] == 1 and first["paused_at_eof"] is True
    assert first["paused_at_report"] is False
    flow.inspect(status(4, 4))  # Complete native drain still needs verification.
    with pytest.raises(RuntimeError, match="after EOF"):
        flow.request(status(4, 4))
