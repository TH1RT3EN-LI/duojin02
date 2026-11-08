from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/duojin02_slam_tools"))
from duojin02_slam_tools.offline import GeometryDropAudit, queue_drained


def status(**changes):
    result = dict(received=3, accepted=2, completed=2, queued=0, processing=False, invalid=0)
    result.update(changes)
    return result


def test_incremental_rejections_close_input_accounting_only_after_queue_drain(tmp_path):
    path = tmp_path / "session.log"
    path.write_text("[WARN] scan_geometry_drop raw_stamp=20 reason=scan awaits future TF support for base")
    audit = GeometryDropAudit(path, [10, 20, 30, 40])
    assert not audit.accounting(status(), 4)["input_accounting_closed"]
    with path.open("a") as stream:
        stream.write(" waited_sec=2.001\n")
    accounted = audit.accounting(status(), 4)
    assert accounted["geometry_rejected_count"] == 1
    assert accounted["geometry_rejection_reasons"] == {"20": "scan awaits future TF support for base"}
    assert queue_drained(status(), accounted)
    assert not queue_drained(status(queued=1), accounted)
    assert not queue_drained(status(processing=True), accounted)
    assert not queue_drained(status(completed=1), accounted)
    assert not queue_drained(status(invalid=1), accounted)


def test_repeated_log_lines_cannot_inflate_received_count(tmp_path):
    path = tmp_path / "session.log"
    line = "[WARN] scan_geometry_drop raw_stamp=20 reason=internal TF sample gap exceeds limit waited_sec=0.1\n"
    path.write_text(line + line)
    audit = GeometryDropAudit(path, [10, 20, 30, 40, 50])
    accounted = audit.accounting(status(), 5)
    assert accounted["geometry_rejected_count"] == 1
    assert accounted["duplicate_rejection_log_lines"] == 1
    assert not queue_drained(status(), accounted)
    assert audit.accounting(status(), 5) == accounted


@pytest.mark.parametrize("line,match", [
    ("scan_geometry_drop raw_stamp=99 reason=missing TF\n", "outside this bag"),
    ("scan_geometry_drop reason=pending_queue_limit raw_stamp=20\n", "queue overflowed"),
    ("scan_geometry_drop reason=no stamp\n", "Incomplete geometry"),
])
def test_unexpected_or_overflow_rejections_fail_replay(tmp_path, line, match):
    path = tmp_path / "session.log"
    path.write_text(line)
    with pytest.raises(RuntimeError, match=match):
        GeometryDropAudit(path, [10, 20]).accounting(status(received=1), 2)
