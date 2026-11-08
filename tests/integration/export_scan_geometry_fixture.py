#!/usr/bin/env python3
"""Export one real source scan and nearby original TF samples, without GT."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--raw-stamp-ns", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    db = sqlite3.connect(f"file:{args.database}?mode=ro", uri=True)
    topics = dict(db.execute("select id,name from topics"))
    selected, scan_hash, transforms, tf_hashes = None, None, [], []
    for topic_id, blob in db.execute("select topic_id,data from messages order by id"):
        topic = topics[topic_id]
        if topic == "/scan":
            scan = deserialize_message(blob, LaserScan)
            stamp = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
            if stamp == args.raw_stamp_ns:
                selected, scan_hash = scan, hashlib.sha256(blob).hexdigest()
        elif topic in ("/tf", "/tf_static"):
            used = False
            for tf in deserialize_message(blob, TFMessage).transforms:
                stamp = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
                if topic == "/tf" and not args.raw_stamp_ns-250_000_000 <= stamp <= args.raw_stamp_ns+300_000_000:
                    continue
                q = tf.transform.rotation
                yaw = math.atan2(2*(q.x*q.y+q.z*q.w), 1-2*(q.y*q.y+q.z*q.z))
                mirrored = 1-2*(q.x*q.x+q.y*q.y) < 0
                transforms.append((tf.header.frame_id, tf.child_frame_id, stamp,
                    tf.transform.translation.x, tf.transform.translation.y, yaw,
                    int(mirrored), int(topic == "/tf_static")))
                used = True
            if used:
                tf_hashes.append(hashlib.sha256(blob).hexdigest())
    if selected is None:
        raise RuntimeError("requested raw scan timestamp not found")
    s = selected
    lines = [f"SCAN {args.raw_stamp_ns} {s.header.frame_id} {s.angle_min:.17g} "
             f"{s.angle_increment:.17g} {s.time_increment:.17g} {s.range_min:.17g} "
             f"{s.range_max:.17g} {len(s.ranges)}"]
    lines += [f"R {float(value):.17g}" for value in s.ranges]
    # History enforces monotonicity per edge. Sorting preserves actual timestamps,
    # and removes exact repeated static messages without inventing samples.
    lines += ["TF " + " ".join(str(x) if isinstance(x, (int, str)) else f"{x:.17g}" for x in tf)
              for tf in sorted(set(transforms), key=lambda t: (t[0], t[1], t[2]))]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines)+"\n")
    args.output.with_suffix(".json").write_text(json.dumps({
        "schema": "duojin02.actual-scan-component-fixture.v1",
        "source_scan_id": f"{s.header.frame_id.lstrip('/')}:{args.raw_stamp_ns}",
        "database": str(args.database), "scan_cdr_sha256": scan_hash,
        "tf_cdr_sha256": tf_hashes,
        "fixture_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "finite_short_indices": [i for i, r in enumerate(s.ranges) if math.isfinite(r) and r < s.range_min],
        "scope": "original source rays and original TF support; no GT or fitted time offset",
        "clock_evidence": "uncalibrated", "beam_order_evidence": "original-index-increasing-time-assumed",
    }, indent=2)+"\n")


if __name__ == "__main__":
    main()
