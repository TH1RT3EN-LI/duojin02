#!/usr/bin/env python3
"""Independent native AMCL motion gate oracle; no truth or map input."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def wrap(v):
    return math.atan2(math.sin(v), math.cos(v))


def interpolate(a, b, f, deskew):
    w = wrap(b[2] - a[2])
    if not deskew:
        return (a[0] + f*(b[0]-a[0]), a[1] + f*(b[1]-a[1]), wrap(a[2]+f*w))
    def coeff(w):
        if abs(w) < 1e-5:
            w2=w*w
            return (1-w2/6+w2*w2/120, w/2-w*w2/24+w*w2*w2/720)
        return (math.sin(w)/w, (1-math.cos(w))/w)
    c,s=math.cos(a[2]), math.sin(a[2]); dx,dy=b[0]-a[0],b[1]-a[1]
    dx,dy=c*dx+s*dy,-s*dx+c*dy
    aa,bb=coeff(w); norm=aa*aa+bb*bb
    vx,vy=(aa*dx+bb*dy)/norm,(-bb*dx+aa*dy)/norm
    cc,dd=coeff(f*w); x,y=f*(cc*vx-dd*vy),f*(dd*vx+cc*vy)
    return (a[0]+c*x-s*y, a[1]+s*x+c*y, wrap(a[2]+f*w))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--replay-dir',type=Path,required=True)
    p.add_argument('--expected-only',action='store_true')
    a=p.parse_args()
    report=json.loads((a.replay_dir/'replay.json').read_text())
    sources=json.loads((a.replay_dir/'scan_sources.json').read_text())
    parameters=report['parameters']; edge=(parameters['odom_frame_id'],parameters['base_frame_id'])
    con=sqlite3.connect(f'file:{a.bag}?mode=ro',uri=True)
    samples={}
    for data, in con.execute("SELECT messages.data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name='/tf' ORDER BY messages.timestamp,messages.id"):
        for t in deserialize_message(bytes(data),TFMessage).transforms:
            if (t.header.frame_id.lstrip('/'),t.child_frame_id.lstrip('/')) != edge:
                continue
            q=t.transform.rotation
            assert abs(q.x)<1e-10 and abs(q.y)<1e-10, 'Nonplanar odometry'
            v=t.transform.translation; ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec
            samples[ns]=(v.x,v.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
    stamps=sorted(samples)
    expected=[]; last=None
    for scan in sources:
        if not scan['support_valid']:
            continue
        ns=scan['reference_stamp_ns']; idx=bisect.bisect_left(stamps,ns)
        if idx<len(stamps) and stamps[idx]==ns:
            pose=samples[ns]
        else:
            assert idx>0 and idx<len(stamps), f'Unavailable native odometry {ns}'
            l,r=stamps[idx-1],stamps[idx]
            pose=interpolate(samples[l],samples[r],(ns-l)/(r-l),parameters['scan_motion_compensation'])
        selected=last is None or abs(pose[0]-last[0])>parameters['update_min_d'] or abs(pose[1]-last[1])>parameters['update_min_d'] or abs(wrap(pose[2]-last[2]))>parameters['update_min_a']
        if selected:
            expected.append(scan['source_scan_id']); last=pose
    expected_path=a.replay_dir/'motion_selection_expected.json'
    expectation={'generated_wall_unix_s':time.time(),'source_scan_manifest_sha256':hashlib.sha256((a.replay_dir/'scan_sources.json').read_bytes()).hexdigest(),
        'oracle_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'expected_source_scan_ids':expected,'expected_pose_count':len(expected)}
    if expected_path.exists():
        frozen=json.loads(expected_path.read_text())
        assert frozen['expected_source_scan_ids']==expected, 'Predeclared motion expectation changed'
        assert frozen['source_scan_manifest_sha256']==expectation['source_scan_manifest_sha256'], 'Manifest changed after predeclaration'
    else:
        expected_path.write_text(json.dumps(expectation,indent=2)+'\n')
    if a.expected_only:
        print(json.dumps({k:expectation[k] for k in ['expected_pose_count','source_scan_manifest_sha256','oracle_source_sha256']})); return
    rows=json.loads((a.replay_dir/'poses.json').read_text()); actual=[r['source_scan_id'] for r in rows]
    result={'contract':'first pose and per-axis odometry motion threshold; D0 tf2 linear translation, D1 SE2 reference interpolation; support-invalid scans do not advance motion gate',
        'expected_pose_count':len(expected),'actual_pose_count':len(actual),'missing_source_scan_ids':sorted(set(expected)-set(actual)),
        'unexpected_source_scan_ids':sorted(set(actual)-set(expected)), 'exact_order_match':expected==actual}
    result['status']='passed' if result['exact_order_match'] else 'failed'
    (a.replay_dir/'motion_selection_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    assert result['status']=='passed', 'Native pose coverage did not match independent motion gate'

if __name__=='__main__': main()
