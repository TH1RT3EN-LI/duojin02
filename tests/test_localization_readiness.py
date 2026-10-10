"""Distinct scan readiness with realistic transform publication delay."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as S

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/duojin01_bringup'))
from duojin01_bringup.localization_readiness import ScanReadiness


def scan(t, frame='laser', ranges=None):
    ns=round(t*1e9)
    return S(header=S(stamp=S(sec=ns//10**9,nanosec=ns%10**9),frame_id=frame),
             angle_min=-1.,angle_max=1.,angle_increment=1.,range_min=.3,range_max=8.,
             scan_time=.1,time_increment=0.,ranges=[1.,2.,math.inf] if ranges is None else ranges)


def test_scan_before_tf_delay_does_not_erase_distinct_progress():
    state=ScanReadiness()
    for i in range(3):
        t=1.+.1*i
        state.receive(scan(t),round(t*1e9),t)
        assert not state.poll(round(t*1e9),t,lambda m:False)
        ready=state.poll(round((t+.02)*1e9),t+.02,lambda m:True)
        assert ready==(i==2)
    assert state.counted==3


def test_duplicates_old_scans_and_repeated_poll_never_advance_or_refresh():
    state=ScanReadiness();state.receive(scan(1.),10**9,1.)
    assert not state.poll(1_020_000_000,1.02,lambda m:True)
    for i in range(10):
        state.receive(scan(1.),1_100_000_000,1.1)
        state.receive(scan(.95),1_100_000_000,1.1)
        assert not state.poll(1_100_000_000,1.1,lambda m:True)
    assert state.counted==1
    assert not state.poll(1_290_000_000,1.31,lambda m:True)
    assert state.counted==0


def test_source_expiry_and_tf_never_available_reset_progress():
    state=ScanReadiness();state.receive(scan(1.),10**9,1.)
    assert not state.poll(1_020_000_000,1.02,lambda m:True)
    state.receive(scan(1.1),1_100_000_000,1.1)
    assert not state.poll(1_380_000_000,1.38,lambda m:False)
    assert not state.poll(1_410_000_000,1.41,lambda m:True)
    assert state.counted==0 and not state.pending


def test_wall_clock_watchdog_during_paused_ros_time():
    state=ScanReadiness();state.receive(scan(1.),10**9,1.)
    assert not state.poll(1_020_000_000,1.02,lambda m:True)
    assert not state.poll(1_020_000_000,1.31,lambda m:True)
    assert state.counted==0
    # Future-stamped callbacks cannot qualify multiple observations while ROS clock is frozen.
    for i in range(3):
        state.receive(scan(1.03+.05*i),1_020_000_000,1.32+.01*i)
        assert not state.poll(1_020_000_000,1.32+.01*i,lambda m:True)
    assert state.counted<=1


def test_clock_rewind_resets_and_preserves_scan_high_water():
    state=ScanReadiness();state.receive(scan(2.),2_000_000_000,1.)
    assert not state.poll(2_020_000_000,1.02,lambda m:True)
    assert not state.poll(1_000_000_000,1.03,lambda m:True)
    for t in [1.,1.1,1.2]:
        state.receive(scan(t),round(t*1e9),1.04)
        assert not state.poll(round(t*1e9),1.04,lambda m:True)
    assert state.counted==0
    for i in range(3):
        t=2.1+.1*i;state.receive(scan(t),round(t*1e9),1.1+.1*i)
        assert state.poll(round((t+.02)*1e9),1.12+.1*i,lambda m:True)==(i==2)


def test_malformed_scan_stamp_metadata_or_all_ranges_resets():
    invalids=[]
    m=scan(1.1);m.header.stamp.nanosec=10**9;invalids.append(m)
    m=scan(1.1);m.header.stamp.sec=-1;invalids.append(m)
    m=scan(1.1);m.angle_max=2.;invalids.append(m)
    m=scan(1.1);m.angle_increment=math.nan;invalids.append(m)
    invalids += [scan(1.1,frame=''),scan(1.1,ranges=[]),scan(1.1,ranges=[math.nan]*3),scan(1.1,ranges=[math.inf,-math.inf,1.]),scan(1.1,ranges=[.1]*3)]
    for malformed in invalids:
        state=ScanReadiness();state.receive(scan(1.),10**9,1.)
        state.poll(1_020_000_000,1.02,lambda m:True)
        state.receive(malformed,1_100_000_000,1.1)
        assert state.counted==0 and not state.pending


def test_queue_is_bounded_and_future_input_cannot_hold_deadline_open():
    state=ScanReadiness(max_pending=4)
    for i in range(100):
        t=1.+i*.001;state.receive(scan(t),round(t*1e9),t)
        assert not state.poll(round(t*1e9),t,lambda m:False)
        assert len(state.pending)<=4
    state.receive(scan(1000.),1_100_000_000,1.1)
    assert not state.pending and state.counted==0


def test_valid_positive_inf_and_partial_nan_can_qualify_scan_liveness():
    state=ScanReadiness()
    for i in range(3):
        t=1.+i*.1;state.receive(scan(t,ranges=[math.nan,math.inf,1.]),round(t*1e9),t)
        assert state.poll(round((t+.02)*1e9),t+.02,lambda m:True)==(i==2)


def test_zero_stamp_cannot_count_as_an_exact_time_observation():
    state=ScanReadiness();state.receive(scan(0.),50_000_000,1.)
    assert not state.poll(60_000_000,1.01,lambda m:True)
    assert not state.pending and state.counted==0
    for i in range(1,4):
        t=i*.1;state.receive(scan(t),round(t*1e9),1.+t)
        assert state.poll(round((t+.02)*1e9),1.+t+.02,lambda m:True)==(i==3)
