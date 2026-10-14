"""Evidence, temporal recovery and navigation lease safety without ROS runtime."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/duojin01_bringup'))
from duojin01_bringup.localization_health import HealthPolicy, HealthState, MapEvidence, HealthSignal


def rings(inner=False, unknown=False):
    size, resolution = 500, 0.01
    data = []
    for y in range(size):
        for x in range(size):
            r = math.hypot(-2.5+x*resolution, -2.5+y*resolution)
            data.append(-1 if unknown else 100 if abs(r-2.0)<0.015 or inner and abs(r-.37)<.005 else 0)
    return MapEvidence(size,size,resolution,(-2.5,-2.5,0),data)


def score(grid, ranges, pose=(0,0,0), increment=2*math.pi/90, minimum=.3):
    return grid.score(ranges,-math.pi,increment,minimum,8.,pose,HealthPolicy(max_ray_cells=100000))


def test_correct_match_and_wrong_low_uncertainty_pose():
    grid=rings()
    assert score(grid,[2.]*90)['valid']
    assert not score(grid,[2.]*90,(1.0,0,0))['valid']


def test_unknown_no_return_and_observed_thin_wall_have_distinct_evidence():
    grid=rings()
    v=score(grid,[2.]*45+[math.inf]*45)
    # Historical Inf-through-wall rejection assumed a free-space certificate.
    # The N10 empty-bin publisher does not provide one; only the 45 returns count.
    assert v['valid'] and v['compared']==45 and v['unobserved']==45
    assert v['contradictions']==0 and v['contradiction_fraction']==0.
    v=score(rings(inner=True),[2.]*90)
    assert not v['valid'] and v['contradiction_fraction']>.9


def test_unobserved_unknown_and_invalid_unsampled_ray_cannot_support_pose():
    assert not score(rings(),[math.inf]*90)['valid']
    assert not score(rings(unknown=True),[2.]*90)['valid']
    grid=rings();scaled=[100 if v==100 else 50 for v in grid.data]
    assert not score(MapEvidence(500,500,.01,(-2.5,-2.5,0),scaled),[2.]*90)['valid']
    values=[2.]*540;values[1]=-math.inf
    assert not score(rings(),values,increment=2*math.pi/540)['valid']
    values[1]=None
    assert not score(rings(),values,increment=2*math.pi/540)['valid']
    values[1]=.1
    assert not score(rings(),values,increment=2*math.pi/540)['valid']
    assert not score(rings(),[2.]*90,increment=0)['valid']
    assert not score(rings(),[2.]*90,minimum=-.1)['valid']


def test_map_convention_and_rotated_amcl_are_explicit():
    near=MapEvidence(3,3,.02,(0,0,0),[0]*9)
    normal=MapEvidence(3,3,.02,(0,0,0),[0]*9,'occupancy_cell')
    assert near.cell(-.1,0)==0 and normal.cell(-.1,0)==-1
    import pytest
    with pytest.raises(ValueError,match='ignores rotated'):
        MapEvidence(1,1,.02,(0,0,.1),[100])


def blind_ring(origin_occupied=False, unknown_origin=False):
    grid = rings()
    for iy in range(grid.height):
        for ix in range(grid.width):
            radius = math.hypot(-2.5 + ix*.01, -2.5 + iy*.01)
            if radius < .15:
                grid.data[iy*grid.width+ix] = (100 if origin_occupied else
                    -1 if unknown_origin and radius < .08 else 100 if radius >= .08 else 0)
    return grid


def test_blind_inf_has_no_positive_support_and_finite_cannot_see_through_near_wall():
    grid = blind_ring()
    v = score(grid, [math.inf]*90)
    assert not v['valid'] and v['hits'] == 0 and v['unobserved'] == 90
    assert v['compared'] == 0
    for rays in ([2.]*90, [2.]*45 + [math.inf]*45):
        v = score(grid, rays)
        assert not v['valid'] and v['contradiction_fraction'] == 1.0
        assert v['unobserved'] == sum(r == math.inf for r in rays)


def test_occupied_origin_and_unknown_before_near_wall_cannot_support_wrong_pose():
    for rays in ([2.]*90, [2.]*45 + [math.inf]*45):
        v = score(blind_ring(origin_occupied=True), rays)
        assert not v['valid'] and v['reason'] == 'laser_origin_occupied'
        v = score(blind_ring(unknown_origin=True), rays)
        assert not v['valid'] and v['contradiction_fraction'] == 1.0


def test_excused_inf_cannot_dilute_contradictions_from_observed_beams():
    grid = rings()
    # 45 matched finite beams, 12 through a near blocker. 44 other directions
    # are censored Inf behind near blockers; the last Inf crosses the far ring.
    for i in list(range(12)) + list(range(45, 89)):
        angle = -math.pi + i*2*math.pi/90
        gx, gy = grid.grid_point(.28*math.cos(angle), .28*math.sin(angle))
        ix, iy = math.floor(gx+.5), math.floor(gy+.5)
        grid.data[iy*grid.width+ix] = 100
    v = score(grid, [2.]*45 + [math.inf]*45)
    assert v['hits'] == 45 and v['hit_fraction'] == 1.0 and v['known_fraction'] == 1.0
    assert v['unobserved'] == 45 and v['compared'] == 45
    assert math.isclose(v['contradiction_fraction'], 12/45)
    assert not v['valid']  # Adding unknown Inf must not dilute finite contradictions.


def test_unknown_sensor_values_and_policy_clipped_finite_returns_are_distinct():
    grid = rings()
    v = score(grid,[2.]*45+[9.]*45)  # 9 exceeds this scan's published maximum 8.
    assert v['valid'] and v['hits']==45 and v['unobserved']==45 and v['compared']==45
    assert v['contradictions']==0
    for rays in ([math.inf]*90,[9.]*90):
        v = score(grid,rays)
        assert not v['valid'] and v['compared']==0 and v['ray_work']==0
        assert v['hits']==0 and v['contradictions']==0
    # The same finite 9 m measurement is valid if the sensor declares 10 m.
    # Policy clipping to 8 m cannot erase its ray through the known 2 m wall.
    v = grid.score([2.]*45+[9.]*45,-math.pi,2*math.pi/90,.3,10.,(0,0,0),
                   HealthPolicy(max_ray_cells=100000))
    assert not v['valid'] and v['hits']==45 and v['unobserved']==0
    assert v['policy_clipped']==45 and v['compared']==90 and v['contradiction_fraction']>.4
    # A finite reading equal to the sensor maximum is still sensor-valid here.
    v = grid.score([2.]*45+[10.]*45,-math.pi,2*math.pi/90,.3,10.,(0,0,0),
                   HealthPolicy(max_ray_cells=100000))
    assert not v['valid'] and v['policy_clipped']==45 and v['unobserved']==0
    v = blind_ring().score([9.]*90,-math.pi,2*math.pi/90,.3,100.,(0,0,0),
                         HealthPolicy(max_ray_cells=100000))
    assert not v['valid'] and v['policy_clipped']==90 and v['contradiction_fraction']==1.


def test_unknown_beams_do_not_refill_uniform_subset_or_support_wrong_pose():
    grid = rings()
    rays = [2.]*540
    for i in range(0,540,6): rays[i]=math.inf
    v = score(grid,rays,increment=2*math.pi/540)
    assert not v['valid'] and v['hits']==0 and v['compared']==0
    for pose in ((1.,0,0),(0,1.,0),(.5,.5,0)):
        v = score(grid,[2.]*45+[math.inf]*45,pose)
        assert not v['valid']
    assert not score(grid,[2.]*44+[math.inf]*46)['valid']


def test_self_masked_nan_is_unknown_and_cannot_refill_or_dilute_evidence():
    grid = rings()
    v = score(grid, [2.]*45 + [math.nan]*45)
    assert v['valid'] and v['hits'] == 45 and v['compared'] == 45
    assert v['unobserved'] == 45 and v['invalid'] == 0 and v['contradictions'] == 0
    v = score(grid, [math.nan]*90)
    assert not v['valid'] and v['hits'] == 0 and v['compared'] == 0
    assert v['unobserved'] == 90 and v['ray_work'] == 0
    assert not score(grid, [2.]*44 + [math.nan]*46)['valid']
    # Keep the requested angular roster. Nearby unmasked rays must not replace
    # masked sampled directions, and an unsampled NaN adds no positive support.
    rays = [2.]*540
    rays[1] = math.nan
    v = score(grid, rays, increment=2*math.pi/540)
    assert v['valid'] and v['hits'] == 90 and v['compared'] == 90
    for i in range(0, 540, 6):
        rays[i] = math.nan
    v = score(grid, rays, increment=2*math.pi/540)
    assert not v['valid'] and v['hits'] == 0 and v['compared'] == 0
    assert v['unobserved'] == 90 and v['ray_work'] == 0
    v = score(blind_ring(), [2.]*45 + [math.nan]*45)
    assert not v['valid'] and v['contradictions'] == 45
    assert v['compared'] == 45 and v['contradiction_fraction'] == 1.


def test_sensor_floor_preserves_near_external_endpoints():
    resolution, size = .01, 100
    data = [100 if abs(math.hypot(-.5+x*resolution, -.5+y*resolution)-.25) < .015
            else 0 for y in range(size) for x in range(size)]
    grid = MapEvidence(size, size, resolution, (-.5, -.5, 0), data)
    v = score(grid, [.25]*90, minimum=.02)
    assert v['valid'] and v['hits'] == 90 and v['matched'] == 90
    assert not score(grid, [.25]*90, minimum=.3)['valid']


def test_blind_blocker_requires_positive_cell_interior_not_boundary_contact():
    for convention, origin in [('occupancy_cell', (0,0,0)), ('amcl_nearest', (.5,.5,0))]:
        grid = MapEvidence(5,2,1,origin,[100,0,100,0,0]+[0]*5,convention)
        values = list(grid.ray_cells(2,.5,-1,0,0,.25,interior=True))
        assert 100 not in values and 0 in values
        # Along an occupied cell edge does not enter its opaque interior.
        grid = MapEvidence(5,2,1,origin,[0]*5+[0,100,0,0,0],convention)
        assert 100 not in grid.ray_cells(.1,1,1,0,0,1.5,interior=True)
        # Cell touched only at a corner is not a blind occluder.
        grid = MapEvidence(3,3,1,origin,[0,100,0,100,0,0,0,0,0],convention)
        assert 100 not in grid.ray_cells(.5,.5,1,1,0,1,interior=True)


def test_blind_blocker_boundary_unknown_and_rotated_cell_convention():
    for convention, origin in [('occupancy_cell',(0,0,0)), ('amcl_nearest',(.5,.5,0))]:
        grid = MapEvidence(5,1,1,origin,[0,100,0,0,0],convention)
        assert 100 not in grid.ray_cells(0,.5,1,0,0,1,interior=True)
        assert 100 in grid.ray_cells(0,.5,1,0,0,1.01,interior=True)
        unknown = MapEvidence(5,1,1,origin,[-1]*5,convention)
        assert 100 not in unknown.ray_cells(0,.5,1,0,0,.3,interior=True)
        assert 100 not in grid.ray_cells(-2,.5,1,0,0,.3,interior=True)
        assert list(grid.ray_cells(0,.5,1,0,0,0,interior=True)) == []
    # The explicit occupancy-cell convention rotates the whole ray and map.
    rotated = MapEvidence(5,1,1,(2,3,math.pi/2),[0,100,0,0,0],'occupancy_cell')
    assert 100 not in rotated.ray_cells(1.5,3,0,1,0,1,interior=True)
    assert 100 in rotated.ray_cells(1.5,3,0,1,0,1.01,interior=True)


def test_new_blind_prefix_work_is_bounded_and_cannot_count_as_known_support():
    grid = rings()
    v = grid.score([2.]*90,-math.pi,2*math.pi/90,.3,8.,(0,0,0),HealthPolicy(max_ray_cells=5))
    assert not v['valid'] and v['reason'] == 'map_ray_work_budget_exceeded'
    # A nearer-than-minimum wall explains absence, never the positive support
    # needed to satisfy 45 observed hits or known finite endpoints/free rays.
    assert not score(blind_ring(unknown_origin=True),[math.inf]*90)['valid']
    assert not score(blind_ring(),[2.]*44+[math.inf]*46)['valid']


def test_nonprogressing_dda_cannot_bypass_work_budget():
    # At this legal but pathological resolution, tx+dtx rounds back to tx.
    # Near occupied evidence must not leave an uncounted infinite rear traversal.
    grid = MapEvidence(3,1,1e-18,(0,0,0),[0,100,0])
    v = grid.score([2.]*90,0,2*math.pi/90,.3,8.,(0,0,0),HealthPolicy(max_ray_cells=5))
    assert not v['valid'] and v['reason'] == 'map_ray_work_budget_exceeded'


def test_finite_near_endpoint_keeps_existing_ray_margin():
    grid = rings()
    for i in range(15):
        angle = -math.pi+i*2*math.pi/90
        gx,gy = grid.grid_point(.31*math.cos(angle),.31*math.sin(angle))
        grid.data[math.floor(gy+.5)*grid.width+math.floor(gx+.5)] = 100
    v = score(grid,[.31]*15+[2.]*75)
    assert v['valid'] and v['contradiction_fraction'] == 0.


def test_n10_540_bin_narrow_corridor_uses_only_observable_evidence():
    # Independent analytic box intersections; no ROS, Gazebo or truth feedback.
    size_x, size_y, res = 621, 521, .01
    origin = (-3.1,-2.6,0)
    boxes = [(-3.1,-3.,-3.,3.),(3.,3.1,-3.,3.),(-3.,3.,-2.6,-2.5),
             (-3.,3.,2.5,2.6),(-.3,.3,.25,2.5),(-.3,.3,-2.5,-.25)]
    data = []
    for iy in range(size_y):
        for ix in range(size_x):
            x, y = origin[0]+ix*res, origin[1]+iy*res
            data.append(100 if any(x0<=x<=x1 and y0<=y<=y1 for x0,x1,y0,y1 in boxes) else 0)
    grid = MapEvidence(size_x,size_y,res,origin,data)
    pose = (.0924721442817,-8.97794961929e-6,0)
    values = []
    for i in range(540):
        a = -math.pi+i*2*math.pi/540; dx, dy = math.cos(a), math.sin(a)
        distances = []
        for x0,x1,y0,y1 in boxes:
            low, high = 0., math.inf
            for p,v,lo,hi in ((pose[0],dx,x0,x1),(pose[1],dy,y0,y1)):
                if abs(v)<1e-15:
                    if not lo<=p<=hi: high=-1
                else:
                    near,far=sorted(((lo-p)/v,(hi-p)/v));low=max(low,near);high=min(high,far)
            if low<=high and high>=0: distances.append(low)
        r = min(distances)
        values.append(r if r>=.3 else math.inf)
    v = score(grid,values,pose,increment=2*math.pi/540)
    assert v['valid'] and v['hits'] >= 45 and v['unobserved'] >= 20
    assert v['hit_fraction'] == 1.0 and v['known_fraction'] == 1.0
    assert v['compared'] == 90-v['unobserved']
    for wrong_y in (-.5,.5):
        v = score(grid,values,(pose[0],wrong_y,0),increment=2*math.pi/540)
        assert not v['valid'] and v['reason'] == 'laser_origin_occupied'


def test_unique_scans_fail_closed_latch_and_explicit_recovery():
    state=HealthState();good={'valid':True};bad={'valid':False,'reason':'mismatch'}
    for _ in range(20):state.observe(1.,1.,good)
    assert not state.healthy and len(state.good)==1
    for i in range(1,6):state.observe(1.+i*.1,1.+i*.1,good)
    assert state.healthy
    state.observe(1.6,1.6,bad)
    assert not state.healthy
    state.observe(1.7,1.7,bad);state.observe(1.8,1.8,bad)
    assert state.state=='MISMATCH'
    for i in range(20):state.observe(2.+i*.1,2.+i*.1,good)
    assert not state.healthy
    state.start_recovery()
    for i in range(15):state.observe(4.+i*.1,4.+i*.1,good,4.)
    assert not state.healthy  # Three loops over one pose are not three observations.
    state.observe(5.5,5.5,good,5.4);state.observe(5.6,5.6,good,5.5)
    assert state.healthy
    state.tick(6.1)
    assert not state.healthy


def test_startup_initial_error_can_converge_but_interrupted_windows_do_not_accumulate():
    state=HealthState()
    for i in range(10):state.observe(i*.1,i*.1,{'valid':False})
    assert not state.healthy and state.state!='MISMATCH'
    for i in range(6):state.observe(1.+i*.1,1.+i*.1,{'valid':True})
    assert state.healthy
    state.recovery_failed('service_failed')
    state.tick(20.)
    assert state.state=='RECOVERY_FAILED' and state.reason=='service_failed'
    state=HealthState()
    for i in range(5):state.observe(i*.1,i*.1,{'valid':True})
    state.observe(1.0,1.0,{'valid':True})
    assert not state.healthy and len(state.good)==1


def status(seq=1,session='a',emitted=10.,scan=9.95,healthy=True):
    return dict(sequence=seq,session=session,emitted_ros_ns=round(emitted*1e9),scan_stamp=scan,
                healthy=healthy,state='HEALTHY' if healthy else 'MISMATCH')


def test_retained_true_old_source_stamp_replay_and_restart_cannot_open_gate():
    signal=HealthSignal()
    assert not signal.accept(status(),10.,0.)
    assert not signal.accept(status(),10.05,.05)
    assert signal.accept(status(2,emitted=10.05,scan=10.),10.05,.05)
    assert not signal.accept(status(3,emitted=10.05),11.,1.)
    assert not signal.accept(status(1,'b',11.,10.95),11.,1.)
    assert not signal.accept(status(4,'a',11.,10.95),11.,1.05)
    assert signal.accept(status(2,'b',11.05,11.),11.05,1.05)


def healthy_state():
    state = HealthState()
    for i in range(6):
        state.observe(10.0 + i*.1, i*.1, {'valid': True})
    assert state.healthy
    return state


def test_pure_dropout_requalifies_only_after_a_complete_new_good_window():
    state = healthy_state()
    state.tick(1.0)
    assert state.state == 'PERCEPTION_STALE' and not state.healthy
    assert not state.good
    for i in range(20):
        assert not state.observe(10.5, 1.0 + i*.01, {'valid': True})
    assert state.last_wall == .5 and not state.good
    for i in range(5):
        state.observe(11.0 + i*.1, 1.5 + i*.1, {'valid': True})
    assert not state.healthy
    state.observe(11.5, 2.0, {'valid': True})
    assert state.healthy and state.state == 'HEALTHY'


def test_gap_detected_by_next_scan_also_rebuilds_permission():
    state = healthy_state()
    state.observe(11.0, 1.5, {'valid': True})  # No watchdog callback between scans.
    assert state.state == 'PERCEPTION_STALE' and not state.healthy
    assert state.good == [11.0]
    for i in range(1, 6):
        state.observe(11.0 + i*.1, 1.5 + i*.1, {'valid': True})
    assert state.healthy


def test_repeated_outages_cannot_combine_partial_confirmation_windows():
    state = healthy_state()
    state.tick(1.0)
    for i in range(5):
        state.observe(11.0 + i*.1, 1.1 + i*.1, {'valid': True})
    assert not state.healthy
    state.tick(2.0)
    assert not state.good
    for i in range(5):
        state.observe(12.0 + i*.1, 2.1 + i*.1, {'valid': True})
    assert not state.healthy
    state.observe(12.5, 2.6, {'valid': True})
    assert state.healthy


def test_startup_gap_does_not_create_a_global_recovery_requirement():
    state = HealthState()
    for i in range(5):
        state.observe(10.0 + i*.1, i*.1, {'valid': True})
    state.tick(1.0)
    assert state.state == 'WAITING' and not state.good
    for i in range(6):
        state.observe(11.0 + i*.1, 1.1 + i*.1, {'valid': True})
    assert state.healthy


def test_paused_sensor_clock_stops_and_cannot_renew_or_count_repeated_scans():
    state = healthy_state()
    for i in range(1, 11):
        wall = .5 + i*.1
        state.observe(10.5, wall, {'valid': True})
        state.tick(wall)
    assert state.state == 'PERCEPTION_STALE' and not state.healthy
    assert state.last_stamp == 10.5 and state.last_wall == .5 and not state.good
    for i in range(6):
        state.observe(11.0 + i*.1, 1.6 + i*.1, {'valid': True})
    assert state.healthy


def test_clock_rewind_is_a_hard_fault_and_cannot_become_pure_stale():
    state = healthy_state()
    assert not state.observe(10.4, .6, {'valid': True})
    assert state.state == 'MISMATCH' and state.reason == 'sensor_stamp_rewind'
    state.tick(2.0)
    for i in range(20):
        state.observe(11.0 + i*.1, 2.1 + i*.1, {'valid': True})
    assert not state.healthy and state.state == 'MISMATCH'
    assert state.reason == 'sensor_stamp_rewind'


def test_existing_hard_faults_keep_their_state_and_cause_across_gaps():
    for failed in (False, True):
        state = healthy_state()
        if failed:
            state.recovery_failed('service_failed')
        else:
            state.invalidate('map_changed', latch=True)
        expected_state, expected_reason = state.state, state.reason
        state.tick(2.0)
        for i in range(20):
            state.observe(11.0 + i*.1, 2.1 + i*.1, {'valid': True})
        assert not state.healthy and state.state == expected_state
        assert state.reason == expected_reason
        state.invalidate('invalid_scan_geometry', latch=True)
        assert state.state == expected_state


def test_malformed_observation_during_requalification_requires_explicit_recovery():
    for reason in ('invalid_scan_geometry', 'invalid_scan_ray'):
        state = healthy_state()
        state.tick(1.0)
        state.observe(11.0, 1.1, {'valid': True})
        state.observe(11.1, 1.2, {'valid': False, 'reason': reason})
        assert state.state == 'MISMATCH' and not state.good
        for i in range(20):
            state.observe(11.2 + i*.1, 1.3 + i*.1, {'valid': True})
        assert not state.healthy and state.reason == reason


def test_nonfinite_sensor_stamp_is_hard_fault_without_advancing_high_water():
    state = healthy_state()
    assert not state.observe(math.nan, .6, {'valid': True})
    assert state.state == 'MISMATCH' and not state.healthy
    assert state.last_stamp == 10.5


def test_recovery_gap_retains_recovery_and_rebuilds_long_confirmation():
    state = healthy_state()
    state.start_recovery()
    for i in range(10):
        state.observe(11.0 + i*.1, 1.0 + i*.1, {'valid': True}, 11.0 + i*.1)
    assert not state.healthy
    state.tick(2.5)
    assert state.state == 'RECOVERING'
    assert not state.good and not state.recovery_observations
    for i in range(14):
        state.observe(12.0 + i*.1, 2.6 + i*.1, {'valid': True}, 12.0 + i*.1)
    assert not state.healthy
    state.observe(13.4, 4.0, {'valid': True}, 13.4)
    assert state.healthy


def test_real_mismatch_after_dropout_does_not_inherit_old_good_or_bad_windows():
    state = healthy_state()
    state.observe(10.6, .6, {'valid': False, 'reason': 'mismatch'})
    state.tick(1.1)
    assert state.state == 'PERCEPTION_STALE' and not state.bad
    for i in range(2):
        state.observe(11.0 + i*.1, 1.2 + i*.1, {'valid': False, 'reason': 'mismatch'})
    assert state.state == 'SUSPECT'
    state.observe(11.2, 1.4, {'valid': False, 'reason': 'mismatch'})
    assert state.state == 'MISMATCH' and not state.healthy


def test_high_rate_good_and_bad_windows_require_count_and_full_continuous_span():
    state = HealthState()
    for i in range(10):
        state.observe(20.0+i*.05, i*.05, {'valid': True})
        assert not state.healthy
    state.observe(20.5, .5, {'valid': True})
    assert state.healthy
    for i in range(4):
        state.observe(20.55+i*.05, .55+i*.05, {'valid': False, 'reason': 'mismatch'})
        assert state.state == 'SUSPECT'
    state.observe(20.75, .75, {'valid': False, 'reason': 'mismatch'})
    assert state.state == 'MISMATCH'


def test_high_rate_recovery_uses_full_continuous_span_with_bounded_storage():
    state = healthy_state()
    state.start_recovery()
    for i in range(28):
        state.observe(20.0+i*.05, 1.0+i*.05, {'valid': True}, 20.0+i*.05)
        assert not state.healthy
    state.observe(21.4, 2.4, {'valid': True}, 21.4)
    assert state.healthy and len(state.good) == state.policy.recovery_scans
    assert state.good_started == 20.


def test_jitter_and_duplicate_scans_cannot_shorten_the_new_qualification_span():
    state = healthy_state()
    state.tick(1.0)
    offsets = [0., .02, .09, .11, .17, .23, .28, .31, .38, .47]
    for dt in offsets:
        state.observe(20.0+dt, 1.1+dt, {'valid': True})
        assert not state.healthy
        assert not state.observe(20.0+dt, 1.11+dt, {'valid': True})
    state.observe(20.5, 1.6, {'valid': True})
    assert state.healthy


def test_opposite_evidence_and_explicit_reset_clear_continuous_start_stamps():
    state = healthy_state()
    state.observe(10.6, .6, {'valid': False, 'reason': 'mismatch'})
    assert state.good_started is None and state.bad_started == 10.6
    state.observe(10.7, .7, {'valid': True})
    assert state.bad_started is None and state.good_started == 10.7
    for i in range(8):
        state.observe(10.75+i*.05, .75+i*.05, {'valid': True})
        assert not state.healthy
    state.observe(11.2, 1.2, {'valid': True})
    assert state.healthy
    state.start_recovery()
    assert state.good_started is None and state.bad_started is None
    state.observe(11.3, 1.3, {'valid': True}, 11.3)
    state.invalidate('map_changed', latch=True)
    assert state.good_started is None and state.bad_started is None
    state.recovery_failed('failed')
    assert state.good_started is None and state.bad_started is None


def monitor_fixture():
    """Exercise real monitor methods without creating nodes or a ROS graph.

    The container test environment has ROS imports. The host's independent
    state suite can still run without them; these method checks skip there.
    """
    import pytest
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from tf2_ros import TransformException
    from duojin01_bringup.localization_manager import LocalizationManager

    def missing_tf(*_args):
        raise TransformException('Missing exact-stamp transform')

    state = healthy_state()
    monitor = SimpleNamespace(state=state, policy=state.policy, pending=None,
                              score_due=True,
                              pending_tf_wait=False, received=0,
                              map_frame='map', recovery_phase=None,
                              buffer=SimpleNamespace(lookup_transform=missing_tf))
    increment = 2*math.pi/90
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='laser',
        stamp=SimpleNamespace(sec=11, nanosec=0)), ranges=[2.]*90,
        angle_min=-math.pi, angle_increment=increment,
        angle_max=-math.pi+89*increment, range_min=.3, range_max=8.,
        scan_time=.1, time_increment=0.)
    return LocalizationManager, monitor, msg, SimpleNamespace(nanoseconds=11_000_000_000)


def test_exact_tf_wait_duplicate_and_natural_aging_requalifies_as_stale():
    methods, monitor, msg, stamp = monitor_fixture()
    methods.on_scan(monitor, msg)
    methods.process_pending(monitor, msg, stamp, .1, .7)
    assert monitor.pending_tf_wait
    methods.on_scan(monitor, msg)
    assert monitor.pending_tf_wait
    methods.process_pending(monitor, msg, stamp, .5, 1.1)
    assert monitor.pending is None
    assert monitor.state.state == 'PERCEPTION_STALE' and not monitor.state.healthy
    assert monitor.state.last_wall == .5


def test_already_stale_scan_arrival_is_a_hard_fault():
    methods, monitor, msg, stamp = monitor_fixture()
    methods.on_scan(monitor, msg)
    methods.process_pending(monitor, msg, stamp, .5, 1.1)
    assert monitor.state.state == 'MISMATCH' and not monitor.state.healthy


def test_malformed_duplicate_does_not_inherit_pure_tf_timeout_permission():
    methods, monitor, msg, stamp = monitor_fixture()
    methods.on_scan(monitor, msg)
    methods.process_pending(monitor, msg, stamp, .1, .7)
    msg.angle_increment = 0.
    methods.on_scan(monitor, msg)
    methods.process_pending(monitor, msg, stamp, .5, 1.1)
    assert monitor.state.state == 'MISMATCH' and not monitor.state.healthy


def test_partial_self_masked_nan_waits_for_exact_tf_without_latching_fault():
    methods, monitor, msg, stamp = monitor_fixture()
    msg.ranges[7] = math.nan
    methods.on_scan(monitor, msg)
    methods.process_pending(monitor, msg, stamp, .1, .7)
    assert monitor.pending_tf_wait and monitor.state.state != 'MISMATCH'
    # The same missing exact transform still expires at the original deadline.
    methods.process_pending(monitor, msg, stamp, .5, 1.1)
    assert monitor.pending is None and monitor.state.state == 'PERCEPTION_STALE'


def test_partial_self_masking_does_not_excuse_malformed_unsampled_ray():
    methods, monitor, msg, stamp = monitor_fixture()
    msg.ranges[7], msg.ranges[8] = math.nan, -math.inf
    methods.on_scan(monitor, msg)
    methods.process_pending(monitor, msg, stamp, .1, .7)
    assert monitor.state.state == 'MISMATCH' and not monitor.state.healthy


def test_recovery_confirmation_at_deadline_is_accepted_but_later_is_closed():
    for wall, expected_failure in ((8., False), (8.000001, True)):
        methods, monitor, _msg, _stamp = monitor_fixture()
        monitor.recovery_phase, monitor.deadline = 'VERIFY_SCANS', 8.
        monitor.state.start_recovery()
        for i in range(15):
            monitor.state.observe(11.0+i*.1, 6.6+i*.1, {'valid': True}, 11.0+i*.1)
        assert monitor.state.healthy
        failures = []

        def fail(reason):
            failures.append(reason)
            monitor.state.recovery_failed(reason)
            monitor.recovery_phase = None

        from types import SimpleNamespace
        monitor.fail_attempt = fail
        monitor.get_logger = lambda: SimpleNamespace(info=lambda _: None)
        methods.recovery_tick(monitor, wall)
        assert bool(failures) == expected_failure
        assert monitor.state.healthy is not expected_failure


def test_monitor_tick_checks_the_deadline_after_scoring_finishes(monkeypatch):
    methods, monitor, msg, stamp = monitor_fixture()
    from types import SimpleNamespace
    from duojin01_bringup import localization_manager as module
    monitor.grid, monitor.pending = object(), msg
    monitor.recovery_phase, monitor.deadline = 'VERIFY_SCANS', 8.
    # Avoid clocks/ROS initialization while preserving tick's actual sequencing.
    monkeypatch.setattr(module.Time, 'from_msg', lambda _: stamp)
    monitor.get_clock = lambda: SimpleNamespace(now=lambda: FakeClockNow())

    class FakeClockNow:
        def __sub__(self, _stamp):
            return SimpleNamespace(nanoseconds=0)

    def finish_score(*_args):
        monitor.state.healthy = True

    failures = []
    monitor.process_pending = finish_score
    monitor.fail_attempt = lambda reason: (failures.append(reason), monitor.state.recovery_failed(reason))
    monitor.recovery_tick = lambda wall: methods.recovery_tick(monitor, wall)
    published = []
    monitor.publish = lambda: published.append(monitor.state.healthy)
    monitor.forward_command = lambda: None
    times = iter((7.95, 8.05))
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(times))
    methods.tick(monitor)
    assert failures == ['bounded_global_recovery_timeout:VERIFY_SCANS']
    assert published == [False]


def recovery_monitor_fixture():
    methods, monitor, msg, _stamp = monitor_fixture()
    from types import SimpleNamespace as S
    monitor.state = HealthState()
    monitor.state.start_recovery()
    monitor.policy = monitor.state.policy
    monitor.recovery_phase, monitor.deadline = 'VERIFY_SCANS', 15.
    monitor.recovery_sensor_cutoff, monitor.recovery_ack_wall = 10., 7.
    monitor.recovery_pose_baseline, monitor.pose_stamp, monitor.pose_rx_wall = 9.5, 9.8, 7.1
    monitor.recovery_timeout = 8.
    monitor.processed, monitor.evidence = 0, {}
    monitor.grid = S(score=lambda *_args: {'valid': True})
    monitor.get_clock = lambda: S(now=lambda: S(nanoseconds=10_000_000_000))
    transform = S(transform=S(translation=S(x=0.,y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
    monitor.buffer = S(lookup_transform=lambda *_args: transform)
    return methods, monitor, msg


def process_recovery_scan(methods, monitor, msg, source, wall, age=0.):
    from types import SimpleNamespace as S
    ns = round(source*1e9)
    msg.header.stamp = S(sec=ns//10**9,nanosec=ns%10**9)
    monitor.pose_stamp, monitor.pose_rx_wall = source, wall
    methods.on_scan(monitor,msg)
    methods.process_pending(monitor,msg,S(nanoseconds=ns),age,wall)


def test_global_ack_rejects_waiting_and_late_pre_response_scans():
    methods, monitor, msg = recovery_monitor_fixture()
    from types import SimpleNamespace as S
    from tf2_ros import TransformException
    ready_buffer = monitor.buffer

    def missing(*_args):
        raise TransformException('Pending exact TF before acknowledgement')

    monitor.recovery_phase = 'WAIT_RESPONSE'
    monitor.buffer = S(lookup_transform=missing)
    monitor.global_future = S(done=lambda: True,result=lambda: S())
    process_recovery_scan(methods,monitor,msg,9.8,6.8,.05)
    assert monitor.pending_tf_wait and not monitor.state.good
    methods.recovery_tick(monitor,7.)
    assert monitor.recovery_sensor_cutoff == 10. and monitor.deadline == 15.
    assert monitor.recovery_phase == 'VERIFY_SCANS'
    monitor.buffer = ready_buffer
    # The waiting scan is still fresh when its exact TF becomes available.
    methods.process_pending(monitor,msg,S(nanoseconds=9_800_000_000),.25,7.05)
    assert monitor.pending is None and monitor.state.good_started is None
    for source in (9.9,10.):  # Late arrivals and equality cannot bridge ACK.
        process_recovery_scan(methods,monitor,msg,source,7.1,.2)
        assert monitor.state.good_started is None and monitor.state.last_stamp is None
    assert monitor.processed == 0  # No pre-response scan entered geometric scoring.
    assert monitor.deadline == 15. and not monitor.state.healthy


def test_post_ack_20hz_recovery_keeps_full_1_4s_after_first_new_scan():
    methods, monitor, msg = recovery_monitor_fixture()
    process_recovery_scan(methods,monitor,msg,9.8,7.01,.2)
    assert monitor.state.good_started is None
    for i in range(28):
        source = 10.05+i*.05
        process_recovery_scan(methods,monitor,msg,source,7.05+i*.05)
        assert not monitor.state.healthy
    assert monitor.state.good_started == 10.05 and len(monitor.state.good) == 15
    assert len(monitor.state.recovery_observations) >= 3
    process_recovery_scan(methods,monitor,msg,11.45,8.45)
    assert monitor.state.healthy and monitor.deadline == 15.


def test_pre_ack_source_filter_does_not_mask_malformed_or_rewound_scan():
    methods, monitor, msg = recovery_monitor_fixture()
    msg.angle_increment = 0.
    process_recovery_scan(methods,monitor,msg,9.8,7.1,.2)
    assert monitor.state.state == 'MISMATCH' and not monitor.state.healthy
    methods, monitor, msg = recovery_monitor_fixture()
    monitor.state.last_stamp = 9.9
    process_recovery_scan(methods,monitor,msg,9.8,7.1,.2)
    assert monitor.state.state == 'MISMATCH' and monitor.state.reason == 'sensor_stamp_rewind'
