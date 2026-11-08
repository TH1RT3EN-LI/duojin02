"""Non-mirrored Jacobian, gauge, budget, source and transaction controls."""
import copy
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/duojin02_slam_tools'))
from duojin02_slam_tools.local_submap import RawObservation, FrozenScan, RigidSubmap, ScanSubmapObservation
from duojin02_slam_tools.shadow_submap_graph import ShadowSubmapGraph, residual_jacobians


def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],math.remainder(a[2]+b[2],2*math.pi))


def local(a,b):
    c,s=math.cos(a[2]),math.sin(a[2]);x,y=b[0]-a[0],b[1]-a[1]
    return (c*x+s*y,-s*x+c*y,math.remainder(b[2]-a[2],2*math.pi))


def submap(key,world=(0.,0.,0.),sources=None):
    scans=[]
    for source in sources or ('shared-reference','reference-'+key):
        raw=RawObservation(source,1,2,'synthetic',((0.,0.),(.1,0.),(.1,.1)),(0,1,2))
        scans.append(FrozenScan(raw,(0.,0.,0.)))
    return RigidSubmap(key,tuple(scans),world)


def observation(m,q,measurement):
    return ScanSubmapObservation(q,'geometry-'+q,m.submap_id,m.geometry_version,
                                m.source_ids,tuple(measurement),tuple(map(tuple,np.eye(3))))


def fixture(gauge=(0.,0.,0.)):
    s0=submap('s0',compose(gauge,(0.,0.,0.)))
    s1=submap('s1',compose(gauge,(.4,.1,.08)))
    desired_query=(1.,.2,.1);desired_submap=(.3,-.2,.05)
    g=ShadowSubmapGraph('test','s0',(.1,.1,.2))
    g.add_submap(s0);g.add_submap(s1)
    g.add_observation('f0','q',observation(s0,'q',desired_query),
                      compose(gauge,(1.4,.7,.3)),np.eye(3),0)
    g.add_observation('f1','q',observation(s1,'q',local(desired_submap,desired_query)),
                      compose(gauge,(1.4,.7,.3)),np.eye(3),0)
    return g,s0,s1,desired_query,desired_submap


class SubmapGraphTests(unittest.TestCase):
    def test_both_pose_jacobians_against_independent_finite_differences(self):
        rng=np.random.default_rng(20191019)
        for _ in range(100):
            s,q,z=rng.normal(size=(3,3))
            r,js,jq=residual_jacobians(s,q,z)
            if abs(abs(r[2])-math.pi)<.01:continue
            for which,jac in ((0,js),(1,jq)):
                columns=[]
                for axis in range(3):
                    d=np.zeros(3);d[axis]=1e-6
                    plus=residual_jacobians(s+d if which==0 else s,q+d if which==1 else q,z)[0]
                    minus=residual_jacobians(s-d if which==0 else s,q-d if which==1 else q,z)[0]
                    columns.append((plus-minus)/2e-6)
                np.testing.assert_allclose(jac,np.asarray(columns).T,atol=2e-9,rtol=0)

    def test_explicit_submap_state_anchor_gauge_and_budget_resume(self):
        g,s0,s1,q,s=fixture()
        first=g.run_budget(1,10.)
        self.assertTrue(first['scheduling_yield'])
        while g.finalize(20,10.)['scheduling_yield']:pass
        snap=g.snapshot()
        np.testing.assert_allclose(snap['queries']['q']['world_pose'],q,atol=1e-9)
        np.testing.assert_allclose(snap['submaps']['s1']['world_pose'],s,atol=1e-9)
        self.assertEqual(snap['submaps']['s0']['world_pose'],list(s0.world_pose))
        self.assertEqual(snap['last_solver']['nullity'],0)
        full,*_=fixture();full.finalize(30,10.)
        np.testing.assert_allclose(full.snapshot()['queries']['q']['world_pose'],snap['queries']['q']['world_pose'],atol=1e-11)
        moved,*_=fixture((3.,-2.,.7));moved.finalize(30,10.)
        np.testing.assert_allclose(moved.snapshot()['queries']['q']['world_pose'],compose((3.,-2.,.7),q),atol=1e-9)
        self.assertIn('shared-reference',snap['source_overlap'])
        self.assertFalse(snap['marginalization_implemented'])

    def test_singular_direction_is_reported_without_diagonal(self):
        m=submap('s0');g=ShadowSubmapGraph('singular','s0',(.1,.1,.2));g.add_submap(m)
        g.add_observation('wall','q',observation(m,'q',(1.,2.,.3)),(4.,5.,.8),np.diag([0.,1.,1.]),0)
        g.finalize(20,10.);snap=g.snapshot()
        self.assertEqual(snap['last_solver']['nullity'],1)
        self.assertEqual(snap['last_solver']['diagonal_regularization'],0.)
        self.assertEqual(snap['queries']['q']['world_pose'][0],4.)
        self.assertIn('rank_deficient',snap['last_solver']['status'])

    def test_duplicate_self_geometry_and_version_reject_atomically(self):
        g,m,_,_,_=fixture();before=g.snapshot()
        existing=observation(m,'q',(1.,.2,.1))
        self.assertFalse(g.add_observation('f0','q',existing,(1.4,.7,.3),np.eye(3),0))
        with self.assertRaises(ValueError):g.add_observation('copied','other-q',existing,(0,0,0),np.eye(3),0)
        self_match=observation(m,'shared-reference',(0,0,0))
        with self.assertRaises(ValueError):g.add_observation('self','new',self_match,(0,0,0),np.eye(3),0)
        changed=copy.deepcopy(existing.__dict__);changed['geometry_version']='other'
        with self.assertRaises(ValueError):g.add_observation('changed','new',ScanSubmapObservation(**changed),(0,0,0),np.eye(3),0)
        with self.assertRaises(ValueError):g.add_observation('version','new',observation(m,'new',(0,0,0)),(0,0,0),np.eye(3),1)
        with self.assertRaises(ValueError):g.add_observation('bad-shape','new',observation(m,'new',(0,0,0)),(0,0,0),np.diag([1,1,-1]),0)
        self.assertEqual(g.snapshot(),before)

    def test_reverse_query_ownership_across_submaps_rejects_atomically(self):
        g,m,s1,_,_=fixture();before=g.snapshot()
        other_map=observation(s1,'q',(0.,0.,0.))
        with self.assertRaises(ValueError):
            g.add_observation('split-query','q-copy',other_map,(0,0,0),np.eye(3),0)
        changed=copy.deepcopy(other_map.__dict__);changed['query_identity']='changed-raw-geometry'
        with self.assertRaises(ValueError):
            g.add_observation('changed-query','q',ScanSubmapObservation(**changed),(0,0,0),np.eye(3),0)
        self.assertEqual(g.snapshot(),before)

    def test_new_query_can_use_known_optimized_pose_snapshot(self):
        g,_,s1,_,_=fixture();g.finalize(30,10.)
        version=g.snapshot()['submaps']['s1']['pose_version']
        self.assertGreater(version,0)
        self.assertTrue(g.add_observation('fresh','fresh-q',observation(s1,'fresh-q',(1,0,0)),
                                         (2,0,0),np.eye(3),version))
        before=g.snapshot()
        with self.assertRaises(ValueError):
            g.add_observation('future','future-q',observation(s1,'future-q',(1,0,0)),
                              (2,0,0),np.eye(3),version+100)
        self.assertEqual(g.snapshot(),before)

    def test_solver_failure_rolls_back_and_snapshot_has_no_live_alias(self):
        g,*_=fixture();before=g.snapshot()
        with patch.object(g,'_iteration',side_effect=RuntimeError('injected compute failure')):
            with self.assertRaises(RuntimeError):g.finalize(20,10.)
        after=g.snapshot()
        for key in ('queries','submaps','state_revision','accepted_ledger','included_factor_ids'):
            self.assertEqual(before[key],after[key])
        self.assertEqual(len(after['failures']),1)
        after['submaps']['s0']['world_pose'][0]=999.
        self.assertNotEqual(g.snapshot()['submaps']['s0']['world_pose'][0],999.)
        g.finalize(20,10.)
        self.assertEqual(g.snapshot()['pending_factor_ids'],[])

    def test_new_admission_during_solve_stays_pending_and_resumes(self):
        g,m,*_=fixture();original=g._iteration;called=False
        def adding(poses,factors):
            nonlocal called
            if not called:
                called=True
                g.add_observation('late','late-q',observation(m,'late-q',(2.,0.,0.)),(2.1,.1,.1),np.eye(3),0)
            return original(poses,factors)
        with patch.object(g,'_iteration',side_effect=adding):
            tx=g.run_budget(1,10.)
        self.assertEqual(tx['newer_pending_factor_ids'],['late'])
        self.assertEqual(g.snapshot()['accepted_factor_count'],3)
        self.assertEqual(g.snapshot()['pending_factor_ids'],['late'])
        g.finalize(30,10.)
        self.assertEqual(g.snapshot()['pending_factor_ids'],[])
        np.testing.assert_allclose(g.snapshot()['queries']['late-q']['world_pose'],(2,0,0),atol=1e-9)

    def test_actual_background_thread_does_not_hold_frontend_admission_lock(self):
        g,m,*_=fixture();original=g._iteration
        entered,release,admitted=threading.Event(),threading.Event(),threading.Event()
        failures=[]
        def held(poses,factors):
            entered.set()
            if not release.wait(2):raise RuntimeError('test worker release timed out')
            return original(poses,factors)
        def worker():
            try:g.run_budget(1,10.)
            except Exception as error:failures.append(error)
        def frontend():
            try:
                g.add_observation('concurrent','concurrent-q',observation(m,'concurrent-q',(2,0,0)),
                                  (2.1,.1,.1),np.eye(3),0)
                admitted.set()
            except Exception as error:failures.append(error)
        with patch.object(g,'_iteration',side_effect=held):
            thread=threading.Thread(target=worker);thread.start()
            try:
                self.assertTrue(entered.wait(1))
                incoming=threading.Thread(target=frontend);incoming.start()
                self.assertTrue(admitted.wait(1))
            finally:
                release.set();thread.join(3)
                if 'incoming' in locals():incoming.join(3)
        self.assertFalse(failures)
        self.assertEqual(g.snapshot()['pending_factor_ids'],['concurrent'])

    def test_external_lists_cannot_mutate_accepted_evidence(self):
        m=submap('s0');g=ShadowSubmapGraph('immutable','s0',(.1,.1,.2));g.add_submap(m)
        shape=np.eye(3);z=[1.,0.,0.]
        data=observation(m,'q',z).__dict__.copy();data['reference_source_ids']=list(m.source_ids);data['query_local_pose']=z
        obs=ScanSubmapObservation(**data)
        g.add_observation('f','q',obs,[1.1,.1,.1],shape,0)
        before=g.snapshot();z[0]=999.;shape[0,0]=999.;data['reference_source_ids'].clear()
        self.assertEqual(g.snapshot(),before)

if __name__=='__main__':unittest.main()
