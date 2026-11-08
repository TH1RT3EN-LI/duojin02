import unittest
import math
import dataclasses
import numpy as np
from duojin02_slam_tools.local_submap import RawObservation, RigidSubmapBuilder, RigidSubmap, FrozenScan, BudgetedUpdateQueue, AcceptedUpdate, ScanSubmapObservation, transform, relative
from duojin02_slam_tools.continuous_geometry import Surface, residual_jacobian, refine_shadow


def obs(source, points):
    return RawObservation(source, 1, 1, "synthetic-explicit.v1", tuple(points), tuple(range(len(points))))


def room():
    # Separated wall rosters avoid joining corners as a synthetic diagonal.
    points = []
    for t in np.linspace(-1.8, 1.8, 121): points.append((float(t), -2.0))
    for t in np.linspace(-1.8, 1.8, 121): points.append((2.0, float(t)))
    for t in np.linspace(-1.8, 1.8, 121): points.append((float(t), 2.0))
    return points


class ShadowTests(unittest.TestCase):
    def test_fixed_association_jacobian(self):
        rng = np.random.default_rng(20261019)
        worst = 0
        for _ in range(64):
            point, a, b, pose = rng.normal(size=2), rng.normal(size=2), rng.normal(size=2), rng.normal(size=3)
            surf = Surface("a", "source", (1,2), tuple(a), tuple(b))
            _, j = residual_jacobian(point, pose, surf)
            fd = []
            for axis in range(3):
                step = np.zeros(3);step[axis]=1e-6
                fd.append((residual_jacobian(point,pose+step,surf)[0]-residual_jacobian(point,pose-step,surf)[0])/2e-6)
            worst=max(worst,float(np.max(np.abs(j-np.asarray(fd)))))
        self.assertLess(worst, 2e-9)

    def test_rich_subgrid_and_rigid_equivariance(self):
        ref = obs("ref", room());query=obs("query",room())
        m=RigidSubmap("a",(FrozenScan(ref,(0.,0.,0.)),),(0.,0.,0.))
        result=refine_shadow(query,m,(.003,-.004,.001),query_stride=8)
        self.assertTrue(result["observable"])
        self.assertLess(np.linalg.norm(result["local_pose"]), 1e-5)
        moved=m.moved((3.,-5.,.71))
        # Transform the same initial pose with the same common frame change.
        xy=transform(((.003,-.004),),moved.world_pose)[0]
        result2=refine_shadow(query,moved,(*xy,.711),query_stride=8)
        self.assertLess(np.max(np.abs(np.asarray(result2['local_pose'])-result['local_pose'])),1e-8)
        self.assertEqual(m.geometry_version,moved.geometry_version)
        self.assertEqual(moved.pose_version,1)
        self.assertEqual(m.scans,moved.scans)

    def test_corridor_exposes_weak_direction(self):
        wall=[(float(t),0.) for t in np.linspace(-2.,2.,201)]
        ref=obs("ref",wall);query=obs("query",wall)
        m=RigidSubmap("a",(FrozenScan(ref,(0.,0.,0.)),),(0.,0.,0.))
        r=refine_shadow(query,m,(.003,.01,0.))
        self.assertFalse(r['observable']);self.assertEqual(r['status'],'weak_geometry')
        self.assertLess(abs(r['dimensionless_eigenvalues'][0]),1e-12)
        self.assertFalse(r['covariance_calibrated'])

    def test_query_dedup_budget_and_geometry_identity(self):
        first=obs("raw1",[(0.,0.),(.1,0.)]);second=obs("raw2",[(0.,0.),(.1,0.)])
        builder=RigidSubmapBuilder("a",(0.,0.,0.),max_scans=1)
        receipt=builder.view().query_receipt(first)
        self.assertTrue(builder.insert_once(first,(0.,0.,0.),receipt))
        self.assertFalse(builder.insert_once(first,(0.,0.,0.),receipt))
        with self.assertRaises(ValueError): builder.view().query_receipt(first)
        with self.assertRaises(OverflowError):builder.insert_once(second,(0.,0.,0.),builder.view().query_receipt(second))
        frozen=builder.freeze()
        with self.assertRaises(ValueError):builder.insert_once(second,(0.,0.,0.),receipt)
        changed=dataclasses.replace(first,points=((0.,0.),(.101,0.)))
        revised=RigidSubmap('a',(FrozenScan(changed,(0.,0.,0.)),),(0.,0.,0.))
        self.assertNotEqual(frozen.geometry_version,revised.geometry_version)
        overlap=RigidSubmap('b',(FrozenScan(first,(1.,0.,0.)),),(0.,0.,0.))
        self.assertEqual(frozen.overlap(overlap),('raw1',))

    def test_support_loss_is_reported_as_failure(self):
        reference=obs('ref',room())
        # A coherent extra stripe matches at the initial offset and disappears
        # after the stronger room support corrects the pose. Low final residual
        # must therefore not conceal dropped observations.
        query=obs('query',room()+[(float(x),-1.8) for x in np.linspace(-1.5,1.5,100)])
        m=RigidSubmap('support',(FrozenScan(reference,(0.,0.,0.)),),(0.,0.,0.))
        result=refine_shadow(query,m,(0.,-.1,0.),query_stride=1,max_iterations=12)
        self.assertLess(result['support_retention'],.9)
        self.assertEqual(result['status'],'support_lost')
        self.assertFalse(result['usable_for_precision'])

    def test_nested_input_immutability_and_receipt_support(self):
        points=[[0.,0.],[.1,0.]];origins=[[0.,0.],[.01,0.]]
        raw=RawObservation('raw',1,2,'prepared-test',points,[0,1],ray_origins=origins)
        identity=raw.identity
        points[1][0]=999.;origins[1][0]=999.
        self.assertEqual(raw.points[1][0],.1);self.assertEqual(raw.ray_origins[1][0],.01)
        self.assertEqual(raw.identity,identity)
        scan_pose=[0.,0.,0.];scans=[FrozenScan(raw,scan_pose)];world=[0.,0.,0.]
        frozen=RigidSubmap('s',scans,world);version=frozen.geometry_version
        scan_pose[0]=999.;world[0]=999.;scans.clear()
        self.assertEqual(frozen.geometry_version,version);self.assertEqual(frozen.scans[0].local_pose[0],0.)
        loaded=RawObservation.from_dict({'source_id':'other','raw_stamp_ns':1,'physical_reference_ns':2,'geometry_kind':'prepared','points':[[0.,0.],[.1,0.]],'beam_indices':[0,1],'calibration_version':'cal-v2','ray_origins':[[0.,0.],[.01,0.]],'source_geometry_hash':'abc'})
        self.assertEqual(loaded.calibration_version,'cal-v2');self.assertEqual(loaded.source_geometry_hash,'abc')
        builder=RigidSubmapBuilder('s',(0.,0.,0.))
        old_receipt=builder.view().query_receipt(loaded)
        builder.insert_once(raw,(0.,0.,0.),builder.view().query_receipt(raw))
        with self.assertRaises(ValueError):builder.insert_once(loaded,(0.,0.,0.),old_receipt)
        self.assertTrue(builder.insert_once(loaded,(0.,0.,0.),builder.view().query_receipt(loaded)))
        self.assertIn('other',builder.insertion_receipts)
        result={'geometry_version':frozen.geometry_version,'local_pose':[0.,0.,0.],'conditional_shape':np.eye(3).tolist()}
        factor=ScanSubmapObservation.from_shadow(loaded,frozen,result)
        self.assertTrue(factor.validate_support(frozen.moved((1.,2.,.3))))
        changed=RigidSubmap('s',(FrozenScan(dataclasses.replace(raw,points=((0.,0.),(.101,0.))),(0.,0.,0.)),),(0.,0.,0.))
        with self.assertRaises(ValueError):factor.validate_support(changed)

    def test_accepted_updates_survive_budget_and_callback_failure(self):
        queue=BudgetedUpdateQueue()
        for i in range(10): self.assertTrue(queue.accept(AcceptedUpdate(str(i), i)))
        self.assertFalse(queue.accept(AcceptedUpdate('0',0)))
        with self.assertRaises(ValueError):queue.accept(AcceptedUpdate('0',999))
        with self.assertRaises(RuntimeError):queue.run_budget(lambda _: (_ for _ in ()).throw(RuntimeError('failed')))
        self.assertEqual(len(queue.pending_ids),10)
        completed=[]
        while queue.pending_ids:
            status=queue.run_budget(lambda u:completed.append(u.update_id),max_updates=3,budget_seconds=1.)
            self.assertTrue(status['all_accepted_accounted'])
        self.assertEqual(completed,[str(i) for i in range(10)])
        self.assertEqual(queue.completed_ids,completed)


if __name__=='__main__':unittest.main()
