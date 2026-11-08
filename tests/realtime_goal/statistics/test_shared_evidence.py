"""Independent closed forms and dense Gaussian checks for the E4 shadow operator."""
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/duojin02_slam_tools'))
from duojin02_slam_tools.shared_evidence import (EvidenceBlock, PersistentSharedGaussian,
    SharedVariable, gaussian_nll, gaussian_nll_derivative)


class SharedEvidenceChecks(unittest.TestCase):
    def model(self, shared_variance=.0004, budget=128):
        return PersistentSharedGaussian(1, [SharedVariable('wall-A', 'normal surface displacement',
            'survey-A/wall-A', 'frozen geometry version')], [[shared_variance]], 'geometry-A',
            'fixed-x-chart-A', max_observations=budget)

    def block(self, index, y, independent_variance=.0001):
        return EvidenceBlock(str(index), (f'scan-{index}/beam-0',), f'scan-{index}', 'geometry-A',
            100 + index, [y], [[1]], [[1]], [[independent_variance]])

    def test_persistent_shared_closed_form_and_conditional_chain(self):
        m = self.model(); increments = []; ys = [.014, -.002, .021, .003]
        for i, y in enumerate(ys):
            m.append(self.block(i, y)); increments.append(m.last_conditional_nll([.01]))
        n = len(ys); residual = np.asarray(ys) - .01
        # Sherman-Morrison eigenbasis: N-1 independent contrasts plus common mode.
        quadratic = sum(residual ** 2) / .0001 - .0004 * sum(residual) ** 2 / (.0001 * (.0001 + n * .0004))
        logdet = (n - 1) * math.log(.0001) + math.log(.0001 + n * .0004)
        expected = .5 * (quadratic + logdet + n * math.log(2 * math.pi))
        self.assertAlmostEqual(m.nll([.01]), expected, places=10)
        self.assertAlmostEqual(sum(increments), expected, places=10)
        independently_reset = sum(gaussian_nll([y-.01], [[.0005]]) for y in ys)
        self.assertGreater(abs(independently_reset - expected), .1)
        # Information must saturate at 1/shared variance, not grow without bound.
        h = 1e-5
        info = (m.nll([.01+h]) - 2*m.nll([.01]) + m.nll([.01-h])) / h**2
        self.assertAlmostEqual(info, n / (.0001 + n * .0004), delta=1e-3)

    def test_duplicate_conflict_budget_and_geometry_atomic(self):
        m = self.model(budget=2); b = self.block(0, .02)
        self.assertTrue(m.append(b)); before = m.audit_record()
        self.assertFalse(m.append(b)); self.assertEqual(before, m.audit_record())
        with self.assertRaises(ValueError): m.append(self.block(0, .03))
        copied = EvidenceBlock('copy', b.measurement_ids, 'scan-0', 'geometry-A', 100,
                               b.y, b.J, b.B, b.R)
        with self.assertRaises(ValueError): m.append(copied)
        changed = EvidenceBlock('new', ('scan-new/0',), 'scan-new', 'geometry-B', 102,
                                [.01], [[1]], [[1]], [[.0001]])
        with self.assertRaises(ValueError): m.append(changed)
        self.assertEqual(before, m.audit_record())
        m.append(self.block(1, .01)); before = m.audit_record()
        with self.assertRaises(ValueError): m.append(self.block(2, .02))
        self.assertEqual(before, m.audit_record())

    def test_zero_shared_independent_limit_and_permutation(self):
        ys = [.01, .03, -.02]; m = self.model(0); p = self.model(0)
        for i in range(3): m.append(self.block(i, ys[i]))
        for i in [2, 0, 1]: p.append(self.block(i, ys[i]))
        expected = sum(gaussian_nll([y], [[.0001]]) for y in ys)
        self.assertAlmostEqual(m.nll([0]), expected, places=12)
        self.assertAlmostEqual(p.nll([0]), expected, places=12)
        # Distinct scans sharing a wall ARE still independent physical epsilon.
        self.assertEqual(m.observation_count, 3)

    def test_evidence_buffers_cannot_be_mutated_after_acceptance(self):
        m = self.model(); block = self.block(0, .02); m.append(block)
        before = m.nll([0]); audit = m.audit_record()
        for matrix in (block.y, block.J, block.B, block.R, m.shared_covariance):
            with self.assertRaises(ValueError): matrix.setflags(write=True)
            with self.assertRaises(ValueError): matrix.flat[0] = 9.
        self.assertEqual(before, m.nll([0])); self.assertEqual(audit, m.audit_record())

    def test_logdet_derivative(self):
        x = .4; r = np.array([x-1, .3+2*x]); v = np.array([[1+x*x,.1],[.1,2+x]])
        dr = np.array([1, 2]); dv = np.array([[2*x,0],[0,1]])
        def objective(t): return gaussian_nll([t-1, .3+2*t], [[1+t*t,.1],[.1,2+t]])
        numerical = (objective(x+1e-6)-objective(x-1e-6))/2e-6
        self.assertAlmostEqual(gaussian_nll_derivative(r,v,dr,dv), numerical, places=8)
        omitted = dr @ np.linalg.solve(v,r) - .5*np.linalg.solve(v,r)@dv@np.linalg.solve(v,r)
        self.assertGreater(abs(numerical - omitted), .1)

    def test_schur_joint_map_and_coordinate_transport(self):
        # Fixed matrices only: eliminate eta in joint MAP, compare marginalized GLS.
        rng = np.random.default_rng(7341); j = rng.normal(size=(9, 3)); b = rng.normal(size=(9,2))
        y = rng.normal(size=9); r = np.diag(np.linspace(.1,.4,9)); s = np.array([[.3,.04],[.04,.2]])
        v = r+b@s@b.T; ri = np.linalg.inv(r)
        hxx = j.T@ri@j; hxeta = j.T@ri@b; he = b.T@ri@b + np.linalg.inv(s)
        rhs = j.T@ri@y-hxeta@np.linalg.solve(he,b.T@ri@y)
        schur = hxx-hxeta@np.linalg.solve(he,hxeta.T)
        gls = np.linalg.solve(j.T@np.linalg.solve(v,j),j.T@np.linalg.solve(v,y))
        np.testing.assert_allclose(np.linalg.solve(schur,rhs), gls, atol=1e-12)
        # A fixed invertible observation coordinate change preserves quadratic;
        # NLL changes by log|det A| (density units), not by a pose-dependent term.
        a = np.diag(np.linspace(.5,2.,9)); residual = y-j@gls
        delta = gaussian_nll(a@residual,a@v@a.T)-gaussian_nll(residual,v)
        self.assertAlmostEqual(delta, np.linalg.slogdet(a)[1], places=11)

    def test_no_fictitious_variance_or_validation(self):
        with self.assertRaises(np.linalg.LinAlgError): gaussian_nll([1], [[0]])
        with self.assertRaises(ValueError): self.model(-.01)
        with self.assertRaises(ValueError): SharedVariable('a','wall','s','map',calibration_status='true')
        self.assertTrue(self.model().audit_record()['shadow_only'])
        self.assertEqual(self.model().audit_record()['physical_accuracy_calibration'], 'unknown')


if __name__ == '__main__': unittest.main()
