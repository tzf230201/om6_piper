import _bootstrap  # Repository-local imports; no hardware access.
import unittest
import numpy as np
from replay_robot_mapping import NativeGraph, reprojection_residual

INTR = dict(width=64, height=48, fx=60., fy=60., cx=32., cy=24.)


class ReplayTests(unittest.TestCase):
    def test_plane_reprojection_and_unknown(self):
        depth = np.full((48,64), .5)
        p = np.eye(4)
        r = reprojection_residual(depth, depth, p, p, INTR)
        self.assertGreater(len(r), 10)
        np.testing.assert_allclose(r, 0)
        translated = p.copy()
        translated[2,3] = .1
        np.testing.assert_allclose(reprojection_residual(depth, depth-.1, p, translated, INTR), 0, atol=1e-12)
        self.assertEqual(len(reprojection_residual(depth, depth*0, p, p, INTR)), 0)

    def test_native_pose_and_invalid_rejection(self):
        depth = np.full((48,64), .5)
        native = NativeGraph(INTR)
        self.addCleanup(native.close)
        bad = np.eye(4)
        bad[0,0] = -1
        with self.assertRaises(RuntimeError):
            native.update(depth, bad, 0, 0)
        # Failed validation must not consume this frame ID/time.
        pose = np.eye(4)
        pose[2,3] = .12
        native.update(depth, pose, 0, 0)
        points, edges = native.snapshot()
        self.assertGreater(len(points), 10)
        self.assertGreater(len(edges), 10)
        np.testing.assert_allclose(points[:,2], .62, atol=1e-12)
        with self.assertRaises(RuntimeError):
            native.update(depth, pose, 0, 0)
        native.update(depth*0, pose, 1, 1)
        self.assertEqual(len(native.snapshot()[0]), 0)


if __name__ == '__main__':
    unittest.main()
