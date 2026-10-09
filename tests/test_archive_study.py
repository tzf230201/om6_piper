import _bootstrap  # Repository-local imports; no hardware access.
"""Pure offline helper tests. No hardware entrypoints are imported."""
import unittest
import numpy as np
from archive_retention_study import canonical
from live_mapping import graph_panel


class StudyTests(unittest.TestCase):
    def test_canonical_identity_is_key_not_slot_order(self):
        a=dict(keys=np.array([[-1,0,0,4],[0,0,0,4],[1,0,0,4]],dtype='int64'),
               points=np.array([[-.01,0,0],[0,0,0],[.01,0,0]]))
        order=[2,0,1];b={k:v[order] for k,v in a.items()}
        ca,cb=canonical(a),canonical(b)
        self.assertTrue(all(np.array_equal(x,y) for x,y in zip(ca,cb)))

    def test_empty_slice_can_render(self):
        image=graph_panel(np.empty((0,3)),np.empty((0,2),dtype='int32'),None,'top')
        self.assertEqual(image.shape,(380,600,3))


if __name__=='__main__':unittest.main()
