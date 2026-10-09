import _bootstrap  # Repository-local imports; no hardware access.
"""Offline native binding/corruption tests with temporary databases only."""
import sqlite3
import tempfile
from pathlib import Path
import unittest
import numpy as np
from block_archive_study import Block, compare, ROOT


class BlockTests(unittest.TestCase):
    def setUp(self):
        self.temp=Path(tempfile.mkdtemp(prefix='block_archive_test_',dir=ROOT/'tmp')).resolve()
        self.assertTrue(self.temp.is_relative_to((ROOT/'tmp').resolve()))
        self.path=self.temp/'map.sqlite'
        self.intr=dict(width=64,height=48,fx=60.,fy=60.,cx=32.,cy=24.)
        self.depth=np.full((48,64),.5)

    # Retain the small fixtures for inspection; no cross-runtime Temp permission workaround.

    def test_close_reopen_and_metadata(self):
        a=Block(self.intr,self.path)
        try:
            for i in range(8):
                ok,_=a.update(self.depth,np.eye(4),i,.2*i,True);self.assertTrue(ok)
            a.flush();snapshot=a.snapshot();self.assertGreater(len(snapshot['points']),0)
            self.assertEqual(a.status()['committed'],8)
            self.assertTrue(np.all(snapshot['node_witness'][:,0]==7))
            with self.assertRaises(ValueError):a.update(self.depth[:-1],np.eye(4),10,2.)
            with self.assertRaises(RuntimeError):a.update(self.depth,np.eye(4),7,1.4)
        finally:a.close()
        a=Block(self.intr,self.path)
        try:self.assertTrue(all(compare(a.snapshot(),snapshot).values()))
        finally:a.close()

    def test_corrupt_checksum_is_rejected(self):
        a=Block(self.intr,self.path)
        try:a.update(self.depth,np.eye(4),0,0.,True);a.flush()
        finally:a.close()
        with sqlite3.connect(self.path) as c:
            x,y,z,payload=c.execute('SELECT rx,ry,rz,payload FROM blocks LIMIT 1').fetchone()
            bad=bytearray(payload);bad[16]^=1
            c.execute('UPDATE blocks SET payload=? WHERE rx=? AND ry=? AND rz=?',(bytes(bad),x,y,z))
        a=Block(self.intr,self.path)
        try:
            with self.assertRaisesRegex(RuntimeError,'Corrupt compressed region'):a.snapshot()
        finally:a.close()

    def test_corrupt_size_is_rejected_before_allocation(self):
        a=Block(self.intr,self.path)
        try:a.update(self.depth,np.eye(4),0,0.,True);a.flush()
        finally:a.close()
        with sqlite3.connect(self.path) as c:
            x,y,z,payload=c.execute('SELECT rx,ry,rz,payload FROM blocks LIMIT 1').fetchone()
            bad=bytearray(payload);bad[8:16]=(2**62).to_bytes(8,'little')
            c.execute('UPDATE blocks SET payload=? WHERE rx=? AND ry=? AND rz=?',(bytes(bad),x,y,z))
        a=Block(self.intr,self.path)
        try:
            with self.assertRaisesRegex(RuntimeError,'size outside budget'):a.snapshot()
        finally:a.close()

    def test_partition_and_coalescing_keep_all_metadata(self):
        snapshots=[]
        for cells,interval in ((8,.2),(16,1.0)):
            a=Block(self.intr,self.temp/f'partition_{cells}.sqlite',interval=interval,region_cells=cells)
            try:
                for i in range(25):
                    pose=np.eye(4);pose[0,3]=-.24+.003*i
                    ok,_=a.update(self.depth,pose,i,.2*i,True);self.assertTrue(ok)
                snapshots.append(a.snapshot())
            finally:a.close()
        a,b=snapshots;compare(a,b)
        def order(g):return np.lexsort(tuple(g['keys'][:,i] for i in range(3,-1,-1)))
        for key in ('normals','radii','node_witness'):
            np.testing.assert_array_equal(a[key][order(a)],b[key][order(b)])
        from block_archive_study import sorted_edges
        for key in ('edge_seen','edge_witness'):
            np.testing.assert_array_equal(a[key][sorted_edges(a)[1]],b[key][sorted_edges(b)[1]])
        with self.assertRaisesRegex(RuntimeError,'format/voxel mismatch'):
            Block(self.intr,self.temp/'partition_8.sqlite',region_cells=16)

    def test_disk_quota_rolls_back_and_latches_failure(self):
        intr=dict(width=640,height=480,fx=200.,fy=200.,cx=320.,cy=240.)
        a=Block(intr,self.path,max_mib=1);last=None;failed=False
        try:
            for i in range(40):
                pose=np.eye(4);pose[0,3]=3.*i
                a.update(np.full((480,640),.7),pose,i,float(i),True)
                try:a.flush()
                except RuntimeError:
                    failed=True;self.assertEqual(a.status()['failed'],1)
                    self.assertEqual(a.status()['committed'],i)
                    with self.assertRaises(RuntimeError):a.update(np.full((480,640),.7),pose,i+1,float(i+1))
                    break
                last=a.snapshot()
        finally:a.close()
        self.assertTrue(failed,'Disk quota was not reached');self.assertIsNotNone(last)
        a=Block(intr,self.path,max_mib=1)
        try:compare(a.snapshot(),last)
        finally:a.close()


if __name__=='__main__':unittest.main()
