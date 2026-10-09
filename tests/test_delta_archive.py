import _bootstrap  # Repository-local imports; no hardware access.
"""File-only contracts for the bounded delta archive; no camera or robot APIs."""
import sqlite3
import tempfile
import unittest
import subprocess
import sys
import os
from pathlib import Path
import numpy as np
from block_archive_study import Block, DELTA_DLL, ROOT
from summarize_block_archive import rich_parity


class DeltaTests(unittest.TestCase):
    def setUp(self):
        self.root=Path(tempfile.mkdtemp(prefix='delta_archive_test_',dir=ROOT/'tmp')).resolve()
        self.assertTrue(self.root.is_relative_to((ROOT/'tmp').resolve()))
        self.intr=dict(width=64,height=48,fx=60.,fy=60.,cx=32.,cy=24.)
        self.depth=np.full((48,64),.5)

    def archive(self,name='delta',**kw):
        config=kw.pop('delta_config',dict(regions=64,kib=32768,tail=32))
        return Block(self.intr,self.root/(name+'.sqlite'),dll=DELTA_DLL,delta_config=config,**kw)

    def test_checkpoints_tails_reopen_and_resume_exact(self):
        a=self.archive();b=Block(self.intr,self.root/'reference.sqlite')
        try:
            for i in range(37):
                p=np.eye(4);p[0,3]=-.08+.001*(i%11)
                a.update(self.depth,p,i,.2*i,True);b.update(self.depth,p,i,.2*i,True);a.flush()
            expected=b.snapshot();rich_parity(a.snapshot(),expected)
            metrics=a.metrics();self.assertGreater(sum(m['compacted_regions'] for m in metrics),0)
            self.assertLessEqual(max(m['compacted_raw_bytes'] for m in metrics),32768*1024)
            self.assertLessEqual(max(m['compacted_regions'] for m in metrics),64)
            with sqlite3.connect(self.root/'delta.sqlite') as c:
                self.assertGreater(c.execute('SELECT count(*) FROM blocks').fetchone()[0],0)
                self.assertGreater(c.execute('SELECT count(*) FROM deltas').fetchone()[0],0)
        finally:a.close();b.close()
        a=self.archive()
        try:
            rich_parity(a.snapshot(),expected)
            a.update(np.zeros_like(self.depth),np.eye(4),38,8.,True)
            rich_parity(a.snapshot(),expected) # Unknown evidence cannot erase the archive.
            with self.assertRaises(RuntimeError):a.update(self.depth,np.eye(4),38,8.,True)
        finally:a.close()

    def test_tail_limit_rolls_back_whole_batch(self):
        config=dict(regions=1,kib=32768,tail=2);a=self.archive(delta_config=config)
        before=None
        try:
            for i in range(2):
                a.update(self.depth,np.eye(4),i,float(i),True);a.flush()
            before=a.snapshot()
            a.update(self.depth,np.eye(4),2,2.,True)
            with self.assertRaisesRegex(RuntimeError,'hard tail limit'):a.flush()
            self.assertEqual(a.status()['committed'],2);self.assertEqual(a.status()['failed'],1)
            with self.assertRaises(RuntimeError):a.update(self.depth,np.eye(4),3,3.,True)
        finally:a.close()
        a=self.archive(delta_config=dict(config,regions=64))
        try:
            rich_parity(a.snapshot(),before)
            a.update(self.depth,np.eye(4),2,2.,True);a.flush();self.assertEqual(a.status()['committed'],3)
        finally:a.close()

    def test_delta_checksum_corruption_rejected(self):
        a=self.archive()
        try:a.update(self.depth,np.eye(4),0,0.,True);a.flush()
        finally:a.close()
        with sqlite3.connect(self.root/'delta.sqlite') as c:
            x,y,z,seq,payload=c.execute('SELECT rx,ry,rz,seq,payload FROM deltas LIMIT 1').fetchone()
            bad=bytearray(payload);bad[16]^=1
            c.execute('UPDATE deltas SET payload=? WHERE rx=? AND ry=? AND rz=? AND seq=?',(bytes(bad),x,y,z,seq))
        a=self.archive()
        try:
            with self.assertRaisesRegex(RuntimeError,'Corrupt compressed region'):a.snapshot()
        finally:a.close()

    def test_declared_size_metadata_mismatch_rejected(self):
        a=self.archive()
        try:a.update(self.depth,np.eye(4),0,0.,True);a.flush()
        finally:a.close()
        with sqlite3.connect(self.root/'delta.sqlite') as c:c.execute('UPDATE deltas SET raw_bytes=raw_bytes+1')
        a=self.archive()
        try:
            with self.assertRaisesRegex(RuntimeError,'Corrupt delta size'):a.snapshot()
        finally:a.close()

    def test_disk_quota_rolls_back_exactly(self):
        self.intr=dict(width=640,height=480,fx=200.,fy=200.,cx=320.,cy=240.)
        a=self.archive(max_mib=1);last=None;failed=False
        try:
            for i in range(40):
                p=np.eye(4);p[0,3]=3.*i
                a.update(np.full((480,640),.7),p,i,float(i),True)
                try:a.flush()
                except RuntimeError as exc:
                    self.assertIn('full',str(exc).lower());failed=True
                    self.assertEqual(a.status()['committed'],i);self.assertEqual(a.status()['failed'],1);break
                last=a.snapshot()
        finally:a.close()
        self.assertTrue(failed);self.assertIsNotNone(last)
        a=self.archive(max_mib=1)
        try:rich_parity(a.snapshot(),last)
        finally:a.close()

    def test_unclean_exit_keeps_only_committed_data(self):
        path=self.root/'crash.sqlite'
        child=subprocess.run([sys.executable,str(Path(__file__).resolve()),'--crash-fixture',str(path)],
            capture_output=True,text=True,creationflags=subprocess.CREATE_NO_WINDOW,timeout=30)
        self.assertEqual(child.returncode,17,child.stdout+child.stderr)
        a=Block(self.intr,path,dll=DELTA_DLL,delta_config=dict(regions=64,kib=32768,tail=32))
        ref=Block(self.intr,self.root/'crash_reference.sqlite')
        try:
            for i in range(12):ref.update(self.depth,np.eye(4),i,.2*i,True)
            self.assertEqual(a.status()['committed'],12)
            rich_parity(a.snapshot(),ref.snapshot())
        finally:a.close();ref.close()


if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--crash-fixture':
        target=Path(sys.argv[2]).resolve()
        if not target.is_relative_to((ROOT/'tmp').resolve()) or target.exists():raise ValueError('Fresh test fixture only')
        intr=dict(width=64,height=48,fx=60.,fy=60.,cx=32.,cy=24.)
        a=Block(intr,target,dll=DELTA_DLL,delta_config=dict(regions=64,kib=32768,tail=32))
        for i in range(12):a.update(np.full((48,64),.5),np.eye(4),i,.2*i,True);a.flush()
        pose=np.eye(4);pose[0,3]=99.
        a.update(np.full((48,64),.5),pose,100,100.,True) # Pending only: not an acknowledged disk commit.
        os._exit(17) # Deliberately skip destructors, for this test child only.
    unittest.main()
