import _bootstrap  # Repository-local imports; no hardware access.
"""Regional read contracts, using only fresh file fixtures."""
import sqlite3
import tempfile
import threading
import unittest
from pathlib import Path
import numpy as np
from regional_read_study import Reader, Block, CONFIG, WRITER_DLL, ROOT, crop, rich_parity
from regional_read_job import RegionalJob


class RegionalTests(unittest.TestCase):
    def setUp(self):
        self.root=ROOT/'tmp'/Path(tempfile.mkdtemp(prefix='regional_read_',dir=ROOT/'tmp')).name
        self.path=self.root/'map.sqlite';self.intr=dict(width=64,height=48,fx=60.,fy=60.,cx=32.,cy=24.)
        self.depth=np.full((48,64),.5)

    def writer(self,**kw):return Block(self.intr,self.path,dll=WRITER_DLL,delta_config=CONFIG,**kw)

    def populate(self,count=12):
        a=self.writer()
        try:
            for i in range(count):
                pose=np.eye(4);pose[0,3]=-.01*(i%3)
                a.update(self.depth,pose,i,float(i),True);a.flush()
            return a.snapshot()
        finally:a.close()

    def test_negative_regions_boundary_links_and_reopen(self):
        expected=self.populate();reader=Reader(self.path)
        try:
            for lower,upper in (([-2,-2,5],[2,2,7]),([-1,-1,6],[0,0,7]),([100,100,100],[101,101,101])):
                g,s=reader.query(lower,upper);rich_parity(g,crop(expected,lower,upper))
                self.assertEqual(s['durable_frames'],12)
            g,s=reader.query([-2,-2,5],[2,2,7]);self.assertGreater(s['owned_boundary_edges_omitted'],0)
        finally:reader.close()
        reader=Reader(self.path)
        try:rich_parity(reader.query([-2,-2,5],[2,2,7])[0],crop(expected,[-2,-2,5],[2,2,7]))
        finally:reader.close()

    def test_pending_input_invisible_and_no_implicit_flush(self):
        a=self.writer();reader=Reader(self.path)
        try:
            a.update(self.depth,np.eye(4),0,0.,True)
            g,s=reader.query([-2,-2,5],[2,2,7]);self.assertEqual(len(g['points']),0);self.assertIsNone(s['durable_source'])
            self.assertEqual(a.status()['committed'],0)
            a.flush();g,s=reader.query([-2,-2,5],[2,2,7]);self.assertGreater(len(g['points']),0);self.assertEqual(s['durable_frames'],1)
            a.update(self.depth,np.eye(4),1,.1,True)
            g,s=reader.query([-2,-2,5],[2,2,7]);self.assertEqual(s['durable_frames'],1);self.assertTrue(np.all(g['seen']==0))
        finally:reader.close();a.close()

    def test_incremental_job_restores_cross_region_links_exactly(self):
        expected=self.populate();reader=Reader(self.path)
        try:
            job=RegionalJob(reader,[-2,-2,5],[2,2,7])
            with self.assertRaises(RuntimeError):job.assemble()
            while not job.ready_to_assemble:job.step()
            actual=job.assemble();rich_parity(actual,crop(expected,[-2,-2,5],[2,2,7]))
            self.assertTrue(job.complete);self.assertGreater(job.restored_boundary_edges,0)
        finally:reader.close()

    def test_incremental_job_invalidates_on_revision_or_retention_limit(self):
        self.populate();a=self.writer();reader=Reader(self.path)
        try:
            job=RegionalJob(reader,[-2,-2,5],[2,2,7]);job.step()
            a.update(self.depth,np.eye(4),20,20.,True);a.flush()
            with self.assertRaisesRegex(RuntimeError,'revision changed'):job.step()
            self.assertTrue(job.failed);self.assertFalse(job.parts)
            with self.assertRaises(RuntimeError):job.assemble()
            job=RegionalJob(reader,[-2,-2,5],[2,2,7],node_cap=1)
            with self.assertRaisesRegex(RuntimeError,'retention budget'):
                while not job.ready_to_assemble:job.step()
            self.assertFalse(job.complete)
        finally:reader.close();a.close()

    def test_budgets_fail_without_stale_result_and_recover(self):
        self.populate();reader=Reader(self.path)
        try:
            reader.query([-2,-2,5],[2,2,7])
            for options in (dict(regions=1),dict(raw_kib=1),dict(compressed_kib=1),dict(nodes=1),dict(edges=1)):
                with self.assertRaisesRegex(RuntimeError,'budget'):reader.query([-2,-2,5],[2,2,7],**options)
                with self.assertRaisesRegex(RuntimeError,'No successful'):reader.copy(0,0)
            with self.assertRaises(RuntimeError):reader.query([0,0,0],[65,1,1])
            with self.assertRaises(ValueError):reader.query([0.5,0,0],[1,1,1])
            with self.assertRaises(RuntimeError):reader.copy(0,0)
            self.assertGreater(len(reader.query([-2,-2,5],[2,2,7])[0]['points']),0)
        finally:reader.close()

    def test_missing_foreign_and_corrupt_archives_rejected(self):
        with self.assertRaises(RuntimeError):Reader(self.root/'missing.sqlite')
        self.assertFalse((self.root/'missing.sqlite').exists())
        with sqlite3.connect(self.root/'foreign.sqlite') as db:db.execute('CREATE TABLE other(x)')
        with self.assertRaises(RuntimeError):Reader(self.root/'foreign.sqlite')
        self.populate()
        with sqlite3.connect(self.path) as db:
            row=db.execute('SELECT rx,ry,rz,seq,payload FROM deltas WHERE rx=-1 AND ry=-1 AND rz=6 LIMIT 1').fetchone()
            self.assertIsNotNone(row);x,y,z,seq,payload=row;bad=bytearray(payload);bad[16]^=1
            db.execute('UPDATE deltas SET payload=? WHERE rx=? AND ry=? AND rz=? AND seq=?',(bytes(bad),x,y,z,seq))
        reader=Reader(self.path)
        try:
            with self.assertRaisesRegex(RuntimeError,'Corrupt'):reader.query([-1,-1,6],[0,0,7])
        finally:reader.close()

    def test_concurrent_commits_are_single_revision_and_readers_release(self):
        a=self.writer();a.update(self.depth,np.eye(4),0,0.,True);a.flush()
        failures=[];stop=threading.Event();observed=[]
        def consume():
            reader=Reader(self.path)
            try:
                while not stop.is_set():
                    g,s=reader.query([-2,-2,5],[2,2,7])
                    observed.append(s['durable_frames'])
                    # Every frame updates every key. Mixed revisions would violate this.
                    if not np.all(g['seen']==s['durable_source']) or not np.all(g['edge_seen']==s['durable_source']):
                        raise AssertionError('Mixed committed revisions')
                    stop.wait(.002)
            except Exception as exc:failures.append(repr(exc))
            finally:reader.close()
        thread=threading.Thread(target=consume);thread.start()
        try:
            for i in range(1,50):a.update(self.depth,np.eye(4),i,float(i),True);a.flush()
        finally:stop.set();thread.join(timeout=10);a.close()
        self.assertFalse(thread.is_alive());self.assertFalse(failures,failures);self.assertGreater(len(observed),2)
        self.assertEqual(observed,sorted(observed))
        # All reader transactions were released; a subsequent checkpoint is not busy.
        with sqlite3.connect(self.path) as db:self.assertEqual(db.execute('PRAGMA wal_checkpoint(TRUNCATE)').fetchone()[0],0)


if __name__=='__main__':unittest.main()
