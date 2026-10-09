import _bootstrap  # Repository-local imports; no hardware access.
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from table_scan import configuration,make_segments,sample,camera_world_pose,validate_start,load_for_execution,ROOT
from live_mapping import LatestSlot,pose_admission,LiveMapper
from vision_capture import JointCacheAudit

Q=np.deg2rad([9.8,1.1,-1.05,-10,10,2])
INTR=dict(width=64,height=48,fx=60.,fy=60.,cx=32.,cy=24.,coeffs=[0.]*5)


class TableTests(unittest.TestCase):
    def test_path_downward_bounded_and_returns(self):
        model,_,config=configuration();segments=make_segments(Q);rows=sample(Q,segments,hz=10)
        for row in rows:
            q=validate_start(row['q_rad'])
            if row['scan']:
                T=camera_world_pose(model,q,config)
                self.assertLess(T[2,2],-.81)
        np.testing.assert_allclose(rows[-1]['q_rad'],Q,atol=1e-14)
        q=np.array([r['q_rad'] for r in rows]);v=np.rad2deg(np.diff(q,axis=0)*10)
        self.assertLess(np.max(np.abs(v)),4.001)

    def test_no_clamp_or_new_start(self):
        for q in ([0]*5,[np.nan]*6,[0,-.1,0,0,0,0]):
            with self.assertRaises(ValueError):validate_start(q)
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp',prefix='table-test-') as directory:
            p=Path(directory)/'plan.json'
            p.write_text(json.dumps(dict(kind='offline_table_scan_candidate',schema=1,q0_rad=Q.tolist())))
            with self.assertRaisesRegex(ValueError,'differs'):load_for_execution(p,Q+[.1,0,0,0,0,0])


class LiveTests(unittest.TestCase):
    def test_bounded_latest_wins(self):
        slot=LatestSlot()
        for i in range(100):slot.put(i)
        self.assertEqual(slot.get(),99);self.assertEqual(slot.dropped,99)
        self.assertIsNone(slot.get(.001))

    def test_admission_rejects_stale_missing_and_future(self):
        r=dict(host_arrival_perf_s=10,host_arrival_wall_s=100,depth_timestamp_ms=99980,
               depth_domain='timestamp_domain.system_time',joints_after=dict(q_rad=[0]*6,cache_updates_recent=True))
        self.assertIsNone(pose_admission(r,10.01))
        self.assertEqual(pose_admission(r,10.2),'frame_queue_age')
        self.assertEqual(pose_admission({**r,'depth_timestamp_ms':101000},10.01),'depth_timestamp_age')
        r['joints_after']['cache_updates_recent']=False
        self.assertEqual(pose_admission(r,10.01),'joint_cache_not_updating')

    def test_one_frozen_pair_not_hidden_by_joint56(self):
        parser=NS()
        for name,ids in [('joint_12',(1,2)),('joint_34',(3,4)),('joint_56',(5,6))]:
            setattr(parser,name,NS(timestamp=1.,msg=NS(**{'joint_'+str(i):0. for i in ids})))
        robot=NS(_parser=parser);audit=JointCacheAudit()
        self.assertFalse(audit.inspect(robot,10)['cache_updates_recent'])
        for item in vars(parser).values():item.timestamp=2.
        self.assertTrue(audit.inspect(robot,10.02)['cache_updates_recent'])
        parser.joint_56.timestamp=3.
        self.assertFalse(audit.inspect(robot,10.2)['cache_updates_recent'])

    def test_worker_and_server_no_hardware(self):
        from urllib.request import urlopen
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp',prefix='map-test-') as directory:
            mapper=LiveMapper(INTR,Path(directory)/'map',port=0,replay=True)
            try:
                now=time.perf_counter()
                mapper.submit(np.full((48,64),.4),np.zeros((48,64,3),np.uint8),
                    dict(host_arrival_perf_s=now,depth_number=1,depth_timestamp_ms=1,joints_after=dict(q_rad=Q.tolist())))
                until=time.perf_counter()+4
                while not mapper.jpeg and not mapper.error and time.perf_counter()<until:time.sleep(.01)
                self.assertIsNone(mapper.error)
                self.assertEqual(mapper.state()['counts']['processed'],1)
                with urlopen(mapper.url+'state') as r:state=json.load(r)
                self.assertEqual(state['mode'],'recorded_replay')
                with urlopen(mapper.url+'view.jpg') as r:self.assertTrue(r.read().startswith(b'\xff\xd8'))
                time.sleep(.55);self.assertTrue(mapper.state()['stale'])
            finally:mapper.close()
            self.assertEqual(json.loads((Path(directory)/'map/result.json').read_text())['hardware_commands_sent'],0)


if __name__=='__main__':unittest.main()
