import _bootstrap  # Repository-local imports; no hardware access.
import unittest
from unittest.mock import patch,MagicMock
import tempfile
from pathlib import Path
from types import SimpleNamespace
from vision_capture import feedback_record,FramePairAudit,create_capture_directory,check_map_bind

ROOT=Path(__file__).resolve().parents[2]


class StartupTests(unittest.TestCase):
    def test_new_directory_keeps_requested_name(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp',prefix='capture-test-') as d:
            wanted=Path(d)/'nested'/'run'
            self.assertEqual(create_capture_directory(wanted),wanted.resolve())
            self.assertTrue(wanted.is_dir())

    def test_retries_preserve_recording_and_handle_same_timestamp(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp',prefix='capture-test-') as d:
            wanted=Path(d)/'run';wanted.mkdir()
            (wanted/'evidence.bin').write_bytes(b'keep this previous run')
            before=(wanted/'evidence.bin').read_bytes()
            with patch('vision_capture.datetime') as clock:
                clock.now.return_value.strftime.return_value='20261007_190000_123456'
                first=create_capture_directory(wanted)
                second=create_capture_directory(wanted)
            self.assertEqual(first.name,'run_20261007_190000_123456')
            self.assertEqual(second.name,'run_20261007_190000_123456_001')
            self.assertEqual(first.parent,wanted.parent)
            self.assertTrue(first.is_dir() and second.is_dir())
            self.assertEqual((wanted/'evidence.bin').read_bytes(),before)

    def test_existing_file_is_not_treated_as_run_directory(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp',prefix='capture-test-') as d:
            wanted=Path(d)/'run';wanted.write_bytes(b'keep')
            with self.assertRaises(NotADirectoryError):create_capture_directory(wanted)
            self.assertEqual(wanted.read_bytes(),b'keep')

    def test_bind_error_explains_missing_address_without_fallback(self):
        error=OSError('address not assigned');error.winerror=10049
        probe=MagicMock();probe.__enter__.return_value.bind.side_effect=error
        with patch('vision_capture.socket.socket',return_value=probe) as make_socket:
            with self.assertRaisesRegex(RuntimeError,'not assigned.*127.0.0.1'):
                check_map_bind('100.93.232.22',8770)
            self.assertEqual(make_socket.call_count,1)

    def test_bind_probe_closes_and_validates_port(self):
        probe=MagicMock()
        with patch('vision_capture.socket.socket',return_value=probe):
            check_map_bind('127.0.0.1',8770)
            probe.__enter__.return_value.bind.assert_called_once_with(('127.0.0.1',8770))
            probe.__exit__.assert_called_once()
            for port in (0,65536,True):
                with self.assertRaises(ValueError):check_map_bind('127.0.0.1',port)


class FeedbackTests(unittest.TestCase):
    def test_duplicate_or_regressed_pair_is_not_written_twice(self):
        audit=FramePairAudit()
        self.assertIsNotNone(audit.inspect(10,1,0.,0.,'clock','clock'))
        self.assertIsNone(audit.inspect(10,1,0.,0.,'clock','clock'))
        self.assertIsNone(audit.inspect(9,2,0.,0.,'clock','clock'))
        self.assertIsNotNone(audit.inspect(11,2,0.,0.,'clock','clock'))
        self.assertEqual(audit.rejected,2)

    def test_timing_outlier_is_retained_but_flagged(self):
        for td,tc,dd,cd in ((0.,24.,'a','a'),(0.,0.,'a','b'),(float('nan'),0.,'a','a')):
            audit=FramePairAudit();result=audit.inspect(1,1,td,tc,dd,cd)
            self.assertIsNotNone(result)
            self.assertFalse(result['timestamp_pair_within_5ms'])
            self.assertEqual(audit.timing_anomalies,1)

    def test_valid_pose_is_not_claimed_synchronized_or_fresh(self):
        r=feedback_record(SimpleNamespace(get_joint_angles=lambda:SimpleNamespace(msg=[0.]*6)))
        self.assertEqual(r['q_rad'],[0.]*6)
        self.assertFalse(r['fresh_verified']);self.assertFalse(r['exposure_synchronized'])
        self.assertGreaterEqual(r['read_end_perf_s'],r['read_begin_perf_s'])

    def test_invalid_feedback_is_missing_not_zero(self):
        for value in (None,[0]*5,[float('nan')]*6,[True]*6):
            r=feedback_record(SimpleNamespace(get_joint_angles=lambda:SimpleNamespace(msg=value)))
            self.assertIsNone(r['q_rad'])


if __name__=='__main__':unittest.main()
