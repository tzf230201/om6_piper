import _bootstrap  # Repository-local imports; no hardware access.
import unittest
from review_live_scan import validate_trace,stage_at


class ReviewTests(unittest.TestCase):
    def test_trace_does_not_reintroduce_the_replaced_frame(self):
        rows=[dict(depth_number=i,depth_timestamp_ms=i*30.,host_arrival_perf_s=i*.03) for i in range(1,4)]
        updates=[dict(depth_number=i,source_depth_ms=i*30.,arrival_perf_s=i*.03,queue_age_ms=2) for i in (1,3)]
        result=dict(counts=dict(processed=2),queue_replaced=1)
        self.assertEqual(len(validate_trace(rows,updates,result)),3)
        updates[1]['source_depth_ms']+=1
        with self.assertRaisesRegex(ValueError,'association'):validate_trace(rows,updates,result)

    def test_incomplete_accounting_is_not_hidden(self):
        rows=[dict(depth_number=1,depth_timestamp_ms=1,host_arrival_perf_s=1)]
        updates=[dict(depth_number=1,source_depth_ms=1,arrival_perf_s=1,queue_age_ms=2)]
        with self.assertRaisesRegex(ValueError,'accounting'):
            validate_trace(rows,updates,dict(counts=dict(processed=1),queue_replaced=1))

    def test_phase_labels_are_causal(self):
        events=[dict(event='command_segment_begin',host_perf_s=2,name='down'),
                dict(event='command_hold_begin',host_perf_s=5,name='down'),
                dict(event='command_segment_begin',host_perf_s=7,name='return')]
        self.assertEqual(stage_at(events,1),('Startup','startup'))
        self.assertEqual(stage_at(events,6),('down','hold_command'))
        self.assertEqual(stage_at(events,7),('return','moving_command'))


if __name__=='__main__':unittest.main()
