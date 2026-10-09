import _bootstrap  # Repository-local imports; no hardware access.
import unittest
from print_joints import joint_values


class JointTests(unittest.TestCase):
    def sample(self):
        return dict(frozen_control=dict(valid_payloads=True,joint_millidegrees=[1000,-2000,0,1234,5000,-6000]),
                    rx_age_s={k:.01 for k in ('0x2A5','0x2A6','0x2A7')},
                    rx_ids={k:10 for k in ('0x2A5','0x2A6','0x2A7')})

    def test_units_and_counter_token(self):
        self.assertEqual(joint_values(self.sample()), ([1,-2,0,1.234,5,-6],(10,10,10)))

    def test_repeated_sample_retains_same_token(self):
        self.assertEqual(joint_values(self.sample())[1],joint_values(self.sample())[1])

    def test_stale_and_network_age_fail(self):
        s=self.sample();s['rx_age_s']['0x2A6']=.151
        with self.assertRaises(ValueError):joint_values(s)
        with self.assertRaises(ValueError):joint_values(self.sample(),.15)

    def test_missing_negative_nonfinite_and_boolean_age_fail(self):
        for value in (None,-.01,float('nan'),True):
            s=self.sample();s['rx_age_s']['0x2A7']=value
            with self.assertRaises(ValueError):joint_values(s)

    def test_invalid_angles_do_not_become_zero(self):
        s=self.sample();s['frozen_control']['joint_millidegrees'][0]=True
        with self.assertRaises(ValueError):joint_values(s)


if __name__=='__main__':unittest.main()
