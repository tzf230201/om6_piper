import _bootstrap  # Repository-local imports; no hardware access.
import unittest
from vision_motion_profile import timing_profile


class ProfileTests(unittest.TestCase):
    def test_original_preserved_and_slow_keeps_path(self):
        source=[dict(name='sweep',duration=12.,offset_deg=[25,55,-50,0,-10,10])]
        original,hold,speed=timing_profile(source,'original')
        self.assertEqual(original,source);self.assertEqual((hold,speed),(1.,20))
        slow,hold,speed=timing_profile(source,'vision_slow')
        self.assertEqual(slow[0]['duration'],24.)
        self.assertEqual(slow[0]['offset_deg'],source[0]['offset_deg'])
        self.assertEqual((hold,speed),(3.,10))
        slow[0]['offset_deg'][0]=0
        self.assertEqual(source[0]['offset_deg'][0],25)

    def test_unknown_rejected(self):
        with self.assertRaises(ValueError):timing_profile([],'unknown')


if __name__=='__main__':unittest.main()
