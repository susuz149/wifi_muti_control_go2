import math
import unittest

from app import formation_offsets, rotate_xy
from fleet.motion import MotionRunner


class FormationTests(unittest.TestCase):
    def test_line_preserves_left_right_order_and_spacing(self):
        poses = {4: {'x': -.2884, 'y': .9875}, 5: {'x': 0., 'y': -.0173}}
        offsets = formation_offsets([4, 5], 'line', 1.2, poses, -.01768)
        self.assertAlmostEqual(offsets[4]['y'], .6)
        self.assertAlmostEqual(offsets[5]['y'], -.6)
        targets = {}
        for n, offset in offsets.items():
            dx, dy = rotate_xy(offset['x'], offset['y'], -.01768)
            targets[n] = {'x': poses[4]['x']+dx, 'y': poses[4]['y']+dy}
        MotionRunner(None, None, None).check_paths(poses, targets, .5)

    def test_order_uses_reference_heading_instead_of_world_y(self):
        # Facing +Y: world -X is the robot's left, despite equal world Y.
        poses = {4: {'x': -1., 'y': 2.}, 5: {'x': 1., 'y': 2.}}
        offsets = formation_offsets([4, 5], 'line', 1.2, poses, math.pi/2)
        self.assertGreater(offsets[4]['y'], offsets[5]['y'])
        # Facing the opposite way reverses left and right.
        offsets = formation_offsets([4, 5], 'line', 1.2, poses, -math.pi/2)
        self.assertLess(offsets[4]['y'], offsets[5]['y'])

    def test_three_robots_follow_positions_instead_of_numbers(self):
        poses = {4: {'x': 0, 'y': 2}, 5: {'x': 0, 'y': -2}, 6: {'x': 0, 'y': 0}}
        offsets = formation_offsets([4, 5, 6], 'line', 1.2, poses)
        self.assertEqual([offsets[n]['y'] for n in [5, 6, 4]], [-1.2, 0., 1.2])

    def test_line_requires_all_poses(self):
        with self.assertRaises(ValueError):
            formation_offsets([4, 5], 'line', 1.2, {4: {'x': 0, 'y': 0}})

    def test_column_keeps_number_order(self):
        self.assertEqual(formation_offsets([4, 5], 'column', 1.2),
                         {4: {'x': 0., 'y': 0.}, 5: {'x': -1.2, 'y': 0.}})


if __name__ == '__main__':
    unittest.main()
