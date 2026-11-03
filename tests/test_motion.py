import asyncio
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from fleet.motion import MotionRunner, parse_program, velocity


def program(targets=None, **settings):
    return parse_program(dict(steps=[{'targets': targets or {'1': {'x': .5, 'y': 0, 'yaw': 0}}}], **settings))


class ProgramTests(unittest.TestCase):
    def test_feedback_errors_distinguish_connection_alignment_and_age(self):
        valid = {'connected': True, 'aligned': True, 'age': .1,
                 'pose': {'x': 0., 'y': 0., 'yaw': 0.}}
        for update, message in [({'connected': False}, '连接不可用'),
                                ({'aligned': False}, '坐标未标定'),
                                ({'age': .713}, '0.713 秒未更新'),
                                ({'age': None}, '位置反馈时间无效')]:
            with self.subTest(update=update):
                runner = MotionRunner(lambda n: {**valid, **update}, None, None)
                with self.assertRaisesRegex(ValueError, message):
                    runner.poses([4])

    def test_rejects_missing_nonfinite_or_inconsistent_targets(self):
        for value in (None, {}, {'steps': []}, {'steps': [{'targets': {'1': {'x': 0, 'y': 0}}}]},
                      {'steps': [{'targets': {'1': {'x': float('nan'), 'y': 0, 'yaw': 0}}}]}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_program(value)
        value = program()
        value['steps'].append({'targets': {'2': {'x': 0, 'y': 0, 'yaw': 0}}})
        with self.assertRaises(ValueError):
            parse_program(value)

    def test_map_velocity_rotates_into_body_frame_and_limits_vector_norm(self):
        command, distance, _ = velocity({'x': 0, 'y': 0, 'yaw': math.pi/2},
                                        {'x': 2, 'y': 0, 'yaw': -math.pi}, .12, .35)
        self.assertAlmostEqual(command[0], 0)
        self.assertAlmostEqual(command[1], -.12)
        self.assertAlmostEqual(distance, 2)
        self.assertLessEqual(abs(command[2]), .35)


class RunnerTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.poses = {1: {'x': 0., 'y': 0., 'yaw': 0.}, 2: {'x': 0., 'y': 2., 'yaw': 0.}}
        self.commands, self.stops, self.latest = [], [], {}
        self.age, self.transform, self.clock = 0., {'x': 0}, 0.
        self.integrate = True
        self.hook = None
        self.runner = MotionRunner(self.snapshot, self.send, self.stop)

    def snapshot(self, n):
        return {'connected': True, 'aligned': True, 'age': self.age,
                'pose': dict(self.poses[n]), 'transform': dict(self.transform)}

    def send(self, n, vx, vy, wz):
        self.commands.append((n, vx, vy, wz))
        self.latest[n] = (vx, vy, wz)

    async def stop(self, n):
        self.stops.append(n)
        self.latest[n] = (0, 0, 0)

    async def run_simulation(self, value):
        real_sleep = asyncio.sleep

        async def tick(dt):
            self.clock += dt
            if self.integrate:
                for n, (vx, vy, wz) in self.latest.items():
                    p = self.poses[n]
                    c, s = math.cos(p['yaw']), math.sin(p['yaw'])
                    p['x'] += (c*vx-s*vy)*dt
                    p['y'] += (s*vx+c*vy)*dt
                    p['yaw'] += wz*dt
            if self.hook:
                self.hook()
            await real_sleep(0)

        with patch('fleet.motion.time', SimpleNamespace(monotonic=lambda: self.clock)), \
             patch('fleet.motion.asyncio.sleep', tick):
            await self.runner.start(value)
            await self.runner.task
        return self.runner.payload()

    async def test_reaches_two_targets_and_respects_speed_acceleration(self):
        value = program()
        value['steps'].append({'timeout': 25, 'targets': {1: {'x': .5, 'y': .5, 'yaw': .5}}})
        state = await self.run_simulation(value)
        self.assertEqual(state['status'], 'completed', state)
        self.assertEqual(state['step'], 2)
        self.assertLessEqual(math.hypot(self.poses[1]['x']-.5, self.poses[1]['y']-.5), .12)
        self.assertLessEqual(abs(self.poses[1]['yaw']-.5), .18)
        self.assertEqual(self.stops, [1])
        self.assertGreater(self.clock, .5)
        previous = (0, 0)
        for _, vx, vy, wz in self.commands:
            self.assertLessEqual(math.hypot(vx, vy), .120001)
            # Arrival may immediately zero velocity; moving samples obey slew limits.
            if vx or vy:
                self.assertLessEqual(math.hypot(vx-previous[0], vy-previous[1]), .020001)
            previous = (vx, vy)

    async def test_all_robots_must_arrive_before_next_step(self):
        value = program({'1': {'x': 0, 'y': 0, 'yaw': 0}, '2': {'x': .5, 'y': 2, 'yaw': 0}})
        value['steps'].append({'timeout': 25, 'targets': {1: {'x': .4, 'y': 0, 'yaw': 0},
                                                         2: {'x': .5, 'y': 2, 'yaw': 0}}})
        def check():
            if self.runner.state['step'] == 1:
                self.assertEqual(self.poses[1]['x'], 0)
            else:
                self.assertLessEqual(abs(self.poses[2]['x']-.5), .12)
        self.hook = check
        state = await self.run_simulation(value)
        self.assertEqual(state['status'], 'completed', state)
        self.assertEqual(set(self.stops), {1, 2})

    async def test_rejects_crossing_paths_before_any_movement(self):
        with self.assertRaisesRegex(ValueError, '路线距离不足'):
            await self.runner.start(program({'1': {'x': 0, 'y': 2, 'yaw': 0},
                                             '2': {'x': 0, 'y': 0, 'yaw': 0}}))
        self.assertEqual(self.commands, [])

    async def test_stale_feedback_stops_entire_group(self):
        self.hook = lambda: setattr(self, 'age', .7)
        state = await self.run_simulation(program({'1': {'x': .5, 'y': 0, 'yaw': 0},
                                                  '2': {'x': .5, 'y': 2, 'yaw': 0}}))
        self.assertEqual(state['status'], 'failed')
        self.assertIn('反馈', state['error'])
        self.assertEqual(set(self.stops), {1, 2})
        self.assertEqual(len(self.commands), 2)

    async def test_transform_change_and_pose_jump_stop(self):
        for change in ('transform', 'jump'):
            self.setUp()
            def mutate():
                if change == 'transform': self.transform['x'] = 1
                else: self.poses[1]['x'] += 1
            self.hook = mutate
            state = await self.run_simulation(program())
            self.assertEqual(state['status'], 'failed')
            self.assertIn('标定' if change == 'transform' else '跳变', state['error'])
            self.assertEqual(self.stops, [1])

    async def test_stalled_robot_times_out(self):
        self.integrate = False
        state = await self.run_simulation(program())
        self.assertEqual(state['status'], 'failed')
        self.assertIn('超时', state['error'])
        self.assertEqual(self.stops, [1])

    async def test_repeated_cancel_never_resumes_movement(self):
        await self.runner.start(program())
        await asyncio.sleep(.01)
        await asyncio.gather(self.runner.cancel(), self.runner.cancel())
        count = len(self.commands)
        await asyncio.sleep(.12)
        self.assertEqual(len(self.commands), count)
        self.assertEqual(self.runner.state['status'], 'cancelled')
        self.assertEqual(self.stops, [1])

    async def test_cancel_before_first_cycle(self):
        await self.runner.start(program())
        await self.runner.cancel()
        self.assertEqual(self.runner.state['status'], 'cancelled')
        self.assertEqual(self.commands, [])
        self.assertEqual(self.stops, [1])

    async def test_cancel_during_stop_delivery_waits_for_stop(self):
        stopping, release = asyncio.Event(), asyncio.Event()
        async def slow_stop(n):
            stopping.set()
            await release.wait()
            self.stops.append(n)
        self.runner.stop = slow_stop
        self.age = .7
        # Enter a failure after preflight, which then waits inside stop delivery.
        self.age = 0
        await self.runner.start(program())
        self.age = .7
        await stopping.wait()
        cancel = asyncio.create_task(self.runner.cancel())
        await asyncio.sleep(0)
        self.assertFalse(cancel.done())
        with self.assertRaisesRegex(ValueError, '已有闭环'):
            await self.runner.start(program())
        release.set()
        await cancel
        self.assertEqual(self.stops, [1])

    async def test_stop_failure_is_reported(self):
        async def failed_stop(n):
            raise RuntimeError('connection lost')
        self.runner.stop = failed_stop
        self.integrate = False
        state = await self.run_simulation(program())
        self.assertIn('connection lost', state['stop_error'])

    async def test_runtime_separation_stops_all(self):
        def close_gap():
            self.poses[2]['y'] -= .3
        self.hook = close_gap
        state = await self.run_simulation(program({'1': {'x': .5, 'y': 0, 'yaw': 0},
                                                  '2': {'x': .5, 'y': 2, 'yaw': 0}}))
        self.assertEqual(state['status'], 'failed')
        self.assertIn('距离不足', state['error'])
        self.assertEqual(set(self.stops), {1, 2})


if __name__ == '__main__':
    unittest.main()
