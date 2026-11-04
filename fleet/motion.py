"""Position-feedback waypoint execution; no robot SDK or automatic motion at import."""
import asyncio
import copy
import math
import time


def number(data, key, default, low, high):
    try:
        value = float(data.get(key, default))
    except (TypeError, ValueError):
        raise ValueError(f'{key} 必须是数字') from None
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{key} 必须在 {low} 到 {high} 之间')
    return value


def parse_program(data):
    if not isinstance(data, dict):
        raise ValueError('编排必须是 JSON 对象')
    settings = {
        'speed': number(data, 'speed', .12, .03, .25),
        'yaw_rate': number(data, 'yaw_rate', .35, .05, 1),
        'acceleration': number(data, 'acceleration', .2, .05, .5),
        'position_tolerance': number(data, 'position_tolerance', .12, .05, .5),
        'yaw_tolerance': number(data, 'yaw_tolerance', .18, .05, .8),
        'settle_time': number(data, 'settle_time', .5, .3, 3),
        'min_separation': number(data, 'min_separation', .8, .5, 3),
    }
    raw_steps = data.get('steps')
    if not isinstance(raw_steps, list) or not 1 <= len(raw_steps) <= 100:
        raise ValueError('steps 必须包含 1 到 100 段目标')
    steps, participants = [], None
    for index, step in enumerate(raw_steps):
        if not isinstance(step, dict) or not isinstance(step.get('targets'), dict) or not step['targets']:
            raise ValueError(f'第 {index+1} 段需要 targets 对象')
        targets = {}
        for key, target in step['targets'].items():
            if not str(key).isdigit() or int(key) < 1 or not isinstance(target, dict):
                raise ValueError('targets 键必须是机器人编号，值必须是坐标对象')
            if int(key) in targets or not all(k in target for k in ('x', 'y', 'yaw')):
                raise ValueError('目标编号不能重复，坐标必须包含 x、y、yaw（弧度）')
            targets[int(key)] = {k: number(target, k, None, -200 if k != 'yaw' else -math.pi,
                                           200 if k != 'yaw' else math.pi) for k in ('x', 'y', 'yaw')}
        if participants is not None and set(targets) != participants:
            raise ValueError('每段必须包含相同的机器人编号；等待的机器人也应填写目标位置')
        participants = set(targets)
        duration = number(step, 'duration', 0, 0, 110)
        timeout = number(step, 'timeout', 25, 5, 120)
        if duration and duration + settings['settle_time'] >= timeout:
            raise ValueError('timeout 必须大于 duration + settle_time')
        steps.append({'targets': targets, 'timeout': timeout, 'duration': duration})
    return dict(settings, steps=steps)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def velocity(pose, target, speed, yaw_rate):
    dx, dy = target['x'] - pose['x'], target['y'] - pose['y']
    distance, heading = math.hypot(dx, dy), angle(target['yaw'] - pose['yaw'])
    # Proportional outer loop, transformed from map frame to body frame.
    scale = min(.8, speed / max(distance, 1e-9))
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    return ((c*dx+s*dy)*scale, (-s*dx+c*dy)*scale,
            max(-yaw_rate, min(yaw_rate, .8*heading))), distance, heading


def segment_distance(a, b, c, d):
    """Minimum distance between two closed 2-D segments (conservative path check)."""
    def cross(p, q, r):
        return (q['x']-p['x'])*(r['y']-p['y']) - (q['y']-p['y'])*(r['x']-p['x'])
    def point_distance(p, q, r):
        x, y = r['x']-q['x'], r['y']-q['y']
        t = max(0., min(1., ((p['x']-q['x'])*x+(p['y']-q['y'])*y)/max(x*x+y*y, 1e-18)))
        return math.hypot(p['x']-q['x']-t*x, p['y']-q['y']-t*y)
    crosses = cross(a,b,c)*cross(a,b,d) < 0 and cross(c,d,a)*cross(c,d,b) < 0
    return 0. if crosses else min(point_distance(a,c,d), point_distance(b,c,d),
                                  point_distance(c,a,b), point_distance(d,a,b))


class MotionRunner:
    def __init__(self, snapshot, send, stop):
        self.snapshot, self.send, self.stop = snapshot, send, stop
        self.task = None
        self.state = {'status': 'idle', 'step': 0, 'robots': {}, 'error': ''}
        self.participants = set()
        self._stopping = False
        self._cancel_lock = asyncio.Lock()

    @property
    def active(self):
        return self._stopping or (self.task is not None and not self.task.done())

    def payload(self):
        return copy.deepcopy(self.state)

    def poses(self, numbers):
        poses = {}
        for n in numbers:
            item = self.snapshot(n)
            if not item or not item.get('connected'):
                raise ValueError(f'#{n} 连接不可用，闭环任务已停止')
            if not item.get('aligned'):
                raise ValueError(f'#{n} 坐标未标定或标定已失效，请重新确认标定')
            age = item.get('age', math.inf)
            if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
                raise ValueError(f'#{n} 位置反馈时间无效，请检查定位数据')
            if age > .6:
                raise ValueError(f'#{n} 位置反馈已 {age:.3f} 秒未更新（上限 0.6 秒），闭环任务已停止')
            pose = item.get('pose') or {}
            if not all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k]) for k in ('x', 'y', 'yaw')):
                raise ValueError(f'#{n} 位置反馈无效')
            poses[n] = item
        return poses

    def check_paths(self, starts, targets, separation):
        numbers = sorted(targets)
        for i, n in enumerate(numbers):
            if math.hypot(targets[n]['x']-starts[n]['x'], targets[n]['y']-starts[n]['y']) > 4:
                raise ValueError(f'#{n} 单段位移超过 4 米，请增加中间目标')
            for m in numbers[i+1:]:
                if segment_distance(starts[n], targets[n], starts[m], targets[m]) < separation:
                    raise ValueError(f'#{n} 与 #{m} 的移动路线距离不足，请绕行或分段错峰')

    async def start(self, program):
        if self.active:
            raise ValueError('已有闭环任务正在执行，请先停止')
        numbers = set(program['steps'][0]['targets'])
        initial = self.poses(numbers)
        starts = {n: item['pose'] for n, item in initial.items()}
        for step in program['steps']:
            self.check_paths(starts, step['targets'], program['min_separation'])
            starts = step['targets']
        self.participants = numbers
        self.state = {'status': 'running', 'step': 0, 'total_steps': len(program['steps']), 'robots': {}, 'error': ''}
        self.task = asyncio.create_task(self._run(program, initial))
        return self.payload()

    async def cancel(self):
        async with self._cancel_lock:
            if self.active:
                self._stopping = True
                try:
                    task = self.task
                    # Do not interrupt stop delivery already running in the task's finally.
                    if self.state['status'] == 'running':
                        task.cancel()
                    await asyncio.gather(task, return_exceptions=True)
                    if self.state['status'] == 'running':
                        await self._stop_all()  # Cancelled before the first controller cycle.
                        self.state['status'] = 'cancelled'
                finally:
                    self._stopping = False
        return self.payload()

    async def _stop_all(self):
        # Stop attempts run concurrently and have a bounded wait, even after cancellation.
        results = await asyncio.gather(*(asyncio.wait_for(self.stop(n), 2.) for n in self.participants),
                                       return_exceptions=True)
        failures = [f'#{n}: {r}' for n, r in zip(self.participants, results) if isinstance(r, BaseException)]
        if failures:
            self.state['stop_error'] = '停止指令未确认：' + '; '.join(failures)

    async def _run(self, program, initial):
        sessions = {n: item.get('session') for n, item in initial.items()}
        transforms = {n: copy.deepcopy(item['transform']) for n, item in initial.items()}
        previous = {n: (0.,0.,0.) for n in initial}
        last_poses = {n: dict(item['pose']) for n, item in initial.items()}
        try:
            for index, step in enumerate(program['steps'], 1):
                self.state['step'] = index
                starts = {n: item['pose'] for n, item in self.poses(self.participants).items()}
                self.check_paths(starts, step['targets'], program['min_separation'])
                start_time = time.monotonic()
                deadline, settled = start_time+step['timeout'], None
                next_tick = start_time
                settle_sequences = None
                duration = step.get('duration', 0)
                if duration:
                    for n, target in step['targets'].items():
                        distance = math.hypot(target['x']-starts[n]['x'], target['y']-starts[n]['y'])
                        if 6*distance/duration**2 > program['acceleration'] or 6*abs(angle(target['yaw']-starts[n]['yaw']))/duration**2 > 1. or 1.5*distance/duration > program['speed'] or 1.5*abs(angle(target['yaw']-starts[n]['yaw']))/duration > program['yaw_rate']:
                            raise ValueError('同步段 duration 太短，请增加时长或缩短距离')
                previous_time = time.monotonic()-.1
                while True:
                    now = time.monotonic()
                    if now >= deadline:
                        raise ValueError(f'第 {index} 段超时，整组已停止')
                    items = self.poses(self.participants)
                    dt = min(.2, max(.001, now-previous_time))
                    previous_time = now
                    all_arrived, commands = True, {}
                    for n, item in items.items():
                        pose, target = item['pose'], step['targets'][n]
                        if item.get('session') != sessions[n] or item['transform'] != transforms[n]:
                            raise ValueError(f'#{n} 坐标标定已改变')
                        if math.hypot(pose['x']-last_poses[n]['x'], pose['y']-last_poses[n]['y']) > .4 or abs(angle(pose['yaw']-last_poses[n]['yaw'])) > .6:
                            raise ValueError(f'#{n} 定位跳变，请重新检查定位')
                        last_poses[n] = dict(pose)
                        command, distance, heading = velocity(pose, target, program['speed'], program['yaw_rate'])
                        if duration and now-start_time < duration:
                            u = max(0., min(1., (now-start_time)/duration))
                            fraction, rate = u*u*(3-2*u), 6*u*(1-u)/duration
                            origin = starts[n]
                            ref = {k: origin[k] + fraction*(target[k]-origin[k]) for k in ('x','y')}
                            ref['yaw'] = angle(origin['yaw'] + fraction*angle(target['yaw']-origin['yaw']))
                            if math.hypot(ref['x']-pose['x'], ref['y']-pose['y']) > .35:
                                raise ValueError(f'#{n} 同步轨迹跟踪误差超过 0.35 米')
                            if abs(angle(ref['yaw']-pose['yaw'])) > .5:
                                raise ValueError(f'#{n} 同步轨迹朝向误差超过 0.5 弧度')
                            correction, _, _ = velocity(pose, ref, program['speed'], program['yaw_rate'])
                            c, sn = math.cos(pose['yaw']), math.sin(pose['yaw'])
                            fx, fy = rate*(target['x']-origin['x']), rate*(target['y']-origin['y'])
                            vx, vy = correction[0]+c*fx+sn*fy, correction[1]-sn*fx+c*fy
                            cap = min(1., program['speed']/max(math.hypot(vx,vy),1e-9))
                            wz = correction[2]+rate*angle(target['yaw']-origin['yaw'])
                            command = (vx*cap, vy*cap, max(-program['yaw_rate'], min(program['yaw_rate'], wz)))
                        arrived = (not duration or now-start_time >= duration) and distance <= program['position_tolerance'] and abs(heading) <= program['yaw_tolerance']
                        all_arrived &= arrived
                        if arrived:
                            command = (0.,0.,0.)
                        else:
                            dx, dy = command[0]-previous[n][0], command[1]-previous[n][1]
                            scale = min(1., program['acceleration']*dt/max(math.hypot(dx,dy),1e-9))
                            wz_delta = max(-dt, min(dt, command[2]-previous[n][2]))
                            command = (previous[n][0]+dx*scale, previous[n][1]+dy*scale, previous[n][2]+wz_delta)
                        previous[n] = commands[n] = command
                        self.state['robots'][n] = {'pose': dict(pose), 'target': target, 'distance': distance,
                            'yaw_error': heading, 'command': dict(zip(('vx','vy','wz'), command)), 'arrived': arrived}
                    # Check every participant before publishing any command for this cycle.
                    poses = {n: item['pose'] for n, item in items.items()}
                    self.check_paths(poses, poses, program['min_separation'])
                    for n, command in commands.items():
                        self.send(n, *command)
                    if all_arrived:
                        settled = now if settled is None else settled
                        if settle_sequences is None:
                            settle_sequences = {n: item.get('sequence') for n, item in items.items()}
                        fresh = all(item.get('sequence') is None or item['sequence'] != settle_sequences[n] for n, item in items.items())
                        if now-settled >= program['settle_time'] and fresh:
                            break
                    else:
                        settled = None
                        settle_sequences = None
                    next_tick += .1
                    if next_tick < time.monotonic():
                        next_tick = time.monotonic() + .1
                    await asyncio.sleep(max(0., next_tick-time.monotonic()))
            self.state['status'] = 'completed'
        except asyncio.CancelledError:
            self.state['status'] = 'cancelled'
        except Exception as exc:
            self.state.update(status='failed', error=str(exc))
        finally:
            await self._stop_all()
            for robot in self.state['robots'].values():
                robot['command'] = {'vx': 0., 'vy': 0., 'wz': 0.}
