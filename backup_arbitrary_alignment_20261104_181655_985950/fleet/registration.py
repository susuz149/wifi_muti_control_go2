"""Planar LiDAR registration: global occupancy correlation followed by trimmed ICP.

Uses metre-valued native voxel points. Only estimates x/y/yaw; intended for
stationary robots on the same flat floor, with overlapping static surroundings.
"""
import math

import numpy as np


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def transform_points(points, transform):
    points = np.asarray(points, dtype=float)
    if not len(points):
        return np.empty((0, 3))
    result = points.copy()
    result[:, :2] = points[:, :2] @ rotation(transform['yaw']).T
    result += [transform['x'], transform['y'], transform.get('z', 0)]
    return result


def prepare(points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        raise ValueError('新鲜点云不足 100 点，请等待扫描')
    # Suppress the shared floor; it supplies little horizontal position evidence.
    low = np.percentile(points[:, 2], 5)
    above = points[(points[:, 2] > low + .18) & (points[:, 2] < low + 2.8)]
    if len(above) < 100:
        raise ValueError('点云缺少墙面或柱子等立体特征，无法自动对齐')
    xy = np.unique(np.round(above[:, :2] / .10).astype(int), axis=0) * .10
    if len(xy) < 60 or np.linalg.eigvalsh(np.cov(xy.T))[0] < .025:
        raise ValueError('环境特征过少或近似直线，请补充扫描或使用备用手动标定')
    if len(xy) > 1200:
        xy = xy[np.linspace(0, len(xy) - 1, 1200, dtype=int)]
    return xy


def nearest(source, target):
    indices, distances = [], []
    for offset in range(0, len(source), 128):
        delta = source[offset:offset + 128, None, :] - target[None, :, :]
        squared = np.einsum('ijk,ijk->ij', delta, delta)
        index = np.argmin(squared, axis=1)
        indices.append(index)
        distances.append(np.sqrt(squared[np.arange(len(index)), index]))
    return np.concatenate(indices), np.concatenate(distances)


def metrics(source, target, yaw, translation):
    moved = source @ rotation(yaw).T + translation
    _, distance = nearest(moved, target)
    _, reverse = nearest(target, moved)
    overlap = min(float(np.mean(distance < .20)), float(np.mean(reverse < .20)))
    inside = distance[distance < .20]
    rmse = float(np.sqrt(np.mean(inside ** 2))) if len(inside) else 1.0
    return overlap, rmse


def refine(source, target, yaw, translation, bounds=None, local_pose=None):
    translation = np.asarray(translation, dtype=float)
    for _ in range(22):
        moved = source @ rotation(yaw).T + translation
        index, distance = nearest(moved, target)
        cutoff = min(.55, max(.12, float(np.quantile(distance, .75))))
        keep = distance < cutoff
        if keep.sum() < 30:
            break
        a, b = source[keep], target[index[keep]]
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - ac).T @ (b - bc))
        r = vt.T @ u.T
        if np.linalg.det(r) < 0:
            vt[-1] *= -1
            r = vt.T @ u.T
        new_yaw = math.atan2(r[1, 0], r[0, 0])
        new_translation = bc - r @ ac
        if bounds is not None:
            # Bound the ROBOT position and heading, not the local-frame origin.
            heading = math.atan2(math.sin(new_yaw + local_pose['yaw']), math.cos(new_yaw + local_pose['yaw']))
            new_yaw = float(np.clip(heading, -bounds['yaw_limit'], bounds['yaw_limit'])) - local_pose['yaw']
            new_r = rotation(new_yaw)
            body = new_r @ np.array([local_pose['x'], local_pose['y']]) + new_translation
            body[0] = np.clip(body[0], -bounds['x_limit'], bounds['x_limit'])
            body[1] = np.clip(body[1], bounds['y_min'], bounds['y_max'])
            new_translation = body - new_r @ np.array([local_pose['x'], local_pose['y']])
        change = np.linalg.norm(new_translation - translation) + abs(math.atan2(math.sin(new_yaw-yaw), math.cos(new_yaw-yaw)))
        yaw, translation = new_yaw, new_translation
        if change < .0005:
            break
    overlap, rmse = metrics(source, target, yaw, translation)
    return {'yaw': yaw, 'translation': translation, 'overlap': overlap, 'rmse': rmse,
            'score': overlap - rmse}


def layout_bounds(rank, min_spacing=.5, max_spacing=.8):
    near, far = abs(rank)*min_spacing, abs(rank)*max_spacing
    return {'rank': rank, 'y_min': near if rank > 0 else -far,
            'y_max': far if rank > 0 else -near, 'x_limit': .4,
            'yaw_limit': math.radians(20), 'min_spacing': min_spacing, 'max_spacing': max_spacing}


def layout_position(transform, local_pose):
    xy = rotation(transform['yaw']) @ np.array([local_pose['x'], local_pose['y']]) + [transform['x'], transform['y']]
    yaw = math.atan2(math.sin(transform['yaw'] + local_pose['yaw']), math.cos(transform['yaw'] + local_pose['yaw']))
    return {'x': float(xy[0]), 'y': float(xy[1]), 'yaw': yaw}


def within_layout(transform, local_pose, bounds):
    pose = layout_position(transform, local_pose)
    return bool(abs(pose['x']) <= bounds['x_limit'] + 1e-6 and
                bounds['y_min'] - 1e-6 <= pose['y'] <= bounds['y_max'] + 1e-6 and
                abs(pose['yaw']) <= bounds['yaw_limit'] + 1e-6)


def row_conflicts(results, adjustments=None):
    """Check pairwise spacing/order, including gaps where a robot is unavailable."""
    adjustments = adjustments or {}
    included = [r for r in results if r.get('bounds') and
                (r.get('accepted') or str(r['number']) in adjustments) and r.get('transform')]
    conflicts = set()
    for index, a in enumerate(included):
        for b in included[index + 1:]:
            ra, rb = a['bounds']['rank'], b['bounds']['rank']
            delta = rb - ra
            if not delta:
                continue
            pa = layout_position(adjustments.get(str(a['number']), a['transform']), a['local_pose'])
            pb = layout_position(adjustments.get(str(b['number']), b['transform']), b['local_pose'])
            spacing = (pb['y'] - pa['y']) / delta
            if not a['bounds']['min_spacing'] - .03 <= spacing <= a['bounds']['max_spacing'] + .03:
                conflicts.update([a['number'], b['number']])
    return conflicts


def register_planar_layout(source_points, target_points, local_pose, bounds):
    source, target = prepare(source_points), prepare(target_points)
    results = []
    local_xy = np.array([local_pose['x'], local_pose['y']])
    for heading in [-.12, 0, .12]:
        yaw = heading - local_pose['yaw']
        for x in [-.2, 0, .2]:
            for y in np.linspace(bounds['y_min'], bounds['y_max'], 3):
                translation = np.array([x, y]) - rotation(yaw) @ local_xy
                results.append(refine(source, target, yaw, translation, bounds, local_pose))
    results.sort(key=lambda item: item['score'], reverse=True)
    best = results[0]
    ambiguous = any(item['score'] > best['score'] - .025 and
                    (np.linalg.norm(item['translation'] - best['translation']) > .25 or
                     abs(math.atan2(math.sin(item['yaw']-best['yaw']), math.cos(item['yaw']-best['yaw']))) > .15)
                    for item in results[1:])
    accepted = best['overlap'] >= .55 and best['rmse'] <= .10 and not ambiguous
    return {'transform': {'x': float(best['translation'][0]), 'y': float(best['translation'][1]),
                          'z': 0.0, 'yaw': float(best['yaw'])},
            'bounds': bounds, 'overlap': round(best['overlap'], 3), 'rmse': round(best['rmse'], 3),
            'accepted': bool(accepted), 'ambiguous': bool(ambiguous),
            'reason': '布局约束匹配通过，请检查预览' if accepted else
                      ('布局范围内仍有多个相似匹配，请检查' if ambiguous else '布局范围内点云匹配不足，未通过')}


def register_planar(source_points, target_points):
    source, target = prepare(source_points), prepare(target_points)
    sc, tc = np.median(source, axis=0), np.median(target, axis=0)
    source_centered, target_centered = source - sc, target - tc
    radius = max(np.linalg.norm(source_centered, axis=1).max(), np.linalg.norm(target_centered, axis=1).max())
    # Full zero-padded correlations avoid circular translation wraparound.
    cell = max(.15, radius / 90)
    side = int(math.ceil(radius / cell)) * 2 + 7
    size = 1 << (2 * side - 1).bit_length()
    center = side // 2

    def grid(points):
        result = np.zeros((size, size))
        ij = np.round(points / cell).astype(int) + center
        result[ij[:, 0], ij[:, 1]] = 1
        return result

    target_fft = np.fft.rfft2(grid(target_centered))
    candidates = []
    for yaw in np.arange(-math.pi, math.pi, math.pi / 36):
        rotated = source_centered @ rotation(yaw).T
        correlation = np.fft.irfft2(target_fft * np.conj(np.fft.rfft2(grid(rotated))), s=(size, size))
        for _ in range(2):
            peak = np.unravel_index(np.argmax(correlation), correlation.shape)
            value = float(correlation[peak])
            shift = np.array(peak, dtype=float)
            shift[shift > size // 2] -= size
            translation = tc - rotation(yaw) @ sc + shift * cell
            candidates.append((value, yaw, translation))
            # Keep a second spatially distinct hypothesis.
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    correlation[(peak[0]+dx) % size, (peak[1]+dy) % size] = -1
    candidates.sort(key=lambda item: item[0], reverse=True)
    diverse = []
    for _, yaw, translation in candidates:
        if any(abs(math.atan2(math.sin(yaw-y), math.cos(yaw-y))) < .22 and np.linalg.norm(translation-t) < .6 for y, t in diverse):
            continue
        diverse.append((yaw, translation))
        if len(diverse) >= 6:
            break
    results = [refine(source, target, yaw, translation) for yaw, translation in diverse]
    results.sort(key=lambda item: item['score'], reverse=True)
    best = results[0]
    ambiguous = any(
        item['score'] > best['score'] - .04 and
        (np.linalg.norm(item['translation'] - best['translation']) > .5 or
         abs(math.atan2(math.sin(item['yaw']-best['yaw']), math.cos(item['yaw']-best['yaw']))) > .25)
        for item in results[1:]
    )
    accepted = best['overlap'] >= .50 and best['rmse'] <= .13 and not ambiguous
    return {
        'transform': {'x': float(best['translation'][0]), 'y': float(best['translation'][1]), 'z': 0.0, 'yaw': float(best['yaw'])},
        'overlap': round(best['overlap'], 3), 'rmse': round(best['rmse'], 3),
        'accepted': bool(accepted), 'ambiguous': bool(ambiguous),
        'reason': '匹配通过，请检查预览' if accepted else ('存在多个相似匹配，请图上微调' if ambiguous else '重叠或匹配精度不足，请图上微调'),
    }
