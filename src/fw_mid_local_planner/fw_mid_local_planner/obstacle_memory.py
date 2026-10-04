"""与 ROS、规划算法解耦的会话级三维障碍记忆。"""

import itertools
import math
import numpy as np
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Point3D = Tuple[float, float, float]
VoxelKey = Tuple[int, int, int]


class ObstacleMemory:
    """保存占用体素，直到三维射线提供足够次数的自由空间证据。

    射线端点必须是真实回波，且与起点、占用点使用同一个固定坐标系，单位为米。
    缺失回波或按量程截断得到的虚拟端点不能证明自由空间；调用方限制射线长度
    和数量。障碍不按时间自动过期，只有显式调用 ``clear`` 才开始新会话。
    """
    # 体素值是最近占用观测的时间戳（秒）；清空次数按扫描累计，命中会重置计数。
    # clear_confirmations 可配置为 1，因此不能把此机制描述为强制多帧确认。

    def __init__(self, resolution=0.10, max_voxels=60000,
                 clear_confirmations=3, endpoint_margin=0.15,
                 free_confirmation_window=0.0, clear_neighbor_radius=0.0):
        self._resolution = float(resolution)
        self._endpoint_margin = float(endpoint_margin)
        self._max_voxels = int(max_voxels)
        self._clear_confirmations = int(clear_confirmations)
        self._free_confirmation_window = float(free_confirmation_window)
        self._clear_neighbor_radius = float(clear_neighbor_radius)
        if (not math.isfinite(self._clear_neighbor_radius)
                or not 0 <= self._clear_neighbor_radius <= 2*self._resolution):
            raise ValueError("clear_neighbor_radius must be between zero and two voxels")
        if not math.isfinite(self._free_confirmation_window) or self._free_confirmation_window < 0:
            raise ValueError("free_confirmation_window must be finite and nonnegative")
        if not math.isfinite(self._resolution) or self._resolution <= 0.0:
            raise ValueError("resolution must be finite and positive")
        if not math.isfinite(self._endpoint_margin) or self._endpoint_margin < 0.0:
            raise ValueError("endpoint_margin must be finite and nonnegative")
        if self._max_voxels < 1 or self._clear_confirmations < 1:
            raise ValueError("max_voxels and clear_confirmations must be positive")
        self._inverse_resolution = 1.0 / self._resolution
        if not math.isfinite(self._inverse_resolution):
            raise ValueError("resolution is too small")
        self._voxels: Dict[VoxelKey, float] = {}
        self._free_streaks: Dict[VoxelKey, int] = {}
        self._free_stamps = {}
        self._last_stamp: Optional[float] = None
        self._overflowed = False
        # 先按邻居中心到射线穿过体素的距离筛选偏移；接受清空证据前，
        # 还会检查邻居中心到实际射线的距离，避免仅凭邻接关系删除障碍。
        reach = int(math.floor(self._clear_neighbor_radius / self._resolution + .5 + 1e-12))
        self._clear_offsets = [offset for offset in itertools.product(range(-reach, reach+1), repeat=3)
                               if any(offset) and sum(max(abs(v)-.5, 0)**2 for v in offset)
                               * self._resolution**2 <= self._clear_neighbor_radius**2 + 1e-12]

    @property
    def resolution(self):
        return self._resolution

    @property
    def overflowed(self):
        """容量不足标志一旦置位就保持，只有显式清空才能解除。"""
        return self._overflowed

    def clear(self):
        self._voxels.clear()
        self._free_streaks.clear()
        self._free_stamps.clear()
        self._last_stamp = None
        self._overflowed = False

    def snapshot(self) -> List[Tuple[float, float, float, float]]:
        """返回 ``(x, y, z, stamp)`` 列表，位置为体素中心，时间为最近占用观测秒数。"""
        resolution = self._resolution
        return [((key[0] + 0.5) * resolution,
                 (key[1] + 0.5) * resolution,
                 (key[2] + 0.5) * resolution, stamp)
                for key, stamp in self._voxels.items()]

    def observe(self, origin_xyz, ray_endpoints_xyz, occupied_points_xyz,
                stamp: float, protected_endpoints_xyz=None) -> bool:
        """处理一帧扫描，拒绝无效起点和不递增的时间戳。

        单个无效点会被忽略，同一帧内多条射线只算一次清空确认。正数确认时间窗
        允许两次确认之间暂时没有观测，适配非重复扫描雷达；时间窗为零时要求
        连续扫描确认。当前占用命中及附近真实回波始终优先于自由空间证据。
        """
        # 一次 observe 对应一个时间戳快照；输入无效或时间不递增时保持旧记忆。
        origin = self._point(origin_xyz)
        try:
            stamp = float(stamp)
        except (TypeError, ValueError, OverflowError):
            return False
        if (origin is None or not math.isfinite(stamp)
                or (self._last_stamp is not None and stamp <= self._last_stamp)):
            return False
        self._last_stamp = stamp

        occupied = self._valid_points(occupied_points_xyz)
        endpoints = self._valid_points(ray_endpoints_xyz)
        protected = (endpoints if protected_endpoints_xyz is None else
                     endpoints + self._valid_points(protected_endpoints_xyz))
        hit_keys = {self._key(point) for point in occupied}
        for key in hit_keys:
            if key in self._voxels or len(self._voxels) < self._max_voxels:
                self._voxels[key] = stamp
            elif not self._overflowed:
                self._overflowed = True

        free_keys: Set[VoxelKey] = set()
        candidates = self._voxels.keys() - hit_keys
        occluder_keys = hit_keys | {self._key(point) for point in protected}
        neighbor_cache = {}
        # 稀疏残留障碍预先建立邻接索引；稠密场景按需缓存，控制射线遍历开销。
        sparse_candidates = len(candidates) <= 4*len(endpoints)
        if sparse_candidates:
            for candidate in candidates:
                for dx, dy, dz in self._clear_offsets:
                    cell = (candidate[0]-dx, candidate[1]-dy, candidate[2]-dz)
                    neighbor_cache.setdefault(cell, []).append(candidate)
        if candidates:
            for endpoint in self._relevant_endpoints(origin, endpoints, candidates):
                if len(free_keys) == len(candidates):
                    break
                crossed = []
                blocked_at = None
                for key in self._ray_voxels(origin, endpoint):
                    if key in occluder_keys:
                        blocked_at = key
                        break
                    if key in candidates:
                        free_keys.add(key)
                    # 空体素同样能提供邻域清空证据，否则锚点清除后会留下孤立残留。
                    crossed.append(key)
                for key in crossed:
                    if not sparse_candidates and key not in neighbor_cache:
                        neighbor_cache[key] = [neighbor for dx, dy, dz in self._clear_offsets
                            for neighbor in [(key[0]+dx, key[1]+dy, key[2]+dz)]
                            if neighbor in candidates]
                    for neighbor in neighbor_cache.get(key, ()):
                        if (neighbor not in free_keys
                                and self._near_free_ray(neighbor, origin, endpoint, blocked_at)):
                            free_keys.add(neighbor)

        # 所有回波附近都保留不确定性，包括高度过滤掉的点；地面回波也会遮挡射线。
        endpoint_bins: Dict[VoxelKey, List[Point3D]] = {}
        if free_keys or self._free_streaks:
            for point in itertools.chain(protected, occupied):
                endpoint_bins.setdefault(self._key(point), []).append(point)
        # 非重复扫描中的漏点不能证明占用或自由；时间窗内保留已有确认，命中则取消。
        next_streaks = {key: count for key, count in self._free_streaks.items()
                        if self._free_confirmation_window > 0 and key in candidates
                        and stamp - self._free_stamps[key] <= self._free_confirmation_window}
        next_stamps = {key: self._free_stamps[key] for key in next_streaks}
        # 即使本帧没有清空射线，附近真实回波仍会取消尚未达标的清空证据。
        for key in free_keys | next_streaks.keys():
            if self._near_endpoint(key, endpoint_bins):
                next_streaks.pop(key, None)
                next_stamps.pop(key, None)
                continue
            if key not in free_keys:
                continue
            previous = next_streaks if self._free_confirmation_window > 0 else self._free_streaks
            streak = previous.get(key, 0) + 1
            if streak >= self._clear_confirmations:
                del self._voxels[key]
                next_streaks.pop(key, None)
                next_stamps.pop(key, None)
            else:
                next_streaks[key] = streak
                next_stamps[key] = stamp
        self._free_streaks = next_streaks
        self._free_stamps = next_stamps
        return True

    def _relevant_endpoints(self, origin, endpoints, candidates):
        """向量化筛掉不可能触及候选体素的射线，保留者仍需经过完整 DDA 检查。"""
        if not endpoints or not candidates:
            return []
        # 稠密场景直接使用 DDA，避免候选体素与射线矩阵占用过多内存。
        if len(candidates) > 2400:
            return endpoints
        centers = (np.asarray(list(candidates), dtype=float)+.5)*self._resolution
        relative = centers - np.asarray(origin)
        squared = np.sum(relative*relative, axis=1)
        # 半个体素对角线覆盖所有实际射线交点，即使禁用邻域清空也不会漏筛相交体素。
        radius = max(self._clear_neighbor_radius, math.sqrt(3)*self._resolution/2)
        retained = []
        for start in range(0, len(endpoints), 64):
            batch = endpoints[start:start+64]
            delta = np.asarray(batch) - np.asarray(origin)
            lengths = np.sqrt(np.sum(delta*delta, axis=1))
            directions = delta / np.maximum(lengths[:, None], 1e-12)
            projection = sum(directions[:, axis, None]*relative[None, :, axis]
                             for axis in range(3))
            radial_squared = squared[None, :] - projection*projection
            possible = ((projection >= -radius)
                        & (projection <= lengths[:, None]+radius)
                        & (radial_squared <= radius*radius+1e-9))
            retained.extend(p for p, keep in zip(batch, np.any(possible, axis=1)) if keep)
        return retained

    def _near_free_ray(self, key, origin, endpoint, blocked_at):
        """仅接受真实回波及遮挡物前方、距离射线足够近的体素中心。"""
        delta = tuple(b-a for a, b in zip(origin, endpoint))
        distance = math.hypot(*delta)
        if distance <= self._endpoint_margin:
            return False
        direction = tuple(v / distance for v in delta)
        center = tuple((k+.5)*self._resolution for k in key)
        projection = sum((p-o)*d for p, o, d in zip(center, origin, direction))
        limit = distance - self._endpoint_margin
        if blocked_at is not None:
            # 在遮挡体素包围球的前表面之前截断，额外保留端点安全距离。
            blocked_center = tuple((k+.5)*self._resolution for k in blocked_at)
            limit = min(limit, sum((p-o)*d for p, o, d in zip(blocked_center, origin, direction))
                        - math.sqrt(3)*self._resolution/2 - self._endpoint_margin)
        if not 0 < projection < limit:
            return False
        radial_squared = sum((p-o-projection*d)**2
                             for p, o, d in zip(center, origin, direction))
        return radial_squared <= self._clear_neighbor_radius**2 + 1e-12

    def _point(self, point) -> Optional[Point3D]:
        try:
            x, y, z = point
            result = (float(x), float(y), float(z))
        except (TypeError, ValueError, OverflowError):
            return None
        if not all(math.isfinite(v) and math.isfinite(v * self._inverse_resolution)
                   for v in result):
            return None
        return result

    def _valid_points(self, points: Iterable[Sequence[float]]) -> List[Point3D]:
        result = []
        for point in points:
            converted = self._point(point)
            if converted is not None:
                result.append(converted)
        return result

    def _key(self, point: Point3D) -> VoxelKey:
        # 使用向下取整保持负坐标与正坐标一致的半开体素区间。
        return tuple(math.floor(value * self._inverse_resolution)
                     for value in point)

    def _ray_voxels(self, origin: Point3D, endpoint: Point3D):
        """用 DDA 遍历真实回波前方具有正长度交段的体素，跳过起点和端点体素。"""
        delta = tuple(endpoint[i] - origin[i] for i in range(3))
        distance = math.hypot(*delta)
        if not math.isfinite(distance) or distance <= self._endpoint_margin:
            return
        limit = 1.0 - self._endpoint_margin / distance
        key = list(self._key(origin))
        origin_key = tuple(key)
        endpoint_key = self._key(endpoint)
        step = [1 if value > 0.0 else -1 if value < 0.0 else 0
                for value in delta]
        next_t = []
        delta_t = []
        for axis in range(3):
            if not step[axis]:
                next_t.append(float("inf"))
                delta_t.append(float("inf"))
                continue
            boundary = (key[axis] + (1 if step[axis] > 0 else 0)) * self._resolution
            next_t.append(max(0.0, (boundary - origin[axis]) / delta[axis]))
            delta_t.append(self._resolution / abs(delta[axis]))
        entered_at = 0.0
        while entered_at < limit:
            exits_at = min(next_t)
            current_key = tuple(key)
            if (min(exits_at, limit) > entered_at + 1e-12
                    and current_key != origin_key and current_key != endpoint_key):
                yield current_key
            if exits_at >= limit:
                break
            # 同时跨越并列轴，避免仅擦过棱或角就把相邻体素判定为自由空间。
            for axis in range(3):
                if abs(next_t[axis] - exits_at) <= 1e-12:
                    key[axis] += step[axis]
                    next_t[axis] += delta_t[axis]
            entered_at = exits_at

    def _near_endpoint(self, key: VoxelKey,
                       bins: Dict[VoxelKey, List[Point3D]]) -> bool:
        # 按端点到整个体素包围盒的距离保护边界，不仅比较体素中心。
        radius = int(math.ceil(self._endpoint_margin / self._resolution))
        margin_squared = self._endpoint_margin ** 2
        bounds = [(index * self._resolution, (index + 1) * self._resolution)
                  for index in key]
        ranges = [range(index - radius, index + radius + 1) for index in key]
        for neighbor in itertools.product(*ranges):
            for point in bins.get(neighbor, ()):
                distance_squared = sum(
                    max(low - value, 0.0, value - high) ** 2
                    for value, (low, high) in zip(point, bounds))
                if distance_squared <= margin_squared:
                    return True
        return False
