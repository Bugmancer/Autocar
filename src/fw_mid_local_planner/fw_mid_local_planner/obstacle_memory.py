"""Session-local 3D obstacle memory, independent of ROS and planning algorithms."""

import itertools
import math
import numpy as np
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Point3D = Tuple[float, float, float]
VoxelKey = Tuple[int, int, int]


class ObstacleMemory:
    """Keep observed occupancy until repeated 3D rays establish free space.

    Ray endpoints must be actual returns, in the same fixed frame as the origin
    and occupied points. Missing returns and range-clipped synthetic endpoints
    are not free-space observations. The caller bounds the ray lengths/count.
    ``clear`` explicitly starts a fresh session; there is no time-based expiry.
    """

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
        self._revision = 0
        # Broad phase: distance from a neighboring center to the traversed cell.
        # The exact center-to-ray distance is checked before accepting evidence.
        reach = int(math.floor(self._clear_neighbor_radius / self._resolution + .5 + 1e-12))
        self._clear_offsets = [offset for offset in itertools.product(range(-reach, reach+1), repeat=3)
                               if any(offset) and sum(max(abs(v)-.5, 0)**2 for v in offset)
                               * self._resolution**2 <= self._clear_neighbor_radius**2 + 1e-12]

    @property
    def resolution(self):
        return self._resolution

    @property
    def overflowed(self):
        """Latched loss of capacity; only an explicit clear resets this flag."""
        return self._overflowed

    @property
    def revision(self):
        return self._revision

    def clear(self):
        self._voxels.clear()
        self._free_streaks.clear()
        self._free_stamps.clear()
        self._last_stamp = None
        self._overflowed = False
        self._revision += 1

    def snapshot(self) -> List[Tuple[float, float, float, float]]:
        """Return voxel centers and their most recent occupied observation time."""
        resolution = self._resolution
        return [((key[0] + 0.5) * resolution,
                 (key[1] + 0.5) * resolution,
                 (key[2] + 0.5) * resolution, stamp)
                for key, stamp in self._voxels.items()]

    def observe(self, origin_xyz, ray_endpoints_xyz, occupied_points_xyz,
                stamp: float, protected_endpoints_xyz=None) -> bool:
        """Process one scan; reject invalid origins and non-increasing stamps.

        Invalid individual points are ignored. Multiple rays in a scan count
        once. A positive free_confirmation_window allows unobserved scans
        between confirmations, as needed for non-repeating LiDAR scan patterns.
        Current hits and nearby return endpoints always override free evidence.
        """
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
        changed = False
        for key in hit_keys:
            if key in self._voxels or len(self._voxels) < self._max_voxels:
                self._voxels[key] = stamp
                changed = True
            elif not self._overflowed:
                self._overflowed = True
                changed = True

        free_keys: Set[VoxelKey] = set()
        candidates = self._voxels.keys() - hit_keys
        occluder_keys = hit_keys | {self._key(point) for point in protected}
        neighbor_cache = {}
        # Sparse residuals: index their neighboring cells once instead of
        # expanding every empty cell along every ray. Dense scenes use lazy caching.
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
                    # Empty cells also provide evidence. Requiring an occupied
                    # anchor leaves isolated remnants after that anchor clears.
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

        # Preserve uncertainty near all returns, including returns filtered out
        # of the obstacle-height band. A floor return is still an occluder.
        endpoint_bins: Dict[VoxelKey, List[Point3D]] = {}
        if free_keys or self._free_streaks:
            for point in itertools.chain(protected, occupied):
                endpoint_bins.setdefault(self._key(point), []).append(point)
        # Missing a voxel in a non-repeating scan is not new occupancy evidence.
        # Retain recent free confirmations; hits still cancel them immediately.
        next_streaks = {key: count for key, count in self._free_streaks.items()
                        if self._free_confirmation_window > 0 and key in candidates
                        and stamp - self._free_stamps[key] <= self._free_confirmation_window}
        next_stamps = {key: self._free_stamps[key] for key in next_streaks}
        # A return cancels pending evidence even on scans with no clearing ray.
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
                changed = True
            else:
                next_streaks[key] = streak
                next_stamps[key] = stamp
        self._free_streaks = next_streaks
        self._free_stamps = next_stamps
        if changed:
            self._revision += 1
        return True

    def _relevant_endpoints(self, origin, endpoints, candidates):
        """Conservative vectorized broad phase; full DDA still validates retained rays."""
        if not endpoints or not candidates:
            return []
        # Avoid adding a large matrix cost in dense maps; existing DDA stays bounded.
        if len(candidates) > 2400:
            return endpoints
        centers = (np.asarray(list(candidates), dtype=float)+.5)*self._resolution
        relative = centers - np.asarray(origin)
        squared = np.sum(relative*relative, axis=1)
        # Half the cell diagonal includes every exact-ray voxel intersection,
        # even with the optional neighbor clearing disabled.
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
        delta = tuple(b-a for a, b in zip(origin, endpoint))
        distance = math.hypot(*delta)
        if distance <= self._endpoint_margin:
            return False
        direction = tuple(v / distance for v in delta)
        center = tuple((k+.5)*self._resolution for k in key)
        projection = sum((p-o)*d for p, o, d in zip(center, origin, direction))
        limit = distance - self._endpoint_margin
        if blocked_at is not None:
            # End before the front face of the occluder's bounding sphere.
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
        return tuple(math.floor(value * self._inverse_resolution)
                     for value in point)

    def _ray_voxels(self, origin: Point3D, endpoint: Point3D):
        """Traverse positive-length voxel intersections before a real return."""
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
            # Advance all tied axes together so touching an edge/corner alone
            # does not claim that a ray observed the neighboring voxel as free.
            for axis in range(3):
                if abs(next_t[axis] - exits_at) <= 1e-12:
                    key[axis] += step[axis]
                    next_t[axis] += delta_t[axis]
            entered_at = exits_at

    def _near_endpoint(self, key: VoxelKey,
                       bins: Dict[VoxelKey, List[Point3D]]) -> bool:
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
