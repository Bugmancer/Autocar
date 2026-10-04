"""按米制弧长查询折线路径；不依赖 ROS，供 V3 跟踪与离线验证共用。"""

import bisect
import math

from .path_processing import PathPoint


class ArcPath:
    """连续重复点只保留一次，所有查询都使用同一条路径的累计弧长。"""

    def __init__(self, points):
        self.points = []
        self.distances = []
        self.length = 0.0
        for point in points:
            try:
                if hasattr(point, "x"):
                    x, y, yaw = float(point.x), float(point.y), float(point.yaw)
                else:
                    x, y = map(float, point)
                    yaw = 0.0
            except (TypeError, ValueError, AttributeError) as error:
                raise ValueError("路径点必须是 PathPoint 或二元坐标") from error
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError("路径坐标和航向必须是有限值")
            if self.points:
                previous = self.points[-1]
                distance = math.hypot(x - previous.x, y - previous.y)
                if distance == 0.0:
                    continue
                self.length += distance
                if not math.isfinite(self.length):
                    raise ValueError("路径累计长度超出有限数值范围")
            self.points.append(PathPoint(x, y, yaw))
            self.distances.append(self.length)
        if not self.points:
            raise ValueError("路径至少需要一个有限坐标点")

    @staticmethod
    def _finite(value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("路径查询参数必须是有限值")
        return value

    def _bound(self, s):
        return max(0.0, min(self.length, self._finite(s)))

    def sample(self, s):
        """插值位置与线段切向；顶点取下一段方向，末点取最后一段方向。"""
        s = self._bound(s)
        if len(self.points) == 1:
            point = self.points[0]
            return PathPoint(point.x, point.y, point.yaw)
        index = min(len(self.points) - 2, bisect.bisect_right(self.distances, s) - 1)
        start, end = self.points[index:index + 2]
        segment_length = self.distances[index + 1] - self.distances[index]
        ratio = (s - self.distances[index]) / segment_length
        return PathPoint(start.x + ratio * (end.x - start.x),
                         start.y + ratio * (end.y - start.y),
                         math.atan2(end.y - start.y, end.x - start.x))

    def project(self, x, y, min_s=0.0, max_s=None):
        """仅在进度窗口内找最近点；自交处距离相同时选择较早的弧长。"""
        x, y = self._finite(x), self._finite(y)
        lower = self._bound(min_s)
        upper = self.length if max_s is None else self._bound(max_s)
        if lower > upper:
            raise ValueError("投影窗口起点不能晚于终点")
        if lower == upper:
            point = self.sample(lower)
            return lower, math.hypot(x - point.x, y - point.y)
        first = max(0, bisect.bisect_right(self.distances, lower) - 1)
        last = min(len(self.points) - 2, bisect.bisect_left(self.distances, upper) - 1)
        best_s, best_distance = lower, math.inf
        for index in range(first, last + 1):
            start, end = self.points[index:index + 2]
            segment_length = self.distances[index + 1] - self.distances[index]
            ux, uy = (end.x - start.x) / segment_length, (end.y - start.y) / segment_length
            along = (x - start.x) * ux + (y - start.y) * uy
            s = max(lower, self.distances[index],
                    min(upper, self.distances[index + 1], self.distances[index] + along))
            along = s - self.distances[index]
            distance = math.hypot(x - start.x - along * ux, y - start.y - along * uy)
            # 以米为单位容忍舍入误差，防止自交点在相同距离的分支间跳变。
            if distance < best_distance - 1e-12:
                best_s, best_distance = s, distance
        return best_s, best_distance

    def curvature(self, s, spacing=0.2):
        """用三点外接圆估计带符号曲率，左转为正；端点采用单侧窗口。"""
        s = self._bound(s)
        spacing = self._finite(spacing)
        if spacing <= 0.0:
            raise ValueError("曲率采样间距必须大于零")
        if len(self.points) < 3 or self.length <= 1e-12:
            return 0.0
        spacing = min(spacing, self.length * 0.5)
        center = max(spacing, min(self.length - spacing, s))
        a, b, c = (self.sample(value) for value in
                   (center - spacing, center, center + spacing))
        ab = math.hypot(b.x - a.x, b.y - a.y)
        bc = math.hypot(c.x - b.x, c.y - b.y)
        ac = math.hypot(c.x - a.x, c.y - a.y)
        denominator = ab * bc * ac
        if denominator <= 1e-18:
            return 0.0
        cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
        return 2.0 * cross / denominator

    def peak_curvature(self, start, end, step=0.1):
        """检查等弧长采样和区间内全部顶点，避免粗采样跨过急弯。"""
        start, end = self._bound(start), self._bound(end)
        step = self._finite(step)
        if start > end or step <= 0.0:
            raise ValueError("曲率区间必须有序，采样步长必须大于零")
        samples = [start, end]
        samples.extend(self.distances[bisect.bisect_left(self.distances, start):
                                      bisect.bisect_right(self.distances, end)])
        count = int(math.ceil((end - start) / step))
        samples.extend(start + index * step for index in range(1, count))
        return max(abs(self.curvature(s)) for s in samples)
