"""在有限距离内直行退出保守的代价地图膨胀区，始终检查真实障碍。"""

import copy
import math
from dataclasses import dataclass

from .path_processing import PathPoint


@dataclass(frozen=True)
class StartRecovery:
    """保存起点落入膨胀区时已验证的直行前缀，位置单位为米、速度为米每秒。"""
    start: tuple
    distance: float
    speed: float
    points: list
    buffered_exit: bool = True

    @property
    def end(self):
        return self.points[-1]


def forward_exit(start, checker, obstacles, occupied, is_inflated,
                 current_velocity, max_distance=1.2, *, speed, diagnostics=None):
    """检查完整前进扫掠及制动轨迹后，才返回可尝试的出口。

    不清除任何栅格，未知区域和静态占用区仍由原始栅格查询拦截；执行前，
    调用方必须先取得从出口通往目标的正常全局路径。
    """
    # 该策略只离开保守膨胀区，不清除障碍、不跳过原始栅格和动态障碍检查。
    if diagnostics is None:
        diagnostics = {}
    diagnostics['reason'] = 'invalid input or start not inflated'
    if (occupied is None or not 0 < max_distance <= 2.0
            or not math.isfinite(speed) or speed <= 0
            or not is_inflated(start[0], start[1])):
        return None
    stationary = checker.check(start, 0, 0, obstacles,
                               current_velocity=current_velocity, occupied=occupied)
    # 先验证当前位置及实测速度的停车过程，再搜索向前出口。
    if not stationary.safe:
        diagnostics.update(reason=stationary.reason, collision_point=stationary.collision_point)
        return None
    cosine, sine = math.cos(start[2]), math.sin(start[2])
    count = int(math.ceil(max_distance / 0.10))
    buffered, unbuffered = [], []
    for index in range(1, count + 1):
        distance = min(index * 0.10, max_distance)
        x, y = start[0] + distance*cosine, start[1] + distance*sine
        if is_inflated(x, y):
            continue
        has_buffer = not any(is_inflated(x + dx, y + dy)
            for dx in (-0.10, 0.0, 0.10) for dy in (-0.10, 0.0, 0.10))
        (buffered if has_buffer else unbuffered).append((distance, has_buffer))
    diagnostics.update(reason='no free exit center within search distance',
                       free_exits=len(buffered) + len(unbuffered), buffered_exits=len(buffered))
    # 优先选取周围 10 厘米均脱离膨胀区的出口；无此缓冲的备选出口
    # 仍须通过完整扫掠与制动检查，这个偏好不替代物理碰撞边界。
    for distance, has_buffer in buffered + unbuffered:
        sweep = copy.copy(checker)
        # 只改变本次恢复检查的预测时长；碰撞器仍会在前进段后追加制动段。
        sweep.prediction_time = distance / speed
        result = sweep.check(start, speed, 0, obstacles,
                             current_velocity=current_velocity, occupied=occupied)
        if not result.safe:
            diagnostics.update(reason=result.reason, collision_point=result.collision_point)
            continue
        samples = max(1, int(math.ceil(distance / checker.sample_distance)))
        points = [PathPoint(start[0] + distance*i/samples*cosine,
                            start[1] + distance*i/samples*sine, start[2])
                  for i in range(samples + 1)]
        diagnostics['reason'] = 'clear buffered exit' if has_buffer else 'clear exit without extra buffer'
        return StartRecovery(start, distance, speed, points, has_buffer)
    return None
