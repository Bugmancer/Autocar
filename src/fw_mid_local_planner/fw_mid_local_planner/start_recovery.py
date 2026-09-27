"""Bounded forward recovery from conservative costmap inflation, not obstacles."""

import copy
import math
from dataclasses import dataclass

from .path_processing import PathPoint


@dataclass(frozen=True)
class StartRecovery:
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
    """Check the entire forward sweep and braking before proposing an exit.

    No cells are cleared. Unknown/static occupied space remains blocked, and
    callers must obtain a normal global route from the exit before executing.
    """
    if diagnostics is None:
        diagnostics = {}
    diagnostics['reason'] = 'invalid input or start not inflated'
    if (occupied is None or not 0 < max_distance <= 2.0
            or not math.isfinite(speed) or speed <= 0
            or not is_inflated(start[0], start[1])):
        return None
    stationary = checker.check(start, 0, 0, obstacles,
                               current_velocity=current_velocity, occupied=occupied)
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
    # The 10 cm planning buffer is a preference, not a physical collision
    # boundary. Every fallback still gets the full swept and braking checks.
    for distance, has_buffer in buffered + unbuffered:
        sweep = copy.copy(checker)
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
