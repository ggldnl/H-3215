"""Smooth moves between two poses."""


def min_jerk(s: float) -> float:
    """Minimum-jerk profile: 0 at s = 0, 1 at s = 1, zero speed and acceleration at both ends."""
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def interpolate(start: list[float], goal: list[float], s: float) -> list[float]:
    """The pose a fraction s of the way from start to goal, along the minimum-jerk profile."""
    k = min_jerk(s)
    return [a + k * (b - a) for a, b in zip(start, goal, strict=True)]
