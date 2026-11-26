from h3215_core.trajectory import interpolate, min_jerk


def test_min_jerk_ends():
    assert min_jerk(0.0) == 0.0
    assert min_jerk(1.0) == 1.0
    assert min_jerk(0.5) == 0.5
    assert min_jerk(-1.0) == 0.0 and min_jerk(2.0) == 1.0


def test_interpolate():
    assert interpolate([0.0, 1.0], [1.0, -1.0], 0.0) == [0.0, 1.0]
    assert interpolate([0.0, 1.0], [1.0, -1.0], 1.0) == [1.0, -1.0]
    assert interpolate([0.0, 1.0], [1.0, -1.0], 0.5) == [0.5, 0.0]
