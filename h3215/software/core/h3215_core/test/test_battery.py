from h3215_core.battery import charge


def test_full_and_empty():
    assert charge(12.6, 3) == 1.0
    assert charge(9.0, 3) == 0.0
    assert charge(13.5, 3) == 1.0  # clamped
    assert charge(6.0, 3) == 0.0


def test_monotonic():
    values = [charge(9.0 + 0.1 * k, 3) for k in range(37)]
    assert values == sorted(values)
