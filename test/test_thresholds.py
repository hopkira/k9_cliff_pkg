from k9_cliff_pkg.cliff_detector import (CLEAR, CLIFF, UNKNOWN, WARNING, Sensor, thresholds)


def test_thresholds():
    assert thresholds(56.8, 1.66) == (66.8, 76.8)
    assert thresholds(37.4, 3.51)[0] == 47.93


def test_immediate_stop_and_three_clear():
    s = Sensor('x', 0, 47.93, 57.4)
    s.update(60, 3)
    assert s.state == CLIFF
    s.update(30, 3)
    assert s.state == CLIFF
    s.update(30, 3)
    assert s.state == CLIFF
    s.update(30, 3)
    assert s.state == CLEAR
    s.update(None, 3)
    assert s.state == UNKNOWN
