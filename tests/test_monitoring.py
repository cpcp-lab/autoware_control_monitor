import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from main import (
    check_ann1, check_ann2,
    check_feas1, check_feas2,
    check_go1, check_go_h, check_go_l,
)

EPS = 0.1
EPS_V = 0.001
TH = 0.001
AA = 1.0
BB = 0.5


class TestAnn1:
    def test_pass(self):
        # kappa * eps = 0.5 * 0.1 = 0.05 <= 1
        assert check_ann1(kappa=0.5, eps=EPS) is True

    def test_fail(self):
        # kappa * eps = 20 * 0.1 = 2.0 > 1
        assert check_ann1(kappa=20.0, eps=EPS) is False

    def test_boundary(self):
        # kappa * eps = 10 * 0.1 = 1.0 <= 1  (boundary, should pass)
        assert check_ann1(kappa=10.0, eps=EPS) is True


class TestFeas1:
    def test_pass(self):
        # waypoint ahead, speed bounds valid
        assert check_feas1(wx1=1.0, vl=0.5, vh=1.5) is True

    def test_fail_waypoint_behind(self):
        assert check_feas1(wx1=-0.1, vl=0.5, vh=1.5) is False

    def test_fail_vl_negative(self):
        assert check_feas1(wx1=1.0, vl=-0.1, vh=1.5) is False

    def test_fail_vl_equals_vh(self):
        assert check_feas1(wx1=1.0, vl=1.0, vh=1.0) is False


class TestFeas2:
    def test_pass(self):
        vl, vh = 0.5, 1.5  # vh - vl = 1.0 >> aa*th = 0.001
        assert check_feas2(AA, BB, TH, vl, vh) is True

    def test_fail_too_narrow(self):
        # vh - vl = 0.0001 < aa*th = 0.001
        assert check_feas2(AA, BB, TH, vl=1.0, vh=1.0001) is False


class TestGo1:
    def test_pass(self):
        # acc=0.3 in [-0.5, 1.0], vel+acc*th = 1.0 + 0.0003 >= 0
        assert check_go1(BB, AA, acc=0.3, vel=1.0, th=TH) is True

    def test_fail_acc_too_high(self):
        assert check_go1(BB, AA, acc=1.5, vel=1.0, th=TH) is False

    def test_fail_acc_too_low(self):
        assert check_go1(BB, AA, acc=-1.0, vel=1.0, th=TH) is False

    def test_fail_speed_goes_negative(self):
        # vel=0, acc=-0.3 (within bounds), but vel+acc*th = -0.0003 < 0
        assert check_go1(BB, AA, acc=-0.3, vel=0.0, th=TH) is False
