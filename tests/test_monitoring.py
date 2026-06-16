import pytest
import sys
import os
from pathlib import Path
import pandas as pd
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from main import (
    check_ann1, check_ann2,
    check_speed1, check_speed2,
    check_go1, check_go_h, check_go_l,
    Params, SingleRunResult, BatchResult,
    run_single, run_batch,
)

EXAMPLES = Path(__file__).parent.parent / 'examples'

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
        assert check_speed1(vl=0.5, vh=1.5) is True

    def test_pass_vl_zero(self):
        assert check_speed1(vl=0.0, vh=1.5) is True

    def test_fail_vl_below_zero(self):
        assert check_speed1(vl=-0.5, vh=0.5) is False

    def test_fail_vl_negative(self):
        assert check_speed1(vl=-0.1, vh=1.5) is False

    def test_fail_vl_equals_vh(self):
        assert check_speed1(vl=1.0, vh=1.0) is False


class TestFeas2:
    def test_pass(self):
        vl, vh = 0.5, 1.5  # vh - vl = 1.0 >> aa*th = 0.001
        assert check_speed2(AA, BB, TH, vl, vh) is True

    def test_fail_too_narrow(self):
        # vh - vl = 0.0001 < aa*th = 0.001
        assert check_speed2(AA, BB, TH, vl=1.0, vh=1.0001) is False


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


class TestAnn2:
    def test_pass_zero_kappa(self):
        # kappa=0 → 0 * anything = 0 < eps
        assert check_ann2(kappa=0.0, wx=1.0, wy=0.0, eps=EPS) is True

    def test_pass_small_kappa(self):
        # waypoint directly ahead (wy=0), small kappa
        assert check_ann2(kappa=0.1, wx=1.0, wy=0.0, eps=EPS) is True

    def test_fail_large_kappa(self):
        # large kappa and large waypoint distance make the product exceed eps
        assert check_ann2(kappa=5.0, wx=2.0, wy=2.0, eps=EPS) is False


class TestGoH:
    def test_pass_via_go_h1(self):
        # vel <= vh and vel+acc*th <= vh
        assert check_go_h(kappa=0.0, eps=EPS, vel=1.0, acc=0.0,
                          th=TH, vh=1.5, bb=BB, wx=1.0, wy=0.0) is True

    def test_fail_go_h1_pass_go_h2(self):
        # vel > vh (go_h1 fails), but waypoint is far enough for braking (go_h2 passes)
        # braking distance ≈ (vel^2 - vh^2)/(2*bb) = (4-2.25)/1 = 1.75; waypoint at wx=10
        assert check_go_h(kappa=0.0, eps=EPS, vel=2.0, acc=0.0,
                          th=TH, vh=1.5, bb=BB, wx=10.0, wy=0.0) is True

    def test_fail_both(self):
        # vel > vh and waypoint too close for braking
        assert check_go_h(kappa=0.0, eps=EPS, vel=2.0, acc=0.0,
                          th=TH, vh=1.5, bb=BB, wx=0.2, wy=0.0) is False


class TestGoL:
    def test_pass_via_go_l1(self):
        # vl <= vel and vl <= vel+acc*th
        assert check_go_l(kappa=0.0, eps=EPS, vel=1.0, acc=0.0,
                          th=TH, vl=0.5, aa=AA, wx=1.0, wy=0.0) is True

    def test_fail_go_l1_pass_go_l2(self):
        # vel < vl (go_l1 fails), but waypoint is far enough to accelerate (go_l2 passes)
        # acceleration distance ≈ (vl^2 - vel^2)/(2*aa) = (2.25-1)/2 = 0.625; waypoint at wx=10
        assert check_go_l(kappa=0.0, eps=EPS, vel=1.0, acc=0.0,
                          th=TH, vl=1.5, aa=AA, wx=10.0, wy=0.0) is True

    def test_fail_both(self):
        # vel < vl and waypoint too close to accelerate
        assert check_go_l(kappa=0.0, eps=EPS, vel=1.0, acc=0.0,
                          th=TH, vl=1.5, aa=AA, wx=0.2, wy=0.0) is False


class TestRunSingle:
    def test_returns_single_run_result(self):
        result = run_single(str(EXAMPLES / '0'), Params())
        assert isinstance(result, SingleRunResult)

    def test_counts_are_consistent(self):
        result = run_single(str(EXAMPLES / '0'), Params())
        assert result.total_count == len(result.windows_rows)
        assert 0 <= result.valid_count <= result.total_count

    def test_examples_2(self):
        result = run_single(str(EXAMPLES / '2'), Params())
        assert isinstance(result, SingleRunResult)
        assert result.total_count > 0

    def test_window_size_affects_count(self):
        result_100 = run_single(str(EXAMPLES / '0'), Params(window=100))
        result_200 = run_single(str(EXAMPLES / '0'), Params(window=200))
        assert result_100.total_count >= result_200.total_count


class TestRunBatch:
    def test_returns_batch_result(self):
        result = run_batch(str(EXAMPLES), Params())
        assert isinstance(result, BatchResult)

    def test_discovers_subdirs(self):
        result = run_batch(str(EXAMPLES), Params())
        names = [name for name, _ in result.runs]
        assert '0' in names
        assert '1' in names
        assert '2' in names

    def test_skips_non_log_dirs(self):
        result = run_batch(str(EXAMPLES), Params())
        names = [name for name, _ in result.runs]
        assert 'raw' not in names

    def test_aggregation_matches_runs(self):
        result = run_batch(str(EXAMPLES), Params())
        assert result.valid_total == sum(r.valid_count for _, r in result.runs)
        assert result.total == sum(r.total_count for _, r in result.runs)

    def test_valid_total_bounded(self):
        result = run_batch(str(EXAMPLES), Params())
        assert 0 <= result.valid_total <= result.total


class TestDataProcessing:
    def test_merge_asof_attaches_nearest_past_control(self):
        mdf = pd.DataFrame({
            'time': pd.to_datetime(['2025-01-01 00:00:01', '2025-01-01 00:00:03']),
            'px': [0.0, 1.0],
        })
        cdf = pd.DataFrame({
            'time': pd.to_datetime(['2025-01-01 00:00:00', '2025-01-01 00:00:02']),
            'sta': [0.1, 0.2],
        })
        merged = pd.merge_asof(
            mdf.sort_values('time'),
            cdf.sort_values('time'),
            on='time', direction='backward'
        ).reset_index(drop=True)
        # row at t=1s should get sta from t=0s (0.1)
        assert merged.loc[0, 'sta'] == pytest.approx(0.1)
        # row at t=3s should get sta from t=2s (0.2)
        assert merged.loc[1, 'sta'] == pytest.approx(0.2)

    def test_windowed_sampling_selects_correct_rows(self):
        mdf = pd.DataFrame({'v': range(10)})
        window = 3
        m_range = range(0, len(mdf) - window, window)
        m_subset = mdf.iloc[m_range]
        assert list(m_subset.index) == [0, 3, 6]
