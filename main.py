import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
import pandas as pd
import numpy as nm
import matplotlib.pyplot as plt

VL_LB = -0.01

ISOLATION_DIST = 0.5

CODE_ANN1   = 1
CODE_ANN2   = 2
CODE_SPEED1 = 4
CODE_SPEED2 = 8
CODE_GO1    = 16
CODE_GO_H   = 32
CODE_GO_L   = 64

def check_ann1(kappa, eps):
    return abs(kappa) * eps <= 1

def check_ann2(kappa, wx, wy, eps):
    #return kappa * abs((wx**2 + wy**2 - eps**2) / 2 - wy) < eps
    return abs(kappa * (wx**2 + wy**2 - eps**2) / 2 - wy) < eps

def check_speed1(vl, vh):
    return VL_LB <= vl and vl < vh

def check_speed2(aa, bb, th, vl, vh):
    return aa*th <= vh - vl and bb*th <= vh - vl

def check_go_init(acc, vel, th):
    return vel + acc*th >= 0

def check_go1(bb, aa, acc, vel, th):
    #print(acc)
    return -bb <= acc and acc <= aa and vel + acc*th >= VL_LB

def check_go_h(kappa, eps, vel, acc, th, vh, bb, wx, wy, ic=False):
    if ic:
        go_h1 = vel <= vh
    else:
        go_h1 = vel <= vh and vel + acc*th <= vh
    go_h2 = (1 + abs(kappa)*eps)**2 \
            * (vel*th + (acc/2)*th**2 + ((vel + acc*th)**2 - vh**2) / (2*bb)) \
            + eps <= \
            wx
            #max(abs(wx), abs(wy))
    return go_h1 or go_h2

def check_go_l(kappa, eps, vel, acc, th, vl, aa, wx, wy, ic=False):
    if ic:
        go_l1 = vl <= vel
    else:
        go_l1 = vl <= vel and vl <= vel + acc*th
    go_l2 = (1 + abs(kappa)*eps)**2 \
            * (vel*th + (acc/2)*th**2 + (vl**2 - (vel + acc*th)**2) / (2*aa)) \
            + eps <= \
            wx
            #max(abs(wx), abs(wy))
    return go_l1 or go_l2


@dataclass
class Params:
    window: int = 100
    wb: float = 2.79
    aa: float = 1.0
    bb: float = 1.0
    eps: float = 0.5
    eps_v: float = 0.2
    th: float = 0.001


@dataclass
class SingleRunResult:
    valid_count: int
    total_count: int
    init_ng_count: int
    reached_ng_count: int
    unsound_count: int
    mdf: object = field(repr=False)          # pd.DataFrame, for plotting
    m_subset: object = field(repr=False)     # pd.DataFrame, for plotting
    windows_rows: list = field(repr=False)   # windows_rows[i] = [{'wx1', 'wy1', 'feas_go'}, ...]
    windows_init: list = field(repr=False)   # windows_init[i] = int bitmask (0 = all satisfied; bits: CODE_ANN1, CODE_ANN2, CODE_SPEED1, CODE_SPEED2, CODE_GO_H, CODE_GO_L)


@dataclass
class BatchResult:
    runs: list       # [(subdir_name, SingleRunResult), ...]
    valid_total: int
    total: int
    init_ng_total: int
    reached_ng_total: int
    unsound_total: int


def run_single(data_dir, params, verbose=False):
    wb = params.wb
    aa = params.aa
    bb = params.bb
    eps = params.eps
    eps_v = params.eps_v
    th = params.th
    window = params.window

    cdf = pd.read_csv(f'{data_dir}/control_log.csv', header=None, names=['time','sta','v','a'], parse_dates=['time'])
    mdf = pd.read_csv(f'{data_dir}/marker_log.csv', header=None, names=['time','px','py','yaw','wx','wy','wv'], parse_dates=['time'])
    if cdf.empty:
        print(f'{data_dir}: skipped (control_log.csv is empty)')
        return None
    if mdf.empty:
        print(f'{data_dir}: skipped (marker_log.csv is empty)')
        return None
    mdf = pd.merge_asof(
        mdf.sort_values('time'),
        cdf.sort_values('time'),
        on='time',
        direction='backward'
    ).reset_index(drop=True)

    m_range = range(0, len(mdf) - window, window)
    m_subset = mdf.iloc[m_range]

    windows_rows = []
    windows_init = []
    windows_iloc = []
    valid_count = 0
    total_count = 0
    init_ng_count = 0
    reached_ng_count = 0
    unsound_count = 0

    for i, (idx, row) in enumerate(m_subset.iterrows()):
        wx0 = row['wx']
        wy0 = row['wy']
        wv0 = row['wv']
        vl = max(VL_LB, wv0 - eps_v)
        vh = wv0 + eps_v

        # Check initial conditions at window start
        wx1_init = nm.cos(-row['yaw']) * (wx0 - row['px']) - nm.sin(-row['yaw']) * (wy0 - row['py'])
        wy1_init = nm.sin(-row['yaw']) * (wx0 - row['px']) + nm.cos(-row['yaw']) * (wy0 - row['py'])

        # Search for idx_end: first row
        # where the waypoint enters the eps-ball with velocity in [vl, vh] or
        # where the waypoint passes behind the vehicle,
        idx_end = idx
        reached = 3
        wx1_prev, wy1_prev = None, None
        for j, r1 in mdf.loc[idx:].iterrows():
            wx1 = nm.cos(-r1['yaw']) * (wx0 - r1['px']) - nm.sin(-r1['yaw']) * (wy0 - r1['py'])
            wy1 = nm.sin(-r1['yaw']) * (wx0 - r1['px']) + nm.cos(-r1['yaw']) * (wy0 - r1['py'])
            # Check if the segment from previous point to current point intersects eps-ball
            if wx1_prev is not None:
                dx, dy = wx1 - wx1_prev, wy1 - wy1_prev
                # |P0 + t*d|^2 = eps^2 => |d|^2 t^2 + 2(P0·d)t + |P0|^2 - eps^2 = 0
                a = dx**2 + dy**2
                b = 2 * (wx1_prev * dx + wy1_prev * dy)
                c = wx1_prev**2 + wy1_prev**2 - eps**2
                in_ball = (a > 0 and b**2 - 4*a*c >= 0 and
                           (-b - nm.sqrt(b**2 - 4*a*c)) / (2*a) <= 1 and
                           (-b + nm.sqrt(b**2 - 4*a*c)) / (2*a) >= 0) or \
                          (a == 0 and c <= 0)
            else:
                in_ball = wx1**2 + wy1**2 <= eps**2
            if in_ball and vl <= r1['v'] <= vh:
                idx_end = j
                reached = 0
                break
            if wx1 <= 0:
                idx_end = j
                reached = (0 if wx1**2 + wy1**2 <= eps**2 else 1) \
                         | (0 if vl <= r1['v'] <= vh else 2)
                break
            wx1_prev, wy1_prev = wx1, wy1

        # Skip window if no subsequent point is found within dist < ISOLATION_DIST of (wx1_init, wy1_init)
        dist = float('nan')
        for _, r_next in mdf.loc[idx+1:idx_end].iterrows():
            wx1_next = nm.cos(-r_next['yaw']) * (wx0 - r_next['px']) - nm.sin(-r_next['yaw']) * (wy0 - r_next['py'])
            wy1_next = nm.sin(-r_next['yaw']) * (wx0 - r_next['px']) + nm.cos(-r_next['yaw']) * (wy0 - r_next['py'])
            d = nm.sqrt((wx1_next - wx1_init)**2 + (wy1_next - wy1_init)**2)
            if d > 0:
                dist = d
                break
        if nm.isnan(dist) or dist >= ISOLATION_DIST:
            if verbose:
                print('%d: skipped (dist=%.4f)' % (i, dist))
            continue

        kappa_init = nm.tan(row['sta']) / wb if not pd.isna(row['sta']) else 0.0
        acc_init = row['a'] if not pd.isna(row['a']) else 0.0
        vel_init = row['v'] if not pd.isna(row['v']) else 0.0
        init_valid = (
            (0 if check_ann1(kappa_init, eps) else CODE_ANN1)
            | (0 if check_ann2(kappa_init, wx1_init, wy1_init, eps) else CODE_ANN2)
            | (0 if check_speed1(vl, vh) else CODE_SPEED1)
            | (0 if check_speed2(aa, bb, th, vl, vh) else CODE_SPEED2)
            | (0 if check_go_init(acc_init, vel_init, th) else CODE_GO1)
            | (0 if check_go_h(kappa_init, eps, vel_init, acc_init, th, vh, bb, wx1_init, wy1_init, ic=True) else CODE_GO_H)
            | (0 if check_go_l(kappa_init, eps, vel_init, acc_init, th, vl, aa, wx1_init, wy1_init, ic=True) else CODE_GO_L)
        )
        windows_init.append(init_valid)

        acc_acc = 0
        a1_acc = 1
        a2_acc = 1
        f1_acc = 1
        f2_acc = 1
        g1_acc = 1
        gh_acc = 1
        gl_acc = 1
        rows = []

        for j, r1 in mdf.loc[idx:idx_end].iterrows():
            wx1 = nm.cos(-r1['yaw']) * (wx0 - r1['px']) - nm.sin(-r1['yaw']) * (wy0 - r1['py'])
            wy1 = nm.sin(-r1['yaw']) * (wx0 - r1['px']) + nm.cos(-r1['yaw']) * (wy0 - r1['py'])

            if pd.isna(r1['sta']):
                raise Exception('No cdf element found')

            delta = r1['sta']
            kappa = nm.tan(delta) / wb
            vel = r1['v']
            acc_acc += r1['a']
            acc = acc_acc / (j - idx + 1)

            ann1 = check_ann1(kappa, eps)
            ann2 = check_ann2(kappa, wx1, wy1, eps)
            speed1 = check_speed1(vl, vh)
            speed2 = check_speed2(aa, bb, th, vl, vh)
            feas = ann1 and ann2 and speed1 and speed2
            go1 = check_go1(bb, aa, acc, vel, th)
            go_h = check_go_h(kappa, eps, vel, acc, th, vh, bb, wx1, wy1)
            go_l = check_go_l(kappa, eps, vel, acc, th, vl, aa, wx1, wy1)
            go = go1 and go_h and go_l

            a1_acc &= ann1
            a2_acc &= ann2
            f1_acc &= speed1
            f2_acc &= speed2
            g1_acc &= go1
            gh_acc &= go_h
            gl_acc &= go_l

            rows.append({'wx1': wx1, 'wy1': wy1, 'feas_go': feas and go})

        feas_go = all(r['feas_go'] for r in rows)
        if verbose:
            v_mask = (0 if a1_acc else CODE_ANN1) | (0 if a2_acc else CODE_ANN2) \
                   | (0 if f1_acc else CODE_SPEED1) | (0 if f2_acc else CODE_SPEED2) \
                   | (0 if g1_acc else CODE_GO1) | (0 if gh_acc else CODE_GO_H) \
                   | (0 if gl_acc else CODE_GO_L)
            print('%d: %s; %s; (%d&%d&%d&%d)&%d&%d&%d; %s' % (
                i,
                'Iok' if init_valid == 0 else ('!I%d' % init_valid),
                'Rok' if reached == 0 else ('!R%d' % reached),
                a1_acc, a2_acc, f1_acc, f2_acc, g1_acc, gh_acc, gl_acc,
                'Vok' if v_mask == 0 else ('!V%d' % v_mask),
            ))
        windows_rows.append(rows)
        windows_iloc.append(i)
        if init_valid != 0:
            init_ng_count += 1
        else:
            total_count += 1
            if feas_go:
                valid_count += 1
            if reached != 0:
                reached_ng_count += 1
                if feas_go:
                    unsound_count += 1

    return SingleRunResult(
        valid_count=valid_count,
        total_count=total_count,
        init_ng_count=init_ng_count,
        reached_ng_count=reached_ng_count,
        unsound_count=unsound_count,
        mdf=mdf,
        m_subset=m_subset.iloc[windows_iloc],
        windows_rows=windows_rows,
        windows_init=windows_init,
    )


def _run_batch_single(args):
    subdir, params, verbose = args
    if verbose:
        print(f'{subdir}:')
    result = run_single(str(subdir), params, verbose=verbose)
    return (subdir.name, result)


def format_run_summary(name, result: SingleRunResult) -> str:
    perfect = result.total_count > 0 and result.valid_count == result.total_count
    return f'{name}:\t!Is: {result.init_ng_count}\t!Rs: {result.reached_ng_count}({result.unsound_count} unsound)\t{result.valid_count}/{result.total_count}{"*" if perfect else ""}'


def run_batch(parent_dir, params, verbose=False, workers=None):
    subdirs = [
        p for p in sorted(Path(parent_dir).iterdir(), key=lambda p: int(p.name) if p.name.isdigit() else float('inf'))
        if p.is_dir() and (p / 'control_log.csv').exists()
    ]

    runs = []
    valid_total = 0
    total = 0
    init_ng_total = 0
    reached_ng_total = 0
    unsound_total = 0

    with ProcessPoolExecutor(max_workers=workers) as executor:
        for name, result in executor.map(_run_batch_single, [(s, params, verbose) for s in subdirs]):
            if result is None:
                continue
            if len(result.windows_rows) == 0:
                continue
            runs.append((name, result))
            valid_total += result.valid_count
            total += result.total_count
            init_ng_total += result.init_ng_count
            reached_ng_total += result.reached_ng_count
            unsound_total += result.unsound_count

    return BatchResult(runs=runs, valid_total=valid_total, total=total,
                       init_ng_total=init_ng_total, reached_ng_total=reached_ng_total,
                       unsound_total=unsound_total)


def main():
    parser = argparse.ArgumentParser(description='Monitor vehicle control logs.')
    parser.add_argument('--data-dir', default='examples/1',
                        help='Directory containing logs, or parent directory in batch mode (default: examples/1)')
    parser.add_argument('--batch', action='store_true',
                        help='Batch mode: process all subdirectories under data_dir')
    parser.add_argument('--window', type=int, default=100,
                        help='Sliding window size in rows (default: 100)')
    parser.add_argument('--debug', action='store_true',
                        help='Show additional debug plot (local waypoint geometry)')
    parser.add_argument('--wb', type=float, default=2.79,
                        help='Vehicle wheelbase [m] (default: 2.79)')
    parser.add_argument('--aa', type=float, default=1.0,
                        help='Upper acceleration bound (default: 1.0)')
    parser.add_argument('--bb', type=float, default=1.0,
                        help='Lower acceleration bound (default: 1.0)')
    parser.add_argument('--eps', type=float, default=0.5,
                        help='Spatial tolerance (default: 0.5)')
    parser.add_argument('--eps-v', type=float, default=0.2,
                        help='Velocity tolerance (default: 0.2)')
    parser.add_argument('--rows', type=int, nargs='+', metavar='ROW',
                        help='Plot only specified row indices (0-based) in single run mode')
    parser.add_argument('--th', type=float, default=0.001,
                        help='Time step [s] (default: 0.001)')
    parser.add_argument('--workers', type=int, default=None,
                        help='Number of parallel workers in batch mode (default: number of CPUs)')
    args = parser.parse_args()

    params = Params(
        window=args.window,
        wb=args.wb,
        aa=args.aa,
        bb=args.bb,
        eps=args.eps,
        eps_v=args.eps_v,
        th=args.th,
    )

    if args.batch:
        batch = run_batch(args.data_dir, params, verbose=args.debug, workers=args.workers)
        for name, run in batch.runs:
            print(format_run_summary(name, run))
        perfect = batch.total > 0 and batch.valid_total == batch.total
        print(f'total({len(batch.runs)} runs): !Is: {batch.init_ng_total}\t!Rs: {batch.reached_ng_total}({batch.unsound_total} unsound)\t{batch.valid_total}/{batch.total}{"*" if perfect else ""}')
        return

    result = run_single(args.data_dir, params, verbose=True)
    if result is None:
        return

    print(format_run_summary(f'total({result.total_count} windows)', result))

    mdf = result.mdf
    m_subset = result.m_subset
    windows_rows = result.windows_rows
    window = params.window

    if args.rows is not None:
        row_indices = sorted(set(args.rows))
        m_subset = m_subset.iloc[row_indices]
        windows_rows = [windows_rows[i] for i in row_indices]
        plot_labels = row_indices
    else:
        plot_labels = list(range(len(windows_rows)))

    n_plots = 3 if args.debug else 2
    _, axes = plt.subplots(1, n_plots, figsize=(4*n_plots, 4))

    # Plot 1: vehicle positions (filled) and waypoints (hollow) with steering arrows
    axes[0].set_title('Global trajectory')
    axes[0].set_xlabel('X')
    axes[0].set_ylabel('Y')
    axes[0].set_aspect('equal')
    axes[0].scatter(m_subset['px'], m_subset['py'], marker='o', color='green')
    axes[0].scatter(m_subset['wx'], m_subset['wy'], marker='o', color='green', facecolor='none')
    for i, (_, row) in zip(plot_labels, m_subset.iterrows()):
        axes[0].annotate(str(i), (row.px, row.py),
                         xytext=(4, 4), textcoords='offset points', fontsize=8)
        axes[0].plot([row.px, row.wx], [row.py, row.wy], '-', color='green')
        if not pd.isna(row['sta']):
            axes[0].plot([row.px, row.px + nm.cos(row['yaw'] + row['sta'])],
                         [row.py, row.py + nm.sin(row['yaw'] + row['sta'])], '-', color='black')

    # Plot 2: monitoring result for each sampled window
    axes[1].set_title('Monitoring result')
    for i, rows in zip(plot_labels, windows_rows):
        for k, r in enumerate(rows):
            col = 'green' if r['feas_go'] else 'red'
            axes[1].plot([r['wy1']], [r['wx1']], 'o', color=col)
            if k == 0:
                axes[1].annotate(str(i), (r['wy1'], r['wx1']),
                                 xytext=(4, 4), textcoords='offset points', fontsize=8)
    axes[1].plot([0], [0], marker='+', color='black')
    eps_circle = plt.Circle((0, 0), params.eps, color='black', fill=False, linestyle='dotted', linewidth=1)
    axes[1].add_patch(eps_circle)
    axes[1].set_xlabel('y')
    axes[1].set_ylabel('x')
    axes[1].set_aspect('equal')
    axes[1].invert_xaxis()

    # Plot 3 (debug): waypoint positions in vehicle-local frame relative to window start
    if args.debug:
        for idx, row in m_subset.iterrows():
            m_range1 = range(idx, idx + window)
            m_ss1 = mdf.iloc[m_range1]
            r0 = m_ss1.iloc[0]
            wx0 = r0['wx']
            wy0 = r0['wy']
            m_ss1 = m_ss1.assign(
                wx1=nm.cos(-m_ss1['yaw']) * (wx0 - m_ss1['px']) -
                    nm.sin(-m_ss1['yaw']) * (wy0 - m_ss1['py']))
            m_ss1 = m_ss1.assign(
                wy1=nm.sin(-m_ss1['yaw']) * (wx0 - m_ss1['px']) +
                    nm.cos(-m_ss1['yaw']) * (wy0 - m_ss1['py']))
            axes[2].scatter(m_ss1['wy1'], m_ss1['wx1'], marker='o', color='green', s=10)
        axes[2].scatter([0], [0], marker='+', color='green')
        axes[2].set_aspect('equal')
        axes[2].invert_xaxis()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()

# EOF
