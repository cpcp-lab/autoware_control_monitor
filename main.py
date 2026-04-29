import argparse
import pandas as pd
#import math
import numpy as nm
import matplotlib.pyplot as plt


def check_ann1(kappa, eps):
    return abs(kappa) * eps <= 1

def check_ann2(kappa, wx1, wy1, eps):
    return kappa * abs((wx1**2 + wy1**2 - eps**2) / 2 - wy1) < eps

def check_feas1(wx1, vl, vh):
    return wx1 > 0 and 0 <= vl and vl < vh

def check_feas2(aa, bb, th, vl, vh):
    return aa*th <= vh - vl and bb*th <= vh - vl

def check_go1(bb, aa, acc, vel, th):
    return -bb <= acc and acc <= aa and vel + acc*th >= 0

def check_go_h(kappa, eps, vel, acc, th, vh, bb, wx1, wy1):
    go_h1 = vel <= vh and vel + acc*th <= vh
    go_h2 = (1 + abs(kappa)*eps)**2 * (vel*th + (acc/2)*th**2 + ((vel + acc*th)**2 - vh**2) / (2*bb)) \
            + eps <= max(abs(wx1), abs(wy1))
    return go_h1 or go_h2

def check_go_l(kappa, eps, vel, acc, th, vl, aa, wx1, wy1):
    go_l1 = vl <= vel and vl <= vel + acc*th
    go_l2 = (1 + abs(kappa)*eps)**2 * (vel*th + (acc/2)*th**2 + (vl**2 - (vel + acc*th)**2) / (2*aa)) \
            + eps <= max(abs(wx1), abs(wy1))
    return go_l1 or go_l2


def main():
    parser = argparse.ArgumentParser(description='Monitor vehicle control logs.')
    parser.add_argument('data_dir', nargs='?', default='examples/1',
                        help='Directory containing control_log.csv and marker_log.csv (default: examples/1)')
    parser.add_argument('--window', type=int, default=100,
                        help='Sliding window size in rows (default: 100)')
    parser.add_argument('--debug', action='store_true',
                        help='Show additional debug plot (local waypoint geometry)')
    parser.add_argument('--wb', type=float, default=2.79,
                        help='Vehicle wheelbase [m] (default: 2.79)')
    parser.add_argument('--aa', type=float, default=1.0,
                        help='Upper acceleration bound (default: 1.0)')
    parser.add_argument('--bb', type=float, default=0.5,
                        help='Lower acceleration bound (default: 0.5)')
    parser.add_argument('--eps', type=float, default=0.1,
                        help='Spatial tolerance (default: 0.1)')
    parser.add_argument('--eps-v', type=float, default=0.001,
                        help='Velocity tolerance (default: 0.001)')
    parser.add_argument('--th', type=float, default=0.001,
                        help='Time step [s] (default: 0.001)')
    args = parser.parse_args()

    window = args.window
    wb = args.wb
    aa = args.aa
    bb = args.bb
    eps = args.eps
    eps_v = args.eps_v
    th = args.th

    # Load and time-synchronize logs: attach the nearest past control record to each marker row
    cdf = pd.read_csv(f'{args.data_dir}/control_log.csv', header=None, names=['time','sta','v','a'], parse_dates=['time'])
    mdf = pd.read_csv(f'{args.data_dir}/marker_log.csv', header=None, names=['time','px','py','yaw','wx','wy','wv'], parse_dates=['time'])
    mdf = pd.merge_asof(
        mdf.sort_values('time'),
        cdf.sort_values('time'),
        on='time',
        direction='backward'
    ).reset_index(drop=True)
    # Sample rows at every `window` steps
    m_range = range(0, len(mdf)-window, window)
    m_subset = mdf.iloc[m_range]

    n_plots = 3 if args.debug else 2
    _, axes = plt.subplots(1, n_plots, figsize=(4*n_plots, 4))

    # Plot 1: vehicle positions (filled) and waypoints (hollow) with steering arrows
    axes[0].set_title('Global trajectory')
    axes[0].set_xlabel('px')
    axes[0].set_ylabel('py')
    axes[0].set_aspect('equal')
    axes[0].scatter(m_subset['px'], m_subset['py'], marker='o', color='green')
    axes[0].scatter(m_subset['wx'], m_subset['wy'], marker='o', color='green', facecolor='none')
    for i, (_, row) in enumerate(m_subset.iterrows()):
        axes[0].annotate(str(i), (row.px, row.py),
                         xytext=(4, 4), textcoords='offset points', fontsize=8)
        # Line from vehicle to waypoint
        axes[0].plot([row.px, row.wx], [row.py, row.wy], '-', color='green')

        # Draw steering direction arrow
        if not pd.isna(row['sta']):
            axes[0].plot([row.px, row.px+nm.cos(row['yaw']+row['sta'])],
                         [row.py, row.py+nm.sin(row['yaw']+row['sta'])], '-', color='black')

    # Plot 2: monitoring result for each sampled window
    axes[1].set_title('Monitoring result')
    for i, (idx, row) in enumerate(m_subset.iterrows()):
        #if i != 5:
        #    continue

        # Initial state.
        wx0 = row['wx']
        wy0 = row['wy']
        wv0 = row['wv']
        # Velocity bounds around the waypoint speed
        vl = max(0, wv0 - eps_v)
        vh = wv0 + eps_v

        # Search for the root state: first row where the waypoint passes behind the vehicle
        idx_end = idx
        for j, r1 in mdf.loc[idx:].iterrows():
            wx1 = nm.cos(-r1['yaw']) * (wx0-r1['px']) - nm.sin(-r1['yaw']) * (wy0-r1['py'])
            if wx1 <= 0:
                idx_end = j
                break

        # Accumulators for averaged acceleration and conjunctive monitor results
        acc_acc = 0

        a1_acc = 1
        a2_acc = 1
        f1_acc = 1
        f2_acc = 1
        g1_acc = 1
        gh_acc = 1
        gl_acc = 1

        for j, r1 in mdf.loc[idx:idx_end].iterrows():
            # Waypoint offset in vehicle-local frame
            wx1 = nm.cos(-r1['yaw']) * (wx0-r1['px']) - nm.sin(-r1['yaw']) * (wy0-r1['py'])
            wy1 = nm.sin(-r1['yaw']) * (wx0-r1['px']) + nm.cos(-r1['yaw']) * (wy0-r1['py'])

            if pd.isna(r1['sta']):
                raise Exception('No cdf element found')

            delta = r1['sta']
            kappa = nm.tan(delta) / wb   # curvature from steering angle
            vel = r1['v']
            acc_acc += r1['a']
            acc = acc_acc / (j - idx + 1)  # running average acceleration

            # Monitoring

            ann1 = check_ann1(kappa, eps)
            ann2 = check_ann2(kappa, wx1, wy1, eps)
            feas1 = check_feas1(wx1, vl, vh)
            feas2 = check_feas2(aa, bb, th, vl, vh)
            feas = (ann1 and ann2 and feas1 and feas2)
            go1 = check_go1(bb, aa, acc, vel, th)
            go_h = check_go_h(kappa, eps, vel, acc, th, vh, bb, wx1, wy1)
            go_l = check_go_l(kappa, eps, vel, acc, th, vl, aa, wx1, wy1)

            go = go1 and go_h and go_l

            # Accumulate conjunctive results across all rows in this window
            a1_acc &= ann1
            a2_acc &= ann2
            f1_acc &= feas1
            f2_acc &= feas2
            g1_acc &= go1
            gh_acc &= go_h
            gl_acc &= go_l

            # Color-code: green if both feasible and go hold, red otherwise
            if feas and go:
                col = 'green'
            else:
                col = 'red'

            axes[1].plot([-wy1], [wx1], 'o', color=col)
            if j == idx:
                axes[1].annotate(str(i), (-wy1, wx1),
                                 xytext=(4, 4), textcoords='offset points', fontsize=8)

        print('%d: (%d&%d&%d&%d)&%d&%d&%d' % (i, a1_acc, a2_acc, f1_acc, f2_acc, g1_acc, gh_acc, gl_acc))

    axes[1].plot([0], [0], marker='+', color='black')

    # Plot 3 (debug): waypoint positions in vehicle-local frame relative to window start
    if args.debug:
        # w0 - p0
        # (w0 - p0) - (p1 - p0) = w0 - p1
        for idx, row in m_subset.iterrows():
            m_range1 = range(idx, idx+window)
            m_ss1 = mdf.iloc[m_range1]

            # Reference pose at the start of this window
            r0 = m_ss1.iloc[0]
            wx0 = r0['wx']
            wy0 = r0['wy']
            # Transform waypoint offsets into the local frame of each row's yaw
            m_ss1 = m_ss1.assign(
                    wx1 = nm.cos(-m_ss1['yaw']) * (wx0-m_ss1['px']) -
                          nm.sin(-m_ss1['yaw']) * (wy0-m_ss1['py']))
            m_ss1 = m_ss1.assign(
                    wy1 = nm.sin(-m_ss1['yaw']) * (wx0-m_ss1['px']) +
                          nm.cos(-m_ss1['yaw']) * (wy0-m_ss1['py']))

            axes[2].scatter(-m_ss1['wy1'], m_ss1['wx1'], marker='o', color='green', s=10)

        # Mark the origin (reference point)
        axes[2].scatter([0], [0], marker='+', color='green')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()

# EOF
