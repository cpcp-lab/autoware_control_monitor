"""Plot trajectory poses from planning_log.csv on the x-y plane."""

TRAJ_WP_STEP = 10  # plot every N-th waypoint

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np


def plot_trajectories(data_dir: str = "examples/1", ids: list[int] | None = None) -> None:
    pdf = pd.read_csv(
        f"{data_dir}/planning_log.csv",
        header=None,
        names=["id", "time", "px", "py", "yaw", "v"],
        parse_dates=["time"],
    )

    mdf = pd.read_csv(
        f"{data_dir}/marker_log.csv",
        header=None,
        names=["time", "px", "py", "yaw", "wx", "wy", "wv"],
    )
    if not mdf.empty:
        mdf["time"] = pd.to_datetime(mdf["time"])
        mdf = mdf.sort_values("time")

    if ids:
        pdf = pdf[pdf["id"].isin(ids)]

    traj_ids = pdf["id"].unique()
    if len(traj_ids) == 0:
        print(f"No trajectories found for ids: {ids}")
        return
    colors = cm.viridis(np.linspace(0, 1, len(traj_ids)))

    fig, ax = plt.subplots(figsize=(10, 8))

    for traj_id, color in zip(traj_ids, colors):
        traj = pdf[pdf["id"] == traj_id]
        ax.plot(traj["px"], traj["py"], "-", color=color, linewidth=0.8, alpha=0.6)

        # waypoints (every 10 points)
        ax.plot(traj["px"].iloc[1::TRAJ_WP_STEP], traj["py"].iloc[1::TRAJ_WP_STEP], "o", color=color, markersize=3)

        # first point: double circle
        x0, y0, yaw0 = traj["px"].iloc[0], traj["py"].iloc[0], traj["yaw"].iloc[0]
        ax.plot(x0, y0, "o", color=color, markersize=8)
        ax.plot(x0, y0, "o", color=color, markersize=4, markerfacecolor="white")

        # label to the right of the heading direction
        offset = 3.0
        rx = np.cos(yaw0 - np.pi / 2) * offset
        ry = np.sin(yaw0 - np.pi / 2) * offset
        ax.annotate(str(traj_id), (x0, y0), xytext=(x0 + rx, y0 + ry),
                    fontsize=7, color=color, ha="center", va="center")

        # print marker_log row nearest to the first point's timestamp and draw circle
        t0 = pd.Timestamp(traj["time"].iloc[0])
        print(f"id={traj_id} time={t0}")
        if mdf.empty:
            print("  marker_log: (empty)")
        else:
            idx = (mdf["time"] - t0).abs().argmin()
            m = mdf.iloc[idx]
            print(f"  marker_log: {m.to_dict()}")
            r = np.hypot(x0 - m["wx"], y0 - m["wy"])
            circle = plt.Circle((x0, y0), r, color=color, fill=False,
                                 linewidth=0.8, linestyle="--", alpha=0.5)
            ax.add_patch(circle)

    ax.set_xlabel("px")
    ax.set_ylabel("py")
    ax.set_title("Planned trajectories")
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.5, alpha=0.5)

    sm = plt.cm.ScalarMappable(cmap="viridis", norm=plt.Normalize(traj_ids.min(), traj_ids.max()))
    sm.set_array([])
    fig.colorbar(sm, ax=ax, label="Trajectory ID")

    plt.tight_layout()
    plt.savefig(f"{data_dir}/trajectories.png", dpi=150)
    print(f"Saved: {data_dir}/trajectories.png")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot planned trajectories from planning_log.csv.")
    parser.add_argument("data_dir", nargs="?", default="examples/1",
                        help="Directory containing planning_log.csv (default: examples/1)")
    parser.add_argument("--ids", type=int, nargs="+", metavar="ID", help="Trajectory IDs to plot")
    args = parser.parse_args()
    plot_trajectories(args.data_dir, args.ids)
