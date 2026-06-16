# Modification

## Requirements

Modify `main.py` to improve monitoring condition correctness, enrich batch-mode reporting, and improve usability. Also add `trajectory.py` as a new auxiliary script for visualizing planned trajectories from `planning_log.csv`.

0. **Trajectory visualization script** — add `trajectory.py` to plot planned trajectories from `planning_log.csv` on the x-y plane; support filtering by trajectory ID and overlay of nearest marker position at trajectory start.
1. **Monitoring condition refinement** — correct and refine `ann2`, `go_h`, `go_l`, and feasibility conditions based on inspection of the formal method.
2. **Initial-condition tracking** — detect and report windows where the initial condition is violated, and track whether a violation is unsound (reached but init failed).
3. **Batch output enrichment** — extend per-run and total summaries to include `!Is` (init-NG count) and `!Rs` (reached-NG count, with unsound sub-count).
4. **Batch parallelization** — run per-directory analysis in parallel using `ProcessPoolExecutor`; expose `--workers` to control parallelism.
5. **Terminology change** — rename `feas1`/`feas2` to `speed1`/`speed2`; reserve `feas` for the conjunction ann1 & ann2 & speed1 & speed2.
6. **Parameter default updates** — update defaults for `bb` (0.5 → 1.0), `eps` (0.1 → 0.5), `eps_v` (0.001 → 0.2).
7. **Plot improvements** — fix waypoint local-frame axis orientation (invert x-axis), add ε-ball circle to monitoring plot, add `--rows` option to plot selected windows only.

## Basic design

- `check_ann2`: add `abs()` around the outer `kappa` factor to match the formal condition.
- `check_go_h` / `check_go_l`: add `ic` flag to relax the two-step velocity check at initial conditions (only check current velocity, not next step).
- `check_speed1` (was `check_feas1`): drop the `wx1 > eps` guard (positional check moved elsewhere); keep velocity-interval check `0 ≤ vl < vh`.
- `SingleRunResult`: add `init_ng_count`, `reached_ng_count`, `unsound_count`; add `windows_init` list storing per-window initial-condition bitmask.
- `BatchResult`: add `init_ng_total`, `reached_ng_total`, `unsound_total`.
- `run_batch`: collect valid subdirs upfront, dispatch via `ProcessPoolExecutor.map`, aggregate counts including new fields.

## Tasks

- [x] Add `trajectory.py`: plot planned trajectories from `planning_log.csv`, with per-trajectory color, waypoint markers, ID labels, and nearest-marker circle overlay
- [x] Correct `check_ann2` formula (add outer `abs`)
- [x] Add `ic` mode to `check_go_h` and `check_go_l`
- [x] Rename `check_feas1` / `check_feas2` → `check_speed1` / `check_speed2`; drop `wx1` argument from `check_speed1`
- [x] Update parameter defaults: `bb=1.0`, `eps=0.5`, `eps_v=0.2`
- [x] Add `init_ng_count`, `reached_ng_count`, `unsound_count` to `SingleRunResult`
- [x] Add `windows_init` bitmask list to `SingleRunResult`
- [x] Extend batch summary output with `!Is` and `!Rs` columns
- [x] Add `init_ng_total`, `reached_ng_total`, `unsound_total` to `BatchResult`
- [x] Parallelize `run_batch` with `ProcessPoolExecutor`; add `--workers` CLI option
- [x] Add `--rows` CLI option to plot selected windows only
- [x] Fix local-frame plot: invert x-axis, add ε-ball circle, correct sign of `wy1`
- [x] Update `docs/requirements.md` and `README.md` to reflect all changes
- [x] Update tests to use renamed `check_speed1` / `check_speed2`
