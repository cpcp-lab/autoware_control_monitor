# Initial Implementation

## Requirements

Implement a single-script offline monitoring tool that:
- Loads Autoware vehicle control logs from a specified directory
- Evaluates formal monitoring conditions (annotation, feasibility, go) using a kinematic bicycle model
- Visualizes the results in three subplots
- Reports per-window pass/fail summary to stdout

## Basic design

- Single file: `main.py`
- CLI via `argparse`: positional `data_dir`, option `--window`
- Time-synchronize `control_log.csv` and `marker_log.csv` with `pd.merge_asof` before windowed sampling
- All monitoring parameters (`wb`, `aa`, `bb`, `eps`, `eps_v`, `th`) hardcoded in `main()`

## Tasks

### Project setup for Claude Code

- [x] Initialize git repository
- [x] Create `CLAUDE.md`
- [x] Create `docs/requirements.md`
- [x] Create `docs/repository-structure.md`
- [x] Create this steering document

### Implementation

- [x] Load CSVs and time-synchronize with `merge_asof`
- [x] Windowed sampling (`m_subset`)
- [x] Plot 1: global trajectory with steering arrows
- [x] Plot 2: waypoint geometry in vehicle-local frame
- [x] Plot 3: per-row monitoring result (green/red)
- [x] Monitoring conditions: `ann1`, `ann2`, `feas1`, `feas2`, `go1`, `go_h`, `go_l`
- [x] Per-window stdout summary
- [x] `main()` function with `argparse`

### Verification

- [x] Refactor monitoring conditions into testable functions
- [x] Write pytest tests for `ann1`, `feas1`, `feas2`, `go1`
- [x] Write pytest tests for `ann2`, `go_h`, `go_l`
- [x] Write pytest tests for time synchronization (`merge_asof`) and windowed sampling
- [x] Run tests and confirm all pass
