# Modification

## Requirements

Adjust data handling and skip tracking in `main.py`.

1. **Parameter tuning** — update the default value of `th` from `0.001` to `0.03`.
2. **CSV column rename** — rename `v` to `gv` in `control_log.csv` columns; add `v` (current velocity) to `marker_log.csv` columns before `wx/wy/wv`.
3. **Skip count tracking** — introduce `skip_count` in `SingleRunResult` and `skip_total` in `BatchResult` to track windows skipped due to either (a) waypoint not reached or passed before end of log, or (b) failing the isolation distance check.
4. **Reached-end detection fix** — introduce `end_found` flag so that windows where the waypoint is never reached or passed are explicitly skipped via `continue`, rather than falling through with a stale `idx_end`.
5. **`idx_end` initialisation** — change initial value of `idx_end` from `idx` to `mdf.index[-1]` (last row of the log).
6. **Batch exclusion condition fix** — include runs in batch results when they have skipped windows even if `windows_rows` is empty (condition changed from `len(windows_rows) == 0` to `len(windows_rows) == 0 and skip_count == 0`).
7. **Summary output** — append `(N skipped)` to both per-run and total summary lines.
8. **Example data update** — update `examples/` data files to match the new `marker_log.csv` column layout (add `v` column between `yaw` and `wx`).

## Basic design

- `skip_count` is incremented in two places inside the window loop:
  1. After the reached search loop, when `end_found` is `False` (waypoint not reached or passed before log ends).
  2. When `dist` is `nan` or `>= ISOLATION_DIST` (isolation check fails).
- `format_run_summary` appends `({result.skip_count} skipped)` after the valid/total fraction.
- The batch total line similarly appends `({batch.skip_total} skipped)`.
- `marker_log.csv` now includes a `v` column (current ego velocity) between `yaw` and `wx`; `control_log.csv` renames `v` to `gv` (goal velocity) to avoid ambiguity.

## Tasks

- [x] Change default `th` from `0.001` to `0.03`
- [x] Rename `v` → `gv` in `control_log.csv` column list
- [x] Add `v` column to `marker_log.csv` column list
- [x] Add `skip_count` field to `SingleRunResult`
- [x] Add `skip_total` field to `BatchResult`
- [x] Increment `skip_count` when waypoint not reached/passed (`end_found` is False)
- [x] Increment `skip_count` when isolation check fails
- [x] Introduce `end_found` flag and explicit `continue` on miss
- [x] Change `idx_end` initial value to `mdf.index[-1]`
- [x] Fix batch exclusion condition to include runs with skips
- [x] Append `(N skipped)` to per-run summary line
- [x] Append `(N skipped)` to batch total summary line
- [ ] Update `examples/` marker_log.csv files to include `v` column between `yaw` and `wx`
