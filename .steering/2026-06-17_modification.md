# Modification

## Requirements

Adjust aggregation data and aggregation method in `main.py`.

1. **Single-run summary display** — print a summary line (matching batch format) at the end of single-run mode.
2. **Refactor summary formatting** — extract a shared `format_run_summary` function used by both single-run and batch modes.
3. **Output order** — reorder per-window debug line and summary line to show `!Is` / `!Rs` before the pass/fail fraction.
4. **Aggregation scope** — exclude init-NG windows from `total_count`, `valid_count`, and `reached_ng_count`; count `unsound` only within init-OK windows.
5. **Window validity display** — append `Vok` / `!V<r>` to per-window debug line, where `<r>` is a bitmask of failed conditions.
6. **Named bit constants** — define `CODE_ANN1/2`, `CODE_SPEED1/2`, `CODE_GO1/H/L` and use them in `init_valid` and `v_mask` calculations.
7. **README update** — reflect all output format changes in `README.md`.

## Basic design

- `format_run_summary(name, result: SingleRunResult) -> str`: formats a single-line summary as `{name}: !Is: ...\t!Rs: ...(...unsound)\t{valid}/{total}`.
- `total_count` / `valid_count` / `reached_ng_count` / `unsound_count`: all scoped to init-OK windows only.
- `v_mask` bitmask for window validity uses the same `CODE_*` constants as `init_valid`, with `CODE_GO1` added.
- Per-window debug line format: `i: <init>; <reached>; (ann1&ann2&speed1&speed2)&go1&go_h&go_l; <valid>`
- Summary line format: `total(<n> windows): !Is: ...\t!Rs: ...(...unsound)\t<valid>/<total>[*]`
- `VL_LB = -0.01`: lower bound for `vl` (replaces hard zero), relaxing `check_speed1`, `check_go1`, and `vl` clamping in `run_single` by a small margin.
- `check_go_init(acc, vel, th)`: separate initial-condition check for `vel + acc*th >= 0` (stricter than `check_go1` which uses `VL_LB`); included as `CODE_GO1` bit in `init_valid`.
- All-skipped runs (no windows passed isolation check) are excluded from batch results and run count.

## Tasks

- [x] Extract `format_run_summary` and use it in batch output
- [x] Print summary line in single-run mode using `format_run_summary`
- [x] Reorder output: `!Is` / `!Rs` before fraction in summary lines
- [x] Reorder per-window debug line: init/reached before accumulators
- [x] Remove `dist` from per-window debug output
- [x] Exclude init-NG windows from `total_count` and `valid_count`
- [x] Exclude init-NG windows from `reached_ng_count`; simplify `unsound_count` condition
- [x] Add `Vok` / `!V<r>` display to per-window debug line
- [x] Define `CODE_ANN1/2`, `CODE_SPEED1/2`, `CODE_GO1/H/L` constants
- [x] Use `CODE_*` constants in `init_valid` and `v_mask` calculations
- [x] Update `README.md` to reflect current output format
- [x] Add tests for `!Is`, `!Rs`, and unsound count aggregation
- [x] Introduce `VL_LB` to slightly relax monitoring conditions
- [x] Add `check_go_init` and include `CODE_GO1` in `init_valid` bitmask
- [x] Append `*` to summary fraction when all windows pass
- [x] Skip all-skipped runs from batch results

## Segment–ball intersection for reached check (2026-06-19)

ウェイポイント到達判定（`reached` フラグ）において，ログデータのサンプリング間隔が粗い場合にウェイポイントの ε-ball を飛び越してしまう問題に対処するため，点単体での距離判定から**線分と ε-ball の交差判定**へ変更した。

### 変更箇所 (`main.py` — `run_single` 内の reached 判定ループ)

- `wx1_prev, wy1_prev` を直前ループのローカル座標として保持する。
- 2 点目以降は，前点 `(wx1_prev, wy1_prev)` から現点 `(wx1, wy1)` への線分と ε-ball の交差を判定する。
  - 交差条件：判別式 `b^2 - 4ac >= 0` かつ解 `t` の範囲 `[0, 1]` に交点が存在すること。
  - `a == 0`（前点と同一点）のときは点判定（`c <= 0`）にフォールバック。
- 最初のループ（`wx1_prev is None`）では従来通り点判定を行う。
- 速度条件 `vl <= r1['v'] <= vh` は現点での値をそのまま使用する（補間なし）。

### Tasks

- [x] Implement segment–ball intersection check in reached loop

## Tests added (2026-06-17)

Tests for `!Is` / `!Rs` / unsound count aggregation were added to `tests/test_monitoring.py`.

**`TestRunSingle`**
- `test_init_ng_count_non_negative` — `init_ng_count` is non-negative
- `test_init_ng_count_plus_total_equals_processed_windows` — `init_ng_count + total_count == len(windows_init)` (all isolation-passed windows are counted)
- `test_reached_ng_count_bounded_by_total` — `reached_ng_count <= total_count`
- `test_unsound_bounded_by_reached_ng` — `unsound_count <= reached_ng_count` (unsound is a subset of !Rs)
- `test_valid_plus_reached_ng_equals_total` — `valid_count + reached_ng_count <= total_count`

**`TestRunBatch`**
- `test_batch_init_ng_aggregation` — `init_ng_total` equals sum of per-run `init_ng_count`
- `test_batch_reached_ng_aggregation` — `reached_ng_total` equals sum of per-run `reached_ng_count`
- `test_batch_unsound_aggregation` — `unsound_total` equals sum of per-run `unsound_count`
- `test_batch_unsound_bounded_by_reached_ng` — `unsound_total <= reached_ng_total`
