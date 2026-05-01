# Inspection

## Requirements

Review the current implementation of `main.py` while reading the code, and:

- Check the correctness of each monitoring condition (`ann1`, `ann2`, `feas1`, `feas2`, `go1`, `go_h`, `go_l`) against the formal specification.
- Check the validity and appropriateness of the verification method (window selection, idx_end search, accumulation logic).
- Apply corrections where issues are found.

## Basic design

Proceed condition by condition.
For each issue found, immediately correct the implementation and add or update the relevant pytest test.

## Tasks

### Conditions
- [x] Run all tests and confirm they pass

- [x] Check the correctness of seven conditions
- [x] Examin whether the conditions are parameterized appropriately

### Verification method

- [x] idx_end search: correctness of the condition `wx1 ≤ 0` and handling when no such row is found
- [x] Accumulation logic: correctness of conjunctive (`&=`) accumulation over rows
- [x] valid judgment: correctness of `all(r['feas_go'] for r in rows)`

### Corrections

- Modified feas1 to allow wx to be slightly negative
- Modified the default parameter values
