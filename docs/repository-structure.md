# Repository Structure

## Directory layout

```
2026-04_monitoring/
├── CLAUDE.md                        # Claude Code guidance
├── docs/                            # Persistent project documents
│   ├── requirements.md              # Objectives, method, usage
│   └── repository-structure.md     # This file
├── .steering/                       # Work-unit steering files (one per task)
│   └── YYYY-MM-DD_<name>.md
├── main.py                          # Main monitoring script
├── tests/                           # pytest test cases
│   └── test_monitoring.py
└── examples/                        # Sample log datasets
    ├── 0/
    │   ├── control_log.csv
    │   ├── marker_log.csv
    │   ├── planning_log.csv
    │   └── vehicle_position_log.csv
    └── 1/
        ├── control_log.csv
        └── marker_log.csv
```

## File placement rules

- **Script files** — placed directly under the repository root.
- **Test files** — placed under `tests/` and named `test_*.py`.
- **Log data directories** — placed under `examples/`, named with a short integer or descriptive identifier (e.g. `0`, `1`). Each directory must contain at least `control_log.csv` and `marker_log.csv`.
- **Persistent documents** — placed under `docs/`. These define project-level requirements and structure and are updated as the project evolves.
- **Steering files** — placed under `.steering/` and named `YYYY-MM-DD_<name-of-the-unit>.md`. Creation and modification require explicit approval.

## CSV schemas

All CSV files have no header row.

| File | Columns |
|------|---------|
| `control_log.csv` | `time, sta, v, a` — timestamp, steering angle [rad], velocity [m/s], acceleration [m/s²] |
| `marker_log.csv` | `time, px, py, yaw, wx, wy, wv` — timestamp, vehicle position (x, y) [m], yaw [rad], waypoint position (x, y) [m], waypoint velocity [m/s] |
| `planning_log.csv` | `id, time, px, py, yaw, v` — path ID, timestamp, planned pose and speed (currently unused) |
| `vehicle_position_log.csv` | `time, px, py, yaw` — timestamp, vehicle pose from localization (currently unused) |

Timestamps are ISO 8601 datetime strings (e.g. `2025-12-18 14:10:09.245958`).
