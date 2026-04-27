# Timing Optimization Log

This file tracks timing experiments for the HIL plant loop (`SAMPLE_TIME_MS = 50 ms`).

## Baseline and experiments

| Date | Change | Result | Notes |
|---|---|---|---|
| 2026-04-26 | Baseline synchronous capture in control loop (`CAPTURE_TIMEOUT_MS=60`, capture left+right every cycle) | `exec ~= 119-121 ms`, `misses ~= cycles` | Control loop blocks on two capture calls. |
| 2026-04-26 | Capture timeout reduced aggressively (`10 ms`) | Faster loop, but PWM input became unresponsive in tests | Timeout too short to reliably observe 50 Hz PWM. |
| 2026-04-26 | Alternating channel capture and pulse-only mode | `eagain` dominated, no valid input updates | Reliability issue on this target/driver path. |
| 2026-04-26 | Added timing and capture diagnostics (`plant timing`, `plant capture`) | Better observability | Enabled hard data for each test. |
| 2026-04-26 | Background PWM capture threads, control loop reads latest sample | Pending hardware validation | Expected to remove capture blocking from 50 ms loop. |
| 2026-04-26 | Background PWM capture threads validated on hardware | `cycles=738`, `misses=0`, `last_exec=0 ms`, `avg_exec=0 ms`, `max_exec=1 ms`, `last_dt=50 ms`, `min_dt=49 ms`, `worst_slack=49 ms` | Control loop deadline margin recovered (near 0-2% CPU budget at 1 ms resolution). |

## How to record a new run

1. Flash new firmware.
2. Reset counters:
   - `plant timing reset`
   - `plant capture reset`
3. Run representative scenario for 30-60 s.
4. Collect:
   - `plant timing`
   - `plant capture`
5. Add one new row in the table above.

## Targets

- Deadline misses: `0`
- Max execution time: `< 40 ms` (preferred)
- Worst slack: `> 10 ms`
- Capture health: `ok` increases steadily for both channels
