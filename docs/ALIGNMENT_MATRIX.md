# Alignment matrix: the filter switched on anywhere in the drive

Generated 2026-11-16 07:52 by `tools/run_alignment_matrix.py` in 154 s. `configs/pos_ins.json` with the `vn100_measured` tuning and initialization `in_motion_align`, run over the full example log once per switch-on time (every 40 s, 64 of them): everything before that time is ignored, so the filter comes up as in a vehicle switched on there, standing or under way. Against the truth channel.

## At the hand-over

| velocity from | aligned | time to align: median, 90 %, longest (s) | heading error: RMS (worst), deg | sigma it starts with, deg | error / sigma: RMS (worst) | tilt error: RMS (worst), deg | switched on under way: n, heading RMS | from a standstill: heading RMS |
|---|---|---|---|---|---|---|---|---|
| velocity_150ms | 64 / 64 | 19, 42, 136 | 3.06 (7.45) | 2.66 | 1.19 (3.90) | 0.74 (2.83) | 48, 2.75 | 3.84 |
| velocity_250ms | 64 / 64 | 18, 42, 136 | 1.61 (3.84) | 2.55 | 0.63 (1.44) | 0.32 (1.08) | 48, 1.63 | 1.53 |
| velocity_350ms | 64 / 64 | 19, 42, 137 | 2.54 (8.48) | 2.49 | 1.13 (3.25) | 0.42 (1.88) | 48, 2.62 | 2.30 |
| velocity_250ms_2deg | 64 / 64 | 23, 53, 137 | 1.11 (3.97) | 1.79 | 0.64 (2.12) | 0.33 (0.98) | 47, 1.20 | 0.83 |
| position | 64 / 64 | 30, 62, 139 | 2.65 (5.22) | 2.74 | 0.95 (1.95) | 0.49 (1.95) | 46, 2.09 | 3.71 |
| velocity_250ms_no_lever | 64 / 64 | 18, 42, 136 | 2.15 (4.89) | 2.54 | 0.98 (3.86) | 0.35 (1.25) | 48, 2.00 | 2.52 |
| velocity_250ms_constraints | 64 / 64 | 18, 42, 136 | 1.61 (3.84) | 2.55 | 0.63 (1.44) | 0.32 (1.08) | 48, 1.63 | 1.53 |
| position_constraints | 64 / 64 | 30, 62, 139 | 2.65 (5.22) | 2.74 | 0.95 (1.95) | 0.49 (1.95) | 46, 2.09 | 3.71 |
| velocity_250ms_zero_motion | 64 / 64 | 18, 42, 136 | 1.61 (3.84) | 2.55 | 0.63 (1.44) | 0.32 (1.08) | 48, 1.63 | 1.53 |
| velocity_250ms_constraints_zero_motion | 64 / 64 | 18, 42, 136 | 1.61 (3.84) | 2.55 | 0.63 (1.44) | 0.32 (1.08) | 48, 1.63 | 1.53 |

- `velocity_150ms`: the receiver's velocity, 150 ms time bias as the example configs give it
- `velocity_250ms`: the receiver's velocity, 250 ms
- `velocity_350ms`: the receiver's velocity, 350 ms
- `velocity_250ms_2deg`: 250 ms, declared at a heading sigma of 2 deg instead of 3
- `position`: differences of the position fixes, no velocity channel
- `velocity_250ms_no_lever`: 250 ms, antenna lever arm left out
- `velocity_250ms_constraints`: 250 ms; afterwards the filter also has the ground-vehicle constraints (same alignment, the difference is in the second table)
- `position_constraints`: position differences; afterwards with the constraints: a vehicle with a position-only receiver
- `velocity_250ms_zero_motion`: 250 ms; afterwards with zero-velocity and zero-rate updates while the vehicle stands, no constraints
- `velocity_250ms_constraints_zero_motion`: 250 ms; afterwards with the constraints and the zero-velocity / zero-rate updates: configs/pos_ins_ground_vehicle.json

The longest times are switch-on times at which the car stands: nothing shows a heading until it moves.

## Afterwards

Heading error (deg) and horizontal position error (m), RMS over the switch-on times, a given time after the hand-over; "given" is the standard run, whose heading comes from the configuration file at the start of the log, at the same moments.

| velocity from | +10 s: heading (worst), given / position (worst), given | +30 s: heading (worst), given / position (worst), given | +60 s: heading (worst), given / position (worst), given | +120 s: heading (worst), given / position (worst), given |
|---|---|---|---|---|
| velocity_150ms | 3.27 (8.15), 0.91 / 1.72 (3.38), 1.54 | 3.52 (10.89), 0.71 / 1.94 (3.79), 1.58 | 2.72 (10.77), 0.88 / 1.88 (4.30), 1.56 | 1.73 (9.96), 0.68 / 1.65 (3.55), 1.53 |
| velocity_250ms | 2.36 (5.26), 0.90 / 1.60 (2.87), 1.55 | 3.11 (9.41), 0.71 / 1.69 (3.36), 1.57 | 2.42 (10.72), 0.88 / 1.75 (3.38), 1.57 | 0.97 (2.45), 0.68 / 1.61 (2.70), 1.55 |
| velocity_350ms | 2.80 (7.82), 0.91 / 1.70 (3.63), 1.55 | 3.06 (7.90), 0.70 / 1.76 (3.26), 1.57 | 2.65 (10.14), 0.90 / 1.83 (3.84), 1.57 | 1.05 (3.77), 0.68 / 1.64 (3.00), 1.55 |
| velocity_250ms_2deg | 2.00 (5.09), 0.92 / 1.56 (2.54), 1.58 | 2.88 (9.41), 0.68 / 1.64 (3.02), 1.59 | 2.16 (10.80), 0.89 / 1.62 (3.77), 1.52 | 0.87 (2.29), 0.68 / 1.64 (3.08), 1.59 |
| position | 2.82 (7.52), 0.92 / 1.68 (3.48), 1.53 | 2.98 (12.13), 0.70 / 1.78 (3.43), 1.60 | 1.75 (6.21), 0.90 / 1.71 (2.94), 1.54 | 0.92 (2.35), 0.65 / 1.70 (2.84), 1.58 |
| velocity_250ms_no_lever | 2.78 (6.29), 0.90 / 1.74 (3.35), 1.55 | 3.52 (11.05), 0.71 / 1.84 (3.86), 1.57 | 2.49 (11.52), 0.88 / 1.89 (3.45), 1.57 | 1.18 (4.22), 0.68 / 1.74 (3.06), 1.55 |
| velocity_250ms_constraints | 1.23 (2.39), 0.90 / 1.61 (2.71), 1.55 | 1.26 (5.09), 0.71 / 1.67 (3.28), 1.57 | 1.21 (6.05), 0.88 / 1.62 (3.24), 1.57 | 0.58 (1.33), 0.68 / 1.55 (2.82), 1.55 |
| position_constraints | 1.86 (5.57), 0.92 / 1.74 (3.36), 1.53 | 1.98 (10.35), 0.70 / 1.76 (2.93), 1.60 | 0.91 (2.87), 0.90 / 1.67 (2.77), 1.54 | 0.52 (1.34), 0.65 / 1.67 (3.02), 1.58 |
| velocity_250ms_zero_motion | 2.30 (5.17), 0.90 / 1.57 (2.87), 1.55 | 2.89 (9.41), 0.71 / 1.69 (3.36), 1.57 | 2.16 (10.72), 0.88 / 1.74 (3.38), 1.57 | 0.66 (1.62), 0.68 / 1.63 (2.72), 1.55 |
| velocity_250ms_constraints_zero_motion | 1.14 (2.39), 0.90 / 1.59 (2.71), 1.55 | 0.73 (1.71), 0.71 / 1.66 (3.28), 1.57 | 0.70 (2.30), 0.88 / 1.59 (2.79), 1.57 | 0.54 (1.33), 0.68 / 1.56 (2.81), 1.55 |
