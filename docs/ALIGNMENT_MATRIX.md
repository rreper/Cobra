# Alignment matrix: the filter switched on anywhere in the drive

Generated 2026-11-16 08:42 by `tools/run_alignment_matrix.py` in 155 s. `configs/pos_ins.json` with the `vn100_driving` tuning and initialization `in_motion_align`, run over the full example log once per switch-on time (every 40 s, 64 of them): everything before that time is ignored, so the filter comes up as in a vehicle switched on there, standing or under way. Against the truth channel.

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
| velocity_150ms | 3.27 (8.15), 0.95 / 1.72 (3.38), 1.53 | 3.52 (10.92), 0.75 / 1.94 (3.76), 1.58 | 2.73 (10.77), 0.91 / 1.87 (4.27), 1.56 | 1.75 (10.07), 0.71 / 1.64 (3.30), 1.54 |
| velocity_250ms | 2.36 (5.26), 0.94 / 1.60 (2.87), 1.55 | 3.12 (9.40), 0.75 / 1.69 (3.35), 1.57 | 2.45 (10.73), 0.91 / 1.74 (3.36), 1.56 | 0.96 (2.39), 0.71 / 1.60 (2.63), 1.55 |
| velocity_350ms | 2.80 (7.82), 0.95 / 1.70 (3.63), 1.54 | 3.07 (7.90), 0.74 / 1.76 (3.26), 1.57 | 2.67 (10.16), 0.93 / 1.82 (3.72), 1.56 | 1.03 (3.38), 0.71 / 1.64 (2.87), 1.55 |
| velocity_250ms_2deg | 2.00 (5.09), 0.95 / 1.56 (2.54), 1.57 | 2.88 (9.40), 0.72 / 1.63 (3.02), 1.59 | 2.18 (10.80), 0.92 / 1.62 (3.68), 1.52 | 0.85 (2.26), 0.71 / 1.63 (2.93), 1.59 |
| position | 2.82 (7.51), 0.96 / 1.68 (3.49), 1.53 | 2.99 (12.15), 0.75 / 1.78 (3.43), 1.60 | 1.75 (6.10), 0.93 / 1.71 (2.88), 1.53 | 0.93 (2.20), 0.69 / 1.70 (2.85), 1.58 |
| velocity_250ms_no_lever | 2.78 (6.29), 0.94 / 1.74 (3.35), 1.55 | 3.52 (11.05), 0.75 / 1.84 (3.85), 1.57 | 2.51 (11.52), 0.91 / 1.89 (3.42), 1.56 | 1.13 (3.49), 0.71 / 1.72 (2.94), 1.55 |
| velocity_250ms_constraints | 1.23 (2.39), 0.94 / 1.61 (2.71), 1.55 | 1.25 (5.07), 0.75 / 1.67 (3.28), 1.57 | 1.24 (6.33), 0.91 / 1.62 (3.24), 1.56 | 0.59 (1.30), 0.71 / 1.56 (2.79), 1.55 |
| position_constraints | 1.86 (5.57), 0.96 / 1.74 (3.36), 1.53 | 1.98 (10.36), 0.75 / 1.76 (2.94), 1.60 | 0.92 (3.08), 0.93 / 1.67 (2.76), 1.53 | 0.52 (1.34), 0.69 / 1.67 (3.07), 1.58 |
| velocity_250ms_zero_motion | 2.30 (5.17), 0.94 / 1.57 (2.87), 1.55 | 2.90 (9.40), 0.75 / 1.69 (3.35), 1.57 | 2.18 (10.73), 0.91 / 1.74 (3.36), 1.56 | 0.69 (1.62), 0.71 / 1.62 (2.63), 1.55 |
| velocity_250ms_constraints_zero_motion | 1.14 (2.39), 0.94 / 1.59 (2.71), 1.55 | 0.72 (1.69), 0.75 / 1.65 (3.28), 1.57 | 0.70 (2.27), 0.91 / 1.59 (2.78), 1.56 | 0.55 (1.30), 0.71 / 1.57 (2.76), 1.55 |
