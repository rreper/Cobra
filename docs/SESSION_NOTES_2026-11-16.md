# Session notes, 16 November 2026: integrity v0.2, Cobra v0.2.10 to v0.2.15

Continues `docs/SESSION_NOTES_2026-11-15.md` (inertial review, v0.2.9). Three things are released: the integrity
layer's v0.2, Cobra v0.2.10 (fusion core) and Cobra v0.2.11 (alignment in motion, section 5.7). Sections 5.1 to
5.6 are the prototype stage of the alignment, kept as written. Zero-velocity and zero-rate updates are
released as v0.2.12 (section 5.8), the start-up gap of the `pva_message` initialization as v0.2.13 (section
5.9), the IMU model for a driving car as v0.2.14 (section 5.10). A standstill confirmed by the receiver's
velocity is released as v0.2.15 (section 5.11). An estimate of the receiver's time offset is the work that
follows (section 5.12).

## 1. What was asked

In order, after the v0.2.9 notes were committed:

1. "can you improve the INS so it is state of the art with respect to 3D fusion and detection of jammed or
   spoofed outputs?"
2. "commit it, merge and push. Then work to improve the fusion core to make it something I can use that will be
   an outstanding performer with GNSS, m-code GPS, IMU, odometry and other inputs."
3. "commit, merge, tag v0.2.10 and push, then do alignment in motion"
4. "update your md files" (this file, the dashboard, the memory notes), after stopping an experiment run.
5. "copy the scripts somewhere durable, then continue the comparison run"
6. "commit, merge, tag v0.2.11 and push, then do the zero-rate updates" (the notes were committed with it as `2044a47`)
7. "commit, merge, tag v0.2.12 and push, then fix the pva_message gap"
8. "commit, merge, tag v0.2.13 and push, then fix the IMU error model"
9. "commit, merge, tag v0.2.14 and push, then do the receiver-confirmed standstill"
10. "commit, merge, tag v0.2.15 and push, then do the receiver time-offset estimate"

## 2. State at the end

| Item | State |
|---|---|
| cobra-integrity | v0.2 on `main` = `origin/main` = `fa50922`, no tag. CI result not read: the repository is private and `gh` is not logged in on this machine |
| Cobra (this repository) | v0.2.11: commit `67870dd` on `main`, tag `v0.2.11` (alignment in motion, correction to v0.2.10); before it v0.2.10 = `03f2947`, whose `ci` and `release` workflows were green |
| Branch `align-in-motion` | merged into `main` (fast-forward) as v0.2.11; all gates passed on the released tree; `ci` (on `2044a47`) and the `release` workflow for the tag are both green |
| Branch `zero-rate` | merged into `main` (fast-forward) as v0.2.12 = `84deb76`; all gates passed on the released tree; `ci` (on `c510049`) and the `release` workflow are green |
| Branch `pva-gap` | merged into `main` (fast-forward) as v0.2.13 = `6c7d695` (notes `dba02cc`); all gates passed on the released tree; `ci` and `release` workflows green |
| Branch `imu-model` | merged into `main` (fast-forward) as v0.2.14 = `239846b`: preset `vn100_driving`, both matrices and the four vehicle examples regenerated with it (section 5.10). All gates passed on the released tree |
| Branch `confirmed-standstill` | merged into `main` (fast-forward) as v0.2.15 = `cca2b41`: `ZeroMotionGeneratorConfig::confirmation_channels`, two new aiding rows, one new alignment row, `pos_ins_ground_vehicle.json` with the confirmation (section 5.11). All gates passed on the released tree |
| Alignment prototype | Python, in `/home/rreper/orin/work/prototypes/align-in-motion/` (outside every repository; section 7) |
| pnt-edge | untouched; pins Cobra v0.2.8. cobra-integrity pins v0.2.7 |

## 3. cobra-integrity v0.2 (released)

Full account in that repository: `docs/DESIGN.md` §8, `docs/PROGRESS.md`, `docs/MONTE_CARLO.md`,
`docs/PROTECTED_MATRIX.md`. In short, all opt-in:

- Vertical separation test and a vertical protection level. An altitude pull of 0.5 m/s on one of two sources
  is isolated in 20 of 20 seeds after 42.5 ± 5.5 s (5.1 m vertical RMS, 51 m unprotected).
- An INS monitor for a receiver that stands alone: each fix against a ladder of six IMU-only coasts. On the
  recorded drive it excludes a horizontal pull of 1 m/s in 5 to 8 s and of 0.75 m/s in 6 to 8 s (one case
  34 s); 0.5 m/s horizontal is never detected. Vertical: 0.5 m/s in 14 to 15 s, 0.3 m/s in 2 of 6 cases.
- Readmission only while the test could still see a fault of the alert-limit size, and an "unverified margin"
  on the protection levels after a readmission that only the IMU vouched for.
- Every v0.1 scenario reproduces v0.1 exactly. 30 tests.

Limits stated in its documents: with one receiver the protection level covers the fault-free case only; an
excluded lone receiver that is not re-validated within about 35 s stays out and the output is free inertial,
flagged EMERGENCY; receiver signal indicators (C/N0, AGC, the u-blox jamming and spoofing flags), tightly
coupled GNSS and a clock test are not there.

## 4. Cobra v0.2.10 (released)

Full account: `docs/DESIGN.md` §7.23, `docs/AIDING_MATRIX.md`, `docs/PROGRESS.md`. Two defects, each remedy
opt-in:

- A measurement stream faster than the receiver's time bias made the filter discard the receiver's fixes as
  "older than the filter" (2493 of 2602, logged at DEBUG). `measurement_reorder_s` applies measurements in
  time order behind a window; a discard is now a WARN whatever the setting.
- Body velocity without calibration states degraded the solution (1.55 m became 3.3 m; 7.9 m for a wheel speed
  2 % off). Processor `pinson_odometry` carries scale-factor and pitch / yaw misalignment states.

Horizontal error at the end of a complete GNSS outage, mean (worst of six), metres:

| Aiding during the outage | no outage, RMS | 60 s | 120 s |
|---|---|---|---|
| IMU alone | 1.55 | 25.1 (56.6) | 130.8 (290.7) |
| constraints from the IMU clock | 1.52 | 13.3 (25.6) | 15.6 (27.3) |
| wheel speed (2 % off) + constraints | 1.58 | 4.9 (10.6) | 7.4 (19.9) |
| the same without calibration states | 7.92 | 12.5 (24.2) | 19.8 (31.1) |
| wheel + second receiver + barometer, first receiver lost | 1.16 | 1.8 (3.9) | 1.2 (2.0) |

The wheel speed and the second receiver are emulated from the recording.

Gates on the committed tree: 23 suites / 245 tests, also under ASan + UBSan; parity dump byte-identical to
v0.2.9 (sha256 `add099b17da4…`, unchanged since v0.2.8); `inertial_parity_check.py` exit 0; acceptance 24 / 24;
`compare_logs.py`: 24 logs, 61,772 messages identical.

Not in it: alignment in motion (section 5), zero-velocity updates from a stationarity detector, adaptive
measurement weighting, tightly coupled GNSS. A military-code receiver is a position / velocity source to the
filter (the second receiver of `configs/pos_ins_full.json`); its own interface belongs in pnt-edge and needs
the receiver's interface document.

## 5. Alignment in motion: where it stands

### 5.1 Why

Every example configuration takes the heading from the configuration file (`ManualHeadingAlignmentConfig`,
0.069 rad for this drive) or from a PVA message. `StaticAlignment` gyro-compasses, which a MEMS gyro cannot do:
the VN-100's turn-on bias on this recording is 20 to 40 times the Earth's rate. A vehicle that is switched on
somewhere else therefore has no heading.

### 5.2 The recording

- The car stands for the first 125 s, then drives in town with thirteen stops; 19.7 % of the log is stationary,
  top speed 19.1 m/s.
- Course minus heading above 3 m/s: mean −0.34°, standard deviation 0.73°, largest 7.7°.
- Horizontal velocity change per second: median 0.26 m/s, 90 % below 1.36 m/s, largest 3.73 m/s.
- Mean gyro output while standing (the turn-on bias): (−1.61, −1.99, −2.75) mrad/s, that is
  (−0.092, −0.114, −0.158) °/s.
- Channels: IMU 100 Hz (integrated type), receiver position, velocity (NED) and PVA at 1 Hz, reference
  `/sensor/ins-d/pva` at 200 Hz. Antenna lever arm in the configurations: (−0.5, 0.38, −0.05) m.

### 5.3 Method of the prototype

Alignment as an attitude-determination problem (the velocity integration formula of the optimization-based
alignment literature, Wu and Pan 2013), extended by the gyro bias. No assumption about how the sensor is
mounted or that the vehicle moves forward.

For the velocity epochs `i = 0..n` of a window (at most 30 s), with `X(t)` the body rotation since the window
start from the gyros (bias `b` removed) and `f` the specific force:

```
x_i = ∫ X f dt  (+ X_i (ω_i × l) − X_0 (ω_0 × l) for an antenna lever arm l)
y_i = v_i − v_0 − ∫ (g − 2 ω_ie × v) dt                (NED, from the receiver)
y_i = C0 (x_i + G_i δb) + c + e_z d_z τ_i + noise_i
```

- `C0`: attitude at the window start. `δb`: gyro-bias correction, `G_i = ∂x_i/∂b` accumulated with the
  integration. `c`: absorbs the velocity noise of the first epoch. `d_z`: vertical accelerometer bias and
  gravity-model error (it cannot be absorbed by a rotation; without it the residuals grew to 0.3 m/s over 30 s).
- Solved by alternating a weighted Wahba problem (SVD) for `C0` with a linear least-squares step for
  `(δb, d_z, c)`, with priors on `b` (3 mrad/s) and `d_z` (0.072 m/s²). The Wahba step is global, so a heading
  that is wrong by any angle is no obstacle.
- Covariance from the Gauss–Newton Hessian over (rotation, bias, `d_z`, `c`), scaled by `max(1, χ²/dof)`, and
  carried to the last epoch including the bias uncertainty times the elapsed time.
- While the vehicle stands (receiver speed below 0.25 m/s and a quiet gyro) the gyro mean becomes the bias
  prior, minus the vertical part of the Earth's rate, which needs no heading; the window then starts at the
  last stationary epoch.
- Declared aligned when the heading sigma is below 3° with at least five epochs in the window.
- Approximations, with their size: the frame is NED at the start (0.16 mrad per kilometre travelled); the
  Earth's rate is left out of the attitude kinematics (0.25° per minute, and a bias taken at standstill already
  contains it); the Coriolis term is kept.

### 5.4 Results so far

Without bias estimation the plain method is useless on this sensor: heading errors of 5 to 30°, residuals
growing with the window. With it, switched on at 64 times through the drive (every 40 s, standing and moving),
receiver velocity at the configured 150 ms time bias unless noted:

| Variant | aligned | heading error RMS (max) | error / sigma RMS (max) | tilt RMS (max) | time, median (max) |
|---|---|---|---|---|---|
| as described, no lever arm | 64 / 64 | 3.85° (8.84°) | 1.65 (5.08) | 0.41° (2.34°) | 16 s (134 s) |
| lever arm compensated | 64 / 64 | 3.18° (7.19°) | 1.40 (4.04) | 0.36° (1.87°) | 16 s (134 s) |
| lever arm, velocity 0.05 s earlier | 64 / 64 | 2.30° (6.39°) | 1.01 (3.25) | 0.25° (1.06°) | 16 s |
| lever arm, velocity 0.10 s earlier | 64 / 64 | 2.02° (7.53°) | 0.80 (2.71) | 0.22° (1.26°) | 16 s |
| lever arm, velocity 0.05 s later | 64 / 64 | 3.81° (10.07°) | 1.58 (5.78) | 0.41° | 16 s |
| lever arm, no bias from standstill | 64 / 64 | 2.75° (7.19°) | 1.05 (2.44) | 0.36° | 15 s (130 s) |
| lever arm, velocity from position differences (σ 0.3 / 0.6 m/s) | 64 / 64 | 1.82° (4.15°) | 0.63 (1.48) | 0.22° (1.19°) | 43 s (166 s) |

The 134 s is the standing start: 125 s of it are the car not moving.

The comparison that decides the default source (`exp3.py`, results in `exp3_results.txt` beside it), lever arm
compensated unless noted:

| Variant | aligned | heading error RMS (max) | error / sigma RMS (max) | tilt RMS (max) | time, median |
|---|---|---|---|---|---|
| receiver velocity 0.20 s earlier | 64 / 64 | 2.51° (7.19°) | 1.26 (3.34) | 0.27° (1.30°) | 15 s |
| the same, declared at 2° instead of 3° | 64 / 64 | 1.93° (5.77°) | 1.20 (3.34) | 0.21° (1.12°) | 18 s |
| the same, reported sigmas × 1.5 | 64 / 64 | 2.40° (7.40°) | 0.99 (2.65) | 0.25° (1.24°) | 17 s |
| the same, no bias from standstill | 64 / 64 | 2.33° (7.19°) | 1.04 (3.28) | 0.27° (1.30°) | 15 s |
| position differences, σ 0.15 m/s | 64 / 64 | 3.07° (8.82°) | 1.17 (3.25) | 0.30° (1.58°) | 24 s |
| position differences, σ 0.10 m/s | 64 / 64 | 3.02° (8.18°) | 1.20 (3.74) | 0.33° (1.47°) | 20 s |
| position differences, σ 0.15 m/s, no lever arm | 64 / 64 | 2.54° (8.83°) | 0.94 (3.26) | 0.30° (1.58°) | 24 s |
| position differences, σ 0.15 m/s, 60 s window | 64 / 64 | 3.22° (8.82°) | 1.21 (3.25) | 0.31° (1.58°) | 24 s |

### 5.5 Findings

1. **Gyro bias must be estimated in the alignment.** Taking it from a standstill is not enough on its own
   either: a vehicle switched on while moving has none.
2. **The receiver's velocity is older than its time stamp says.** Against the reference, RMS horizontal error
   of the velocity stream by time shift beyond the configured 150 ms:

   | shift (s) | −0.30 | −0.25 | −0.20 | −0.15 | −0.10 | −0.05 | 0 | +0.05 | +0.10 |
   |---|---|---|---|---|---|---|---|---|---|
   | m/s | 0.091 | 0.076 | 0.069 | 0.073 | 0.086 | 0.105 | 0.127 | 0.150 | 0.175 |

   Best at 0.35 s total. The position fits best about 0.03 to 0.05 s beyond its 150 ms. `configs/pos_vel_ins.json`
   applies 150 ms to both channels. Not changed: it is a default configuration and its acceptance rows would
   move.
3. **Velocity from position differences is a sound alternative, not a better one.** Against the reference
   displacement it is good to 0.10 m/s per axis and it does not depend on the velocity stream's latency. Its
   1.8° in the first table came from waiting longer (the cautious 0.3 m/s); given a realistic 0.10 to 0.15 m/s
   it declares after 20 to 24 s with 3.0° RMS, against 2.0 to 2.5° after 15 s for the receiver's velocity.
   How long the method waits, that is the heading sigma it is asked for, decides the accuracy more than the
   source does.
   The alignment itself puts the velocity 0.10 s earlier than configured (2.02°) rather than 0.20 s (2.51°):
   the reference solution has a time offset of its own against the IMU, and it is the offset between receiver
   and IMU that counts. So the velocity stream is 0.25 to 0.35 s old, not 0.15 s.
4. **The antenna lever arm matters at low speed**: 0.63 m at a turn rate of 20°/s is 0.2 m/s of lateral
   velocity.
5. The reported sigma is optimistic by a factor of about 1.4 to 1.65 when the velocity timing is off, and
   about right (0.8 to 1.0) when it is corrected.

### 5.6 Not done, not decided

- Decided by the comparison above: the receiver's velocity when a velocity channel is among the alignment
  channels, position differences otherwise; threshold 3° until the filter's behaviour after the hand-over is
  measured (2° costs 3 s and gives 1.9° RMS).
- Not explained: the lever-arm compensation helps with the receiver's velocity (3.85° to 3.18°) and does not
  with position differences (2.54° without, 3.07° with). With 64 starts a difference of 0.5° is about two
  standard deviations of the RMS estimate, so this may be noise.
- An estimate of the receiver's time offset inside the alignment would remove the largest error source; not
  in the prototype.
- Nothing in C++ yet. Plan:
  - `inertial::InMotionAlignment` beside `StaticAlignment`, an initialization plugin and an
    `InMotionAlignmentConfig` (JSON), selected by the application's `initialization` entry. Nothing changes for
    a configuration that does not name it.
  - Hand-over at the latest IMU time (the receiver's fixes arrive 2 s behind the IMU): position, velocity,
    attitude with the full 3×3 tilt covariance, the gyro bias as an inertial error with its covariance.
  - A stationarity detector as its own small class; the zero-velocity updates that come next need the same one.
  - Failing tests first; `tools/run_alignment_matrix.py`: switch-on at many times of the drive, time to align,
    heading error and sigma at hand-over, navigation error a minute later against the run that is given the
    heading.
- After a hand-over with a few degrees of heading error the filter has to finish the job; whether the
  15-state linear model does that well from 3° has not been measured yet.

### 5.7 C++ implementation (released as v0.2.11)

Written after the notes above. The full account is `docs/DESIGN.md` §7.24 and `docs/ALIGNMENT_MATRIX.md`.

- `inertial::InMotionAlignment` (`include/pntos/cobra/initialization/InMotionAlignment.hpp`, `.cpp`),
  `InMotionAlignInitializationPlugin`, `InMotionAlignmentConfig` (registry and JSON), application entry
  `"initialization": "in_motion_align"`. `AlignBase` gained `accepts_velocity()` and `summary()`; the static
  alignments are untouched.
- Differences from the prototype: the whole accelerometer bias is estimated (three parameters with the model's
  initial sigma as prior, in place of the vertical term). `InMotionAlignment.TheGyroBiasHasToBeEstimatedAlong`
  showed why: on a gentle simulated drive 3 milli-g of level accelerometer bias passed for a vertical gyro bias
  and the method claimed 3 degrees while being 12.4 degrees off; with the bias modelled 2.6 degrees. Lost IMU
  samples are held (as the inertial's `imu_gap_policy` "hold"); the hand-over is at the last IMU sample.
- `tests/test_alignment_in_motion.cpp`: 16 tests on simulated drives (standing start, switched on under way,
  any heading and mounting, no heading claimed without horizontal acceleration, positions only, lever arm,
  honest sigma over 40 drives, lost samples, IMU gap, plugin, config and JSON, and the example configuration
  through `cobra::Filter`: first solution 15 s into the drive with the heading 1.1 deg off, then at worst
  0.9 deg, 0.8 m and 0.2 m/s). All pass.
- `tools/run_alignment_matrix.py`: the recorded drive, switched on at 64 times, through `cobra_run`
  (`docs/ALIGNMENT_MATRIX.md`):

  | velocity from | aligned | time: median, 90 %, longest | heading RMS (worst) | sigma | error / sigma | tilt RMS |
  |---|---|---|---|---|---|---|
  | receiver, 150 ms | 64 / 64 | 19, 42, 136 s | 3.06 deg (7.45) | 2.66 | 1.19 | 0.74 |
  | receiver, 250 ms | 64 / 64 | 18, 42, 136 s | 1.61 deg (3.84) | 2.55 | 0.63 | 0.32 |
  | receiver, 350 ms | 64 / 64 | 19, 42, 137 s | 2.54 deg (8.48) | 2.49 | 1.13 | 0.42 |
  | receiver, 250 ms, declared at 2 deg | 64 / 64 | 23, 53, 137 s | 1.11 deg (3.97) | 1.79 | 0.64 | 0.33 |
  | position differences | 64 / 64 | 30, 62, 139 s | 2.65 deg (5.22) | 2.74 | 0.95 | 0.49 |
  | receiver, 250 ms, no lever arm | 64 / 64 | 18, 42, 136 s | 2.15 deg (4.89) | 2.54 | 0.98 | 0.35 |

  So against the IMU the velocity stream is 250 ms old (350 ms was the fit against the reference solution).
- After the hand-over (plain `pos_ins`, position fixes only), heading RMS over the switch-on times: 1.6 deg at
  the hand-over, 2.2 after 10 s, 2.5 after 30 s, 2.4 after 60 s, 1.0 after 120 s. Switched on under way the
  vertical gyro bias is still unknown (estimate near 0, truth -2.7 mrad/s), the heading drifts by 0.15 deg/s
  until the filter has found it (worst case 10.7 deg after 60 s). With the motion constraints of v0.2.10 in
  the filter: 1.6, 1.2, 0.8, 0.8, 0.6 deg. With the receiver's velocity as a filter measurement: no better
  (1.8, 2.2, 2.4, 0.9) and the position gets worse (2.1 m against 1.5 m).
  `prototypes/align-in-motion/after_handover.py` prints this per switch-on time.
- **Flaw found in v0.2.10 on the way:** `measurement_reorder_s` counts back from the IMU's time, and a
  measurement that passed the controller's 2 s buffer is already 2 s behind it. The 0.5 s that v0.2.10
  recommended for that case orders nothing; `configs/pos_ins_full.json` as released drops ten or more fixes of
  the first receiver. Fixed here, test first (`Filter.ReorderWindowMustExceedTheControllerBuffer`): a window
  not longer than the buffer is a WARN at start-up, the documents say 2.5 s, the two example configurations are
  regenerated with 2.5 s. The aiding table moved by at most 0.4 m (worst cases; the means by 0.2 m).
- With the constraints the three worst cases (4 to 6 deg within a minute of the hand-over) are switch-on times
  shortly before a stop; the filter's sigma there is 5 to 8 deg. Zero-rate updates at standstill are what is
  missing.
- New example configurations: `configs/pos_ins_align_in_motion.json`, `configs/pos_ins_ground_vehicle.json`
  (alignment and constraints: a receiver and an IMU, nothing else).
- Gates on this tree (`scratchpad/align_gates.sh`): 24 suites / 262 tests, also under ASan + UBSan with no
  report; parity dump byte-identical (`add099b17da4`); `inertial_parity_check.py` exit 0; acceptance 24 / 24;
  `compare_logs.py`: 24 logs, 61,772 messages identical.
- Documents: DESIGN.md §7.24, deviations 23 and 24, §9.6; PROGRESS.md; GETTING_STARTED.md; TESTING.md;
  TEST_MATRIX.md regenerated; README; AIDING_MATRIX.md regenerated with the 2.5 s window.
- Not done: zero-velocity / zero-rate updates from a stationarity detector, an estimate of the receiver's time
  offset, the heading / gyro-bias correlation in the hand-over.

### 5.8 Standing still: zero-velocity and zero-rate updates (released as v0.2.12)

Full account: `docs/DESIGN.md` §7.25. In short:

- `inertial::StationarityDetector` (IMU only: 0.2 s means over a 2 s window, 0.003 rad/s and 0.15 m/s²),
  preprocessor `zero_motion_generator`, measurement processor `pinson_zero_rate` (eleventh), aux channel
  `INERTIAL_SENSOR_ERRORS`. The zero velocity goes through the existing `pinson_velocity`.
- Measured first on the recording (`prototypes/align-in-motion/stationarity*.py`): per sample an idling engine
  hides the difference between standing and driving; the 0.2 s means separate them (95 % of standing windows,
  0 of 1030 above 0.5 m/s). As run: 420 messages, 396 at a true standstill, 24 while creeping (up to 0.49 m/s).
- Results: standing a minute, heading drift 0.15° instead of 8.9°. Outages, mean (worst): 60 s 10.7 (23.4) m
  instead of 25.1 (56.6); 120 s 33.8 (65.6) m instead of 130.8 (290.7); with the constraints 6.2 (12.2) and
  9.3 (15.7) m; 40 s while standing 2.2 m instead of 10.9. After an alignment in motion with constraints and
  zero-motion updates: heading 0.7° RMS (1.7° worst) 30 s after the hand-over.
- The zero velocity is weighted at 0.2 m/s, not at what a standing car does: at 0.05 m/s the whole-drive error
  with the receiver present rose from 1.55 / 1.58 m to 1.85 / 2.51 m. Causes measured: detections while
  creeping, and accelerometer biases that wander by 2.3 mm/s² between stretches of the drive against 0.6 in
  the IMU model (`prototypes/align-in-motion/accel_regime.py`, `zero_motion_sigma.py`).
- Found, not fixed: the `pva_message` initialization leaves a 2 s gap before the first IMU sample; the solution
  falls at 19.5 m/s three seconds after the start.
- Gates on this tree: 24 suites / 272 tests, also under ASan + UBSan with no report; parity dump
  byte-identical; acceptance 24 / 24; 61,772 messages identical.
- `configs/pos_ins_ground_vehicle.json` now has alignment in motion, constraints and zero-motion updates;
  `docs/AIDING_MATRIX.md` has two new rows and a standing-outage column; `docs/ALIGNMENT_MATRIX.md` two new rows.

### 5.9 The start-up hole of the `pva_message` initialization (released as v0.2.13)

Full account: `docs/DESIGN.md` §7.26.

- Cause: the orchestration drops IMU messages until the initialization has a solution; the PVA of `pva_message`
  passes the controller's 2 s buffer, so the inertial is created two seconds in the past and mechanizes the
  hole as free fall. Inherited from Cobra, kept as the default.
- Measured: 60 s recording (standing) 15.8 m/s of fall and 59 m of altitude error within ten seconds; nine
  starts while driving on the full recording 62 to 73 m in altitude, 5 to 13 m horizontally
  (`prototypes/align-in-motion/pva_start.py`).
- Fix, opt-in: `StandardOrchestrationConfig::initialization_imu_replay_s` keeps the IMU of that many seconds
  and gives what is later than the initial solution to the new inertial, the straddling sample cut to its part.
  With 2.5 s: 0.27 m/s and 0.75 m standing; 0.5 to 1.2 m in altitude and 0.6 to 1.5 m horizontally driving.
- `imu_gap_policy: "hold"` lessens the fall to 5.8 m/s on the standing car; why not to zero is not established.
- Tests: `Filter.AnInitialSolutionOlderThanTheImuLeavesAHoleByDefault` (pins the default),
  `Filter.ImuKeptBeforeTheInitializationClosesTheHole` (failed before the fix), `JsonConfig.InitializationImuReplayKey`.
- Gates on this tree: 24 suites / 275 tests, also under ASan + UBSan with no report; parity dump byte-identical;
  acceptance 24 / 24; 61,772 messages identical.

### 5.10 The IMU model in a driving car (released as v0.2.14)

Full account: `docs/DESIGN.md` §7.27. The request was the IMU error model, so that the zero velocity could carry
its real weight; my diagnosis in v0.2.12 (accelerometer biases that wander) was half wrong.

- Accelerometer error against the reference by averaging time (`prototypes/align-in-motion/accel_allan.py`):
  vertical axis at 20 s 1.2 mm/s² standing, 3.1 mm/s² driving; below 20 s and on the level axes the reference
  itself limits what can be seen. 3.1 mm/s² at 20 s is 1.4e-2 m/s/√s of white noise, six times
  `vn100_measured`, which is the unit at rest. New preset `vn100_driving`: that one term.
- Sweeps through the filter (`imu_model_sweep.py`, `imu_model_candidates.py`, `vertical_test.py`): the driving
  noise improves the vertical error in every aiding row and makes the 60 s outage sigma honest (0.97 for the
  IMU alone, was 1.10); a larger accelerometer bias sigma makes everything worse; gyro terms help nowhere
  consistently (one lead: gyro bias ×4 halves the outage error of constraints + zero motion).
- The horizontal cost of a tight zero velocity is creeping detections (`creep_test.py`,
  `zero_motion_check.py`): default detector 21 creeping messages of 420 → 1.78 m; 0.0015/0.08/2 s, 6 → 1.61 m;
  0.001/0.06/3 s, none of 286 → 1.55 m, the same as without zero velocity, and the 60 s outage at 9.5 (19.8) m
  against 10.7 (23.8) m for the default at 0.2 m/s. Defaults kept; the strict setting is documented as the
  recipe for a vehicle whose standstill has been looked at.
- Not explained: 0.13 m of vertical RMS that a zero velocity at 0.05 m/s still costs under the strict detector.
- The tools and the four vehicle examples use `vn100_driving`; `docs/AIDING_MATRIX.md` and
  `docs/ALIGNMENT_MATRIX.md` are regenerated with it (the alignment figures are unchanged).
- Gates on this tree: 24 suites / 276 tests, also under ASan + UBSan with no report; parity dump byte-identical;
  acceptance 24 / 24; 61,772 messages identical.

### 5.11 A standstill confirmed by the receiver (released as v0.2.15)

Full account: `docs/DESIGN.md` §7.28.

- The defect: a car creeping at a steady speed is as quiet to the IMU as one that stands. The C++ generator
  makes 480 messages on the recording, 23 of them with the reference above 0.05 m/s (fastest 0.52 m/s).
- The receiver's velocity (F9T) at rest never exceeds 0.026 m/s; 0.05 m/s separates standing from all but the
  slowest creeping (`prototypes/align-in-motion/confirm_check.py`).
- Design, decided by how messages flow: the receiver's velocity reaches the preprocessors through the
  controller's 2 s buffer, so the IMU's window cannot be confirmed when it closes. Instead each receiver epoch
  is judged when it arrives: slow, and a second of quiet IMU on either side → zero velocity and zero rate
  stamped at the epoch. Fast → counted, logged with the speed (INFO at the 1st, 10th, ...; DEBUG each). No
  usable velocity for 2.5 s → the IMU alone as in v0.2.12. The existing reorder rule (buffer plus time bias,
  2.5 s) is enough; the filter test checks that nothing is discarded.
- Keys: `confirmation_channels`, `confirmation_speed` 0.05, `confirmed_velocity_sigma` 0.05 (north and east),
  `confirmed_vertical_velocity_sigma` 0.2, `confirmation_max_sigma` 0.5, `confirmation_timeout` 2.5.
- Measured (`confirm_sweep.py`, `docs/AIDING_MATRIX.md`): h / v RMS 1.53 / 1.61 m (IMU alone at 0.2 m/s:
  1.57 / 1.61; at 0.05: 1.78 / 1.72); 60 s outages 10.3 (22.5) m against 10.7 (23.8); standing outage 1.8
  against 2.1 m; 120 s unchanged. With constraints and after an alignment in motion: no resolvable difference.
  The gain on this recording is small.
- The vertical item of §5.10 is explained (`confirm_vertical.py`, `confirm_vertical2.py`, `accel_mean.py`,
  `confirm_axes.py`): the receiver's height is −0.89 m off the reference on average (std 1.38), which is most
  of every "vertical RMS" here; the vertical accelerometer reads about 2 mm/s² differently driving than
  standing (eight of eight bracketed stretches, +0.7 to +3.7); a firm vertical zero velocity pins the standstill
  bias and the solution sinks 0.13 m against the receiver. Vertical kept at 0.2 m/s → no cost.
- Correction: the counts 420 / 396 / 24 / 21 in the v0.2.12 to v0.2.14 documents came from a Python replica that
  spaces messages 1.2 s apart on the raw IMU stamps; the generator makes 480 (29 above 0.03 m/s, 23 above
  0.05). Found by printing the generator's own stamps through a temporary hook (removed; outputs kept in
  `prototypes/align-in-motion/zm/`). Lesson: a figure about the C++ belongs to the C++'s own output.
- A robustness hole found on review and fixed test-first: one velocity message with a future time stamp held
  up every later epoch (`AVelocityStampedInTheFutureDoesNotHoldUpTheOthers`).
- Gates on this tree: 24 suites / 288 tests, also under ASan + UBSan with no report; parity dump byte-identical;
  acceptance 24 / 24; 61,772 messages identical. The aiding matrix re-run with the final binary reproduces
  `docs/aiding_matrix.json` exactly. v0.2.14's `ci` and `release` workflows are green.

## 6. Open decisions carried forward


- `pos_vel_ins.json`: velocity time bias 150 ms where 250 ms fits against the IMU (350 ms against the reference
  solution). Change the default configuration, or leave it and use the measured figure in new configurations
  only? The two new configurations use 250 ms.
- cobra-integrity: should an excluded lone receiver's fix be offered as an unassured last resort?
- Bump the `pntos-cpp` pins to the latest tag (cobra-integrity v0.2.7, pnt-edge v0.2.8) and surface the new integrity
  status keys in pnt-edge.
- Receiver signal indicators into the integrity monitor.
- The items of `SESSION_NOTES_2026-11-15.md` §6 and §7 are unchanged.

## 7. How to pick this up again

- Build and gates: `export PATH=$PWD/Cobra/.venv/bin:$PATH; meson compile -C build; meson test -C build`;
  `tools/inertial_parity_check.py`; `build/tools/inertial_parity_dump <log> 25` against a pristine build;
  `tools/run_acceptance.py`; `tools/compare_logs.py`. The recipe for the pristine reference is in
  `SESSION_NOTES_2026-11-15.md` §8; use `REF=v0.2.10`.
- Prototype: `/home/rreper/orin/work/prototypes/align-in-motion/` (copied there on 16 November; its
  `README.md` lists the scripts). `oba3.py` is the prototype of §5.3, `exp2.py` the table of §5.4, `exp3.py`
  the comparison of §5.6, `lag.py` finding 2, `profile.py` §5.2. Run with `Cobra/.venv/bin/python`.
- Scratch directory of this session (gone after a reboot):
  `/tmp/claude-1000/-home-rreper-orin-work/390f72b7-9ad8-4396-8ce9-ea4300851024/scratchpad/` with `baseline/`
  (pristine build outputs), `rel0210/` (the v0.2.10 gate outputs), `asan/` (sanitizer build).

## 8. Where the rest is

- Design: `docs/DESIGN.md` (§7.22 inertial for live sensors, §7.23 aiding beside the receiver, §9.6 open items).
- Progress log: `docs/PROGRESS.md`. Measurements: `docs/AIDING_MATRIX.md`, `docs/TEST_MATRIX.md`.
- Integrity layer: `/home/rreper/orin/work/cobra-integrity/docs/`.
- Dashboard: `~/projects/_dashboard/projects/{pntos,cobra-integrity,pnt-edge}.{json,md}`.
