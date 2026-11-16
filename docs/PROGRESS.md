# Progress log

Newest first. Each entry: what changed, what was verified, what is next.

## 2026-11-16 — v0.2.13: the start-up hole of the pva_message initialization

- Asked for: the gap found while testing the zero-motion updates. The orchestration drops IMU messages until the
  initialization has a solution. The PVA of `pva_message` passes the controller's buffer and is two seconds old
  when it arrives; the inertial is created two seconds in the past and mechanizes the hole as free fall.
  Measured: on the 60 s recording the solution falls at up to 15.8 m/s and is 59 m off in altitude within ten
  seconds; started while driving at nine points of the full recording, 62 to 73 m in altitude and 5 to 13 m
  horizontally. Inherited from Cobra, so kept as the default.
- `StandardOrchestrationConfig::initialization_imu_replay_s` (DESIGN.md §7.26): the IMU messages of that many
  seconds are kept until the initialization has its solution, and those after the solution's time go to the new
  inertial first, the straddling one cut to its part. With 2.5 s: 0.27 m/s and 0.75 m on the 60 s recording;
  0.5 to 1.2 m in altitude and 0.6 to 1.5 m horizontally for the nine starts while driving.
- `imu_gap_policy: "hold"` is not the answer: it lessens the fall to 5.8 m/s on the standing car (why not to
  zero I have not established) and would hold one sample across two seconds of a moving one.
- 24 suites / 275 tests. Default mode unchanged: parity dump byte-identical, acceptance 24 / 24, the 61,772
  output messages identical to v0.2.12.

## 2026-11-16 — v0.2.12: standing still, zero-velocity and zero-rate updates

- Asked for: zero-rate updates. A standing vehicle's heading ran away with the gyro bias (8.9 deg in a minute with
  position fixes alone, `Filter.StandingStillTheHeadingDriftsWithTheGyroBias`), and after an alignment in motion
  the bias about the vertical stayed unknown until a manoeuvre. DESIGN.md §7.25.
- `inertial::StationarityDetector` (the IMU alone: 0.2 s means over a 2 s window; per sample an idling engine
  hides the difference), preprocessor `zero_motion_generator` (a zero velocity and a zero rate a second while the
  window is quiet), measurement processor `pinson_zero_rate` (eleventh), aux channel `INERTIAL_SENSOR_ERRORS`
  (the bias the inertial applies, which no processor could see). All opt-in.
- On the recording the detector says "stands" 420 times: 396 at a true standstill, 24 while the car creeps (up
  to 0.49 m/s), never above 0.5 m/s.
- Result: standing for a minute, heading drift 0.15 deg instead of 8.9. Outages (mean, worst of six): 60 s 10.7
  (23.4) m instead of 25.1 (56.6), 120 s 33.8 (65.6) m instead of 130.8 (290.7), no extra sensor; with the
  constraints as well 6.2 (12.2) and 9.3 (15.7) m. A 40 s outage while standing: 2.2 m instead of 10.9. After an
  alignment in motion, with constraints and zero-motion updates, the heading is 0.7 deg RMS (1.7 worst) 30 s
  after the hand-over, level with the run that is given its heading. `configs/pos_ins_ground_vehicle.json` has
  all three now.
- Found on the way: (1) a zero velocity weighted at 0.05 m/s made the solution worse with the receiver present
  (1.55 / 1.58 m became 1.85 / 2.51 m): the detector also fires while creeping, and the filter pins accelerometer
  biases that wander by 2.3 mm/s2 between stretches of the drive (the IMU model allows 0.6). The default is
  0.2 m/s, which keeps the outage gain and costs 0.03 / 0.10 m. (2) Not fixed: the `pva_message` initialization
  leaves a 2 s gap before the first IMU sample (the PVA passes the controller's buffer); the solution falls at
  19.5 m/s three seconds after the start.
- 24 suites / 272 tests. Default mode unchanged: parity dump byte-identical, acceptance 24 / 24, the 61,772
  output messages identical to v0.2.11.
- Not done: an IMU error model that holds under driving (it would let the zero velocity weigh what it is
  worth), the receiver's time offset, adaptive measurement weighting, tightly coupled GNSS. The detector is for
  road vehicles: what glides at constant velocity looks like standing.

## 2026-11-16 — v0.2.11: alignment in motion; the reorder window of v0.2.10 corrected

- Asked for: alignment in motion. Until now the heading came from the configuration file or a PVA message; the
  VN-100 cannot find north standing still (turn-on bias 0.16 deg/s about the vertical, forty times the Earth's
  rate). Prototyped in Python on the recorded drive first, then `inertial::InMotionAlignment`, initialization
  `in_motion_align`, `InMotionAlignmentConfig` (DESIGN.md §7.24): the attitude from the receiver's velocity and
  the IMU as Wahba's problem, with the gyro and accelerometer biases estimated along; no assumption about the
  mounting or the direction of travel; hand-over at the last IMU sample with covariance and biases.
- Recorded drive, switched on at 64 times (`tools/run_alignment_matrix.py`, `docs/ALIGNMENT_MATRIX.md`): aligned
  64 of 64, in 18 s median (42 s for 90 %; longer only while the car stands), heading 1.6 deg RMS (3.8 worst)
  with a sigma of 2.6, tilt 0.3 deg. From position fixes alone: 30 s, 2.7 deg (5.2).
- Found on the way: (1) the receiver's velocity is 250 ms old, not the 150 ms the example configurations give
  both channels (3.1 deg RMS at 150 ms, 1.6 at 250, 2.5 at 350). (2) Leaving the accelerometer bias out, as the
  first version did, lets 3 milli-g pass for a vertical gyro bias on a gentle drive: 12.4 deg off with 3 claimed
  (`InMotionAlignment.TheGyroBiasHasToBeEstimatedAlong`). (3) **v0.2.10 was wrong about
  `measurement_reorder_s`**: a window not longer than the controller's 2 s buffer orders nothing that passed the
  buffer, and the released `pos_ins_full.json` dropped fixes with the recommended 0.5 s. Now a WARN at start-up,
  2.5 s in the documents and the regenerated examples (`Filter.ReorderWindowMustExceedTheControllerBuffer`); the
  aiding table moved by at most 0.4 m (worst cases; the means by 0.2 m).
- After the hand-over, switched on under way, the vertical gyro bias is still unknown and the plain filter's
  heading drifts for a minute (RMS 2.4 deg after 10 s, 3.1 after 30 s, 1.0 after 120 s; 9 to 11 deg worst). With
  the ground-vehicle constraints it holds (1.2, 1.3, 0.6 deg). `configs/pos_ins_align_in_motion.json` and
  `configs/pos_ins_ground_vehicle.json` (alignment and constraints: receiver and IMU, nothing else).
- 24 suites / 262 tests (new suite `alignment_in_motion`, 16). Default mode unchanged: parity dump
  byte-identical, acceptance 24 / 24, the 61,772 output messages identical to v0.2.10.
- Not done: zero-velocity / zero-rate updates from a stationarity detector (they would give the vertical gyro
  bias at every stop), an estimate of the receiver's time offset, the heading / gyro-bias correlation in the
  hand-over, adaptive measurement weighting, tightly coupled GNSS.

## 2026-11-15 — v0.2.10: fusion core, measurement order and odometry with calibration states

- Asked for: a filter that performs with GNSS, a second (military-code) receiver, IMU, odometry and more.
  Measured first (`docs/AIDING_MATRIX.md`): the IMU alone ends a 60 s GNSS outage 25 m off (57 m worst), a 120 s
  one 131 m (291 m). And two defects: (1) any measurement stream faster than the receiver's time bias makes the
  filter drop the receiver's fixes as "old" (2493 of 2602, at DEBUG level); (2) body velocity without calibration
  states degrades the solution with the receiver present (3.3 m against 1.55 m; 7.9 m for a wheel speed with a
  2 % scale error).
- `measurement_reorder_s`: measurements wait behind a window and are applied in time order; the discard is a
  WARN. `pinson_odometry`: body velocity with scale-factor and pitch / yaw misalignment states, for wheel speeds,
  three-axis sensors and the ground-vehicle constraints alike. `SensorDegradationConfig` can emulate a wheel
  odometer. DESIGN.md §7.23.
- Result with a wheel speed and the constraints: 4.9 m (10.6) after 60 s, 7.4 m (19.9) after 120 s, the filter's
  sigma within a factor 1.4 of the error, 1.58 m with the receiver present. Constraints alone: 13.3 / 15.6 m.
  With a second receiver and the barometer the loss of the first costs nothing (1.2 to 1.8 m).
- `tools/run_aiding_matrix.py` (the table, and `--write-configs` for `configs/pos_ins_odometry.json` and
  `configs/pos_ins_full.json`). 23 suites / 245 tests. Default mode unchanged: parity dump byte-identical,
  acceptance 24 / 24, the 61,772 output messages identical to v0.2.9.
- Not done: alignment in motion (the heading still comes from the config or a PVA message), zero-velocity
  updates from a stationarity detector, robust or adaptive measurement weighting, tightly coupled GNSS. A
  military-code receiver's own interface is not here either: to the filter it is a position / velocity source.

## 2026-11-15 — v0.2.9: opt-in handling of live IMU data

- Review of the inertial for live sensors (DESIGN.md §7.22). Six defects, all inherited from NavToolkit and all
  left as they are by default: a lost sample is mechanized as free fall (0.098 m/s per lost 10 ms sample), one
  future-stamped sample strands the inertial, a SAMPLED IMU message throws out of `Filter::push`, one NaN
  poisons every later solution, `estimated_dt()` never forgets a gap, and the RPY interpolation fails at pitch
  ±90°. Each has a key in `inertial_config` (`imu_gap_policy`, `max_imu_gap`, `sampled_imu`,
  `reject_non_finite`, `dt_window`, `attitude_interpolation`); `ImuRobustness::live_sensor()` sets them all.
  The two Earth-rate constants and the per-sample allocations were measured and left alone.
- Not opt-in: a non-positive or non-finite `expected_dt` / buffer length is refused with an ERROR, and
  interpolation next to a solution without a quaternion no longer reads an empty optional.
- Verified: 23 suites / 238 tests, also under ASan + UBSan. Default mode byte for byte that of v0.2.8 (parity
  dump and all 61,772 acceptance output messages, `tools/compare_logs.py`); acceptance 24/24 with the default
  and with the live settings. 60 s log with 4 % of the IMU messages lost: 5.3 m vertical error by default,
  0.06 m with the live settings.
- Next: the `TimeAdjuster` of the standard configs turns a lost sample into a 10 ms time slip before the
  inertial sees it, and the mediator's publish timer latches onto a future-stamped message. Neither is changed
  here.

## 2026-10-12 — v0.2.8: the push Filter no longer retains solutions behind a callback

- `Filter::set_solution_callback` replaces the `take_solutions()` queue by default (`also_queue = true` keeps
  both). Found by pnt-edge's first 8 h soak: the daemon used the callback only and the queue grew by one
  solution copy (about 1 kB) per second, 28 MB over the run. Test extended; C ABI (poll only) unchanged.

## 2026-10-11 — v0.2.7: cobra_version() reports the project version

- `cobra_version()` was a literal left at 0.2.0 since the C ABI landed; it now comes from the meson project
  version. The project version is bumped with every tag from here on.

## 2026-10-10 — v0.2.6: log sink

- `utils::set_log_sink` / `set_log_to_console`: a process-wide sink receives every message printed through
  `print_message` (a host daemon keeps the recent log in memory for its web page). 1 test.

## 2026-10-06 — v0.2.5: Allan tool as a library

- `tools/imu_allan.py` exposes `analyze(log, imu_channel, truth_channel)`; without a truth channel the static
  segments come from the gyro norm. Used by the integrity repository's per-installation tuning procedure.

## 2026-10-06 — Measured VN-100 preset and the Allan tool, v0.2.4

- `tools/imu_allan.py`: static segments from the truth channel's speed, overlapping Allan deviation per axis of
  specific force and angular rate, white-noise densities at τ = 1 s, Allan floors, bias wander across the stops.
  On the example log: 128 s static start plus twelve stops; gyro white noise 0.5–1.2e-4 rad/s/√Hz (preset
  6–10e-4), accelerometer 2–2.7e-3 m/s²/√Hz with engine vibration (preset 3.9e-6), Allan floors 1–2e-5 rad/s and
  3–6e-4 m/s², gyro mean wander across 40 minutes below 1.5e-4 rad/s (heading changes included).
- Preset `vn100_measured` with those values (initial sigmas unchanged). The default stays `vn100_corrected`:
  acceptance limits and the Python comparison are tied to it, and the Cobra tuning absorbs installation effects.
  Consumers that need a consistent free-inertial covariance (integrity monitoring) select the measured preset.

## 2026-10-04 — v0.2.3: innovation covariance in the probe

- `StandardFusionEngine::InnovationStatistic` also carries S = H P Hᵀ + R, so a consistency monitor can
  normalise innovation sequences itself. 22 suites / 207 tests green.

## 2026-10-04 — Hooks for out-of-tree plugins, v0.2.2

- `RegistryConfig`: a generic JSON config type whose values go verbatim into a registry group, so a plugin that
  lives outside this library can be configured from the same config file (DESIGN §7.14).
- `app::cobra_run_main`: the cobra_run command line as a library function; `apps/cobra_run.cpp` is one line.
- `StandardFusionEngine::innovation_statistic` (side-effect-free NIS probe) and `set_registry_reporting`
  (quiet shadow engines); `clone()` now carries the gate configuration (DESIGN §7.15). `update()` builds its
  full-state model through a shared helper; outputs unchanged (short acceptance 12/12 both modes).
- Two more orchestration hooks virtual (`send_message_as_aux_data`, `send_inertial_aux_to_vsbs`).
- 22 suites / 207 tests green. Tagged `v0.2.2`. The v0.2.1 CI and release runs passed
  (`pntos-cobra-v0.2.1-linux-x86_64.tar.gz`).

## 2026-10-04 — Degradation extensions, orchestration extension points, plugin registration, v0.2.1

- **Orchestration extension points**: `StandardOrchestrationPlugin` is no longer `final`; its per-step methods
  (`initialize_filter`, `propagate_to_time`, `send_inertial_aux_to_*`, `apply_inertial_feedback`,
  `perform_measurement_update`) are `protected virtual`, so a derived orchestration can observe or replace any
  step while reusing the standard loop (DESIGN §7.9). Test: a counting subclass sees every step.
- **Plugin registration**: `app::register_orchestration` / `app::register_extra_plugin` let a library linked
  against Cobra add orchestrations and utility plugins selectable from config files (`app.orchestration`,
  `app.extra_plugins`) without editing the app builder (DESIGN §7.14).
- **Sensor degradation**: `position_ramps` (slow pulls, optional duration) and a **derived position source**
  (a second, independent position channel synthesised from truth with its own noise and rate) so two-source
  apps can be exercised from the single-GNSS example log. `configs/pos_ins_two_sources.json` fuses GNSS and the
  synthetic `/synthetic/cell/position` source with a FOGM bias block each and a 0.999 gate.
- **Degraded matrix** gained five two-source rows (clean, 2 m/s pull, 0.5 m/s pull, 30 m step, second-source
  pull). Gating handles the step (1 rejection) and the 2 m/s pull (2000 rejections, 2.7 m std); the **0.5 m/s
  pull defeats gating alone** (solution follows GNSS to hundreds of metres) — the case that needs solution
  separation, which lives in a separate repository.
- Verified: 22 suites / 205 tests green; acceptance 24/24 both modes; degraded matrix 16/16 with re-derived
  limits. Tagged `v0.2.1`.

## 2026-10-04 — Hygiene: sanitizer + clang-tidy CI job, release workflow, CONTRIBUTING, v0.2.0

- `CONTRIBUTING.md` (the loop, where each kind of addition goes, style), `.clang-tidy` (a conservative check set,
  advisory in CI until reviewed), `.clang-format`.
- CI gained a `sanitizers` job: ASan + UBSan build and all suites (clean locally: 22 suites), then clang-tidy on
  the library sources. `release.yml` builds a release configuration on a `v*` tag, runs the suites, installs into
  a staging tree and attaches `pntos-cobra-<tag>-linux-x86_64.tar.gz` (+ sha256) to a GitHub release.
- Tagged `v0.2.0`: the release workflow published `pntos-cobra-v0.2.0-linux-x86_64.tar.gz` (+ sha256) at
  https://github.com/rreper/Cobra/releases/tag/v0.2.0; the CI run with the sanitizer job passed.

## 2026-10-04 — CI live on GitHub Actions

- Token given the `workflow` scope; `ci/github-ci.yml` moved to `.github/workflows/ci.yml`. First run failed
  because `*.log` in `.gitignore` had excluded `testdata/example_60s.log`; now tracked explicitly. Second run
  green: build, 22 suites, short acceptance (compiled apps, config files via `cobra_run`, push API) and the C
  example. Session notes (repo and shared page) updated with the roadmap execution summary.

## 2026-10-04 — Roadmap Phase 3: push API, C ABI, network LCM, CSV, packaging

- **Push API** (`app/Filter.hpp`, DESIGN.md §7.18): `StandardControllerPlugin::start()/stop()` split out of
  `take_control()`; `PushTransportPlugin`; `cobra::Filter` with `push`, `take_solutions`, a solution callback,
  `solution(time)`, `registry`, `stop`. `run_app_via_push` (flag `--via-push`) is the LCM log runner on top of it:
  **all 12 apps × 2 modes identical to the log transport to every printed digit.**
- **C ABI** (`capi/cobra.h`, §7.19): create / push imu, position, velocity, raw LCM / poll, solution_at / stop,
  destroy, last_error; `examples/c/run_log.c` replays the example log from plain C (`build/examples/c_run_log`).
- **Network LCM** (`transport/LcmUdpTransportPlugin.hpp`, §7.20): the LCM UDP multicast wire protocol (short and
  fragmented datagrams) without liblcm; `idle_timeout_sec`, `output_file`; `tools/lcm_log_player`;
  `configs/pos_ins_network.json`; `tools/run_network_acceptance.py`: the full log at 25× speed reproduces pos_ins
  exactly (2570 epochs, pos std 0.916 / 0.706 / 1.39 m, yaw std 0.831°), 262 806 datagrams, no loss.
- **CSV transport** (`transport/CsvTransportPlugin.hpp`): header-matched IMU / position / velocity files merged
  in time, solutions to CSV; `tools/lcm_to_csv`; `configs/pos_ins_csv.json`; the 60 s run equals the LCM run
  (times identical, latitudes within 2 cm from the diagonal-only covariance).
- **Packaging** (§7.21): versioned library, `meson install` (headers, `cobra_run`, geoid, configs), `pntos-cobra.pc`,
  `docs/GETTING_STARTED.md`. Project version 0.2.0.
- Verification: 22 suites / 202 tests green (new `filter`, `transports`); acceptance 24/24 compiled, 24/24 via push;
  CI script PASS (compiled, config files, push); network acceptance PASS.
- Deferred: ROS 2 adapter (no ROS environment), UI server plugin. The GitHub token still needs the `workflow`
  scope before `ci/github-ci.yml` can move to `.github/workflows/`.

## 2026-10-04 — Roadmap Phase 2: innovation gating, degraded-sensor matrix, geoid

- **Innovation gating** (`StandardFusionEngine::set_innovation_gate`, DESIGN.md §7.15): chi-square gate per
  processor from `MeasurementProcessorConfig::innovation_gate_probability` or the `FusionEngineConfig` default;
  counters in registry group `fusion/gating`; off by default. A 50 m outlier injected into pos_ins is rejected
  (1 rejection, solution unchanged).
- **Sensor degradation** (`SensorDegradationConfig` / `SensorDegradationPreprocessor` in the extras plugin, §7.16)
  and `tools/run_degraded_matrix.py`: 11 rows (3 IMU grades, 5 GNSS conditions, 3 sensor sets) through
  `cobra_run`, limits recorded in `docs/limits_degraded.json`, table in `docs/DEGRADED_MATRIX.md`. All rows run
  clean; e.g. consumer-grade IMU emulation raises yaw std from 0.83° to 4.1°, 0.1 Hz GNSS raises position std
  from 0.92 to 1.12 m north.
- **Geoid** (`nav::Geoid`, §7.17): EGM96 15' grid bundled as `data/egm96_15min.bin` (2 MB, `tools/make_geoid.py`);
  `AltitudeMeasurementProcessor` converts MSL to HAE; `configs/pos_ins_baro.json` runs the barometer through it.
  Deviation 11 (MSL rejected) resolved.
- `--no-record-input` on every app and `cobra_run`: acceptance logs shrink from 476 MB to ~1 MB each; the
  acceptance tools use it (a full run is 53 MB instead of 11 GB). The orchestration now warns when a preprocessor
  config names an identifier no loaded plugin provides (Python skips silently).
- Verification: 20 suites / 195 tests green; acceptance 24/24 (12 apps × 2 modes); CI script compiled + config
  files PASS; degraded matrix 11/11.

## 2026-10-04 — Roadmap Phase 1: config files, presets, corrected default, CI

- `docs/ROADMAP.md` and `docs/SESSION_NOTES.md` written (shared copies on claude.ai).
- **Config files.** `config/JsonConfig.hpp` serialises every config class to and from JSON with the Python class and
  field names; `app/AppBuilder.hpp` builds the plugin set from an `AppSpec`; `apps/cobra_run config.json` runs any
  app; the compiled apps go through the same builder and `--dump-config` wrote `configs/*.json` (12 files).
  `--dump-registry` shows the registry contents are byte-identical between a compiled app and its config file in
  both modes, and `run_acceptance.py --runner` reproduces every acceptance number.
- **Presets.** `presets/Presets.hpp`: `vn100`, `vn100_corrected` and six datasheet-derived IMU models
  (`imu_from_datasheet`), five GNSS receiver profiles; `cobra_run --list-presets`.
- **Corrected Pinson-Q is now the default.** Yaw gyro random walk retuned 6.7e-5 → 6.0e-4 rad/√s for the corrected
  rotation (68.2 % of yaw errors inside 1σ, yaw std 0.831° vs 0.845° untuned and 0.805° legacy); the mode flag swaps
  rotation and tuning together, so `--legacy-q` still reproduces Python exactly. Corrected-mode limits derived once
  into `docs/limits_corrected.json`. **Acceptance: corrected 12/12, legacy 12/12, through config files 12/12 + 12/12.**
- **CI.** `.github/workflows/ci.yml`, `tools/ci_acceptance.py`, `tools/log_stats` (C++), `testdata/example_60s.log`.
- 20 suites, 191 tests green (new `json_config`). JSON for Modern C++ v3.11.3 vendored (MIT).

## 2026-10-03 — All 12 log-replay apps ported and passing; legacy Pinson-Q is the app default

- **Default flipped to the Python-compatible Pinson-Q rotation** (`--legacy-q`; `--corrected-q` restores the
  from-a-copy rotation). Rationale: the goal is parity with Python; the corrected mode needs a yaw gyro
  random-walk retune before it can be the default (DESIGN.md §9.6).
- **Remaining plugins ported**: tutorial state model / orchestration / UI summary plugin (`tutorial/`), the
  diagnostic log plugin with a dependency-free HDF5 writer (`diagnostics/`, `utils/hdf5.hpp`, files verified
  with h5py: `estimate (5181, 18)`, `sigma`, `state_labels (1, 18)`, `time`), and the extras zero-velocity
  preprocessor (`extras/`). New apps: `pos_ins_record_states`, `tutorial_pos_ins`, `tutorial_pos_vel_ins`,
  `pos_ins_zerovel2d`. New configs: `ZeroVelocity2dGeneratorConfig`, `TutorialOrchestrationConfig`,
  `UiLogPlottingConfig`. `Pinson15NedBlock` gained `tutorial_model`.
- **outage_sim gap closed (307 → 305.5 m = Python).** Epoch-by-epoch comparison against a Python run showed
  the filters identical to 1 cm throughout the 600 s outage; only the 1 Hz solution grid differed in phase
  (up to 0.4 s), because Python's mediator reads the IMU timestamp after the time adjuster has replaced it.
  `utils/effective_time.hpp` lets the orchestration report that time; the mediator uses it. Former deviation
  #14 is resolved and every app now produces Python's epoch count (2570 / 2593).
- **Acceptance (legacy mode): 12/12 PASS**, corrected mode 3/12 (tilt limits). 19 suites, 186 tests green.
- Not portable here: `pos_ins_network` (network LCM), `pos_ins_ros`, `pos_ins_ui`.

## 2026-10-03 — Yaw investigation closed, seven more apps, two-mode acceptance

- **Root cause of the yaw gap found and proven.** Parity harnesses showed every numerical component (nav helpers,
  Pinson dynamics at a rotated attitude, processors, VSB, error-state application, alignment, buffered inertial
  incl. resets) identical to Python/navtk at machine precision; a propagate/update trace of both apps on a 60 s
  cut showed identical step sequences and inputs with the first divergence inside the second Pinson
  propagation. Cause: deviation #1 (Python re-rotates its stored process-noise matrix in place every call, which
  inflates yaw process noise). `PinsonStateBlockConfig::legacy_q_rotation` (app flag `--legacy-q`) reproduces it;
  with it pos_ins matches Python to three digits (yaw std 0.805°, 68.1 % within 1σ) and passes the Python limits.
  The Joseph form was ruled out first (identical results with `--no-joseph`).
- Apps added: `pos_vel_ins`, `posvel_ins`, `pos_ins_leverarm`, `pos_ins_bodyvel`, `outage_sim`, `pos_ins_vsb`,
  `direction_to_points` (shared `apps/standard/app_common.hpp`). LCM decode added for
  `measurement_direction_3d_to_points`; the vendored `type_image_feature` was fixed to the log's int16
  descriptor layout (the pinned aspn-generated commit changed it to byte).
- `tools/run_acceptance.py` replicates the Python integration-test checks (std/max limits, sigma coverage, point
  count ±5, NaNs, start/end) for every app in corrected and legacy modes; `docs/TEST_MATRIX.md` shows both columns.
  **Legacy mode: 7/8 apps pass; `outage_sim` misses the position-std limit by 0.3 % (307 m vs 306 m after a
  600 s GNSS outage). Corrected mode: 2/8 pass (leverarm, outage_sim); the rest fail only the tilt (and for the
  velocity apps, velocity) limits that were tuned on the Python behaviour.**
- Tools: `parity_dump`/`parity_check.py`, `inertial_parity_dump`/`inertial_parity_check.py`,
  `trace_python_pos_ins.py`, `PNTOS_TRACE_FILE` hook. 16 suites, 170 tests green.
- Not ported: `pos_ins_record_states` (HDF5 diagnostics plugin), the two tutorial apps (tutorial plugins),
  `pos_ins_zerovel2d` (extras preprocessor).

## 2026-10-03 — Test pass/fail matrix

- `tools/test_matrix.py` generates `docs/TEST_MATRIX.md` from meson's JUnit output: suite table with Python
  origins, every individual test with its result, Python-test coverage matrix, and the 13-app acceptance matrix
  (with the latest pos_ins numbers when given the comparison JSON). Current: 16 suites, 169 tests, all green;
  2 of 13 apps ported (dummy, pos_ins).

## 2026-10-03 — LCM log transport, apps, and pos_ins acceptance on the example log (16/16 suites green)

- Decided against liblcm: `transport/LcmLog` reads/writes the event-log format directly and the lcm-gen C++
  classes are vendored header-only (`third_party/`, see NOTICE.md). `LcmConversions` covers IMU, position,
  velocity, PVA, altitude, barometer; `LcmLogTransportPlugin` replays with channel filter, UI source gate,
  output recording and the end-of-log shutdown flag. Fingerprints match the venv-generated Python types
  (verified by decoding the real example log).
- Apps: `apps/dummy/minimal`, `apps/standard/pos_ins` (configs identical to the Python app).
  `tools/compare_to_truth.py` computes NED/velocity/RPY RMS against `/sensor/ins-d/pva`.
- **Acceptance:** on the 43-minute example log the C++ pos_ins gives position RMS 0.917/1.233/1.668 m vs Python
  0.921/1.235/1.667 (same tool), velocity 0.0850/0.0939/0.0423 vs 0.0837/0.0930/0.0431 m/s, attitude
  0.090/0.079/0.847° vs 0.089/0.078/0.810°. Wall time 2.1 s vs 22.6 s; peak RSS 8 MB vs 196 MB. No WARN/ERROR
  logged during the run. Open follow-up: the 4.6 % higher yaw RMS (Joseph form vs `(I−KH)P` is the first suspect).
- Tests: `lcm_transport` (7). **16/16 suites, 169 tests green.** Pushed.
- Remaining (Tier 2/3, DESIGN.md §9.6): the other 12 apps' missing plugins (UI, diagnostics/HDF5 log, tutorial,
  Buscat), network LCM transport, ROS, geoid model for MSL altitude, C ABI shim.

## 2026-10-03 — Preprocessors ported (15/15 suites green)

- `StandardPreprocessorPlugin` with the six preprocessors; `utils::with_time_of_validity` added so timestamp
  rewrites work on any timed ASPN type without mutating the original message.
- Tests: `preprocessors` (9), ported from `test_preprocessor_plugin.py`. **15/15 suites, 162 tests green.**
- Next: LCM log transport, then the apps (`dummy/minimal`, `standard/pos_ins`) and acceptance on the example log.

## 2026-10-03 — Inertial mechanization and alignment ported into Eigen (14/14 suites green)

- Decision: port NavToolkit's mechanization/alignment (~2,300 lines) into Eigen instead of adding NavToolkit
  (xtensor + BLAS + Python-at-configure + spdlog + data download) as a subproject. DESIGN.md §9.1/9.2 lists the
  file mapping and the conventions (C_nav_to_sensor in NavSolution, relative perturbation in the RPY Jacobian).
- Added `inertial/Mechanization` (standard NED mechanization, error model, `Inertial`), `inertial/BufferedImu`
  (time-sorted rings, interpolation, resets with re-propagation, force/rate queries, no-reset-since solutions),
  `StandardInertialPlugin`; `initialization/Alignment` (`ImuModel`, static + manual-heading alignment,
  gyro-compass), the four initialization plugins and `PvaMessageInitializationConfig`.
- Tests: `inertial` (5) and `initialization` (5), ported from the Python tests plus closed-form checks.
  **14/14 suites, 153 tests green.** Pushed.
- Next: preprocessors + StandardPreprocessorPlugin, then the LCM log transport and the pos_ins app.

## 2026-10-03 — Design and testing guides; sanitizer run

- Added `docs/DESIGN.md` (build, layout, type mapping, every component with diagrams, the complete list of
  deviations from Python, the roadmap with Python-source pointers, recipes, conventions) and `docs/TESTING.md`
  (running, suite inventory with Python origins, mocks, goldens, how to port a Python test, baseline
  reproduction, acceptance plan, unported tests, debugging). README links both.
- Whole suite run under ASan + UBSan (`meson setup build-asan -Db_sanitize=address,undefined`): clean after
  fixing one shared_ptr cycle in `DummyOrchestrationPlugin` (it stored a pointer to itself when handed the full
  plugin list).

## 2026-10-03 — Orchestration plugin ported (12/12 suites green)

- Added `StandardOrchestrationPlugin`, the orchestration utilities (`apply_error_states`, best / dead-reckoning
  solutions, inertial set-up, `SolutionCache` replacing the Python `Cache`/`CacheEntry` classes with three typed,
  time-validated entries), and the remaining configs: `InertialConfig`, `FeedbackConfig`, `PreprocessorConfig`
  (+ baro / downsampler / IMU rotator / time adjuster / time bias / outage), `ManualAlignmentConfig`,
  `StaticAlignmentConfig`, `ManualHeadingAlignmentConfig`, `StandardOrchestrationConfig`.
- Config note: nested state block / processor / VSB / preprocessor configs are stored by group pointer (same
  registry layout as Python); on read only the base fields come back and the providers read their full configs.
- Tests: `test_orchestration` (12) with a mock inertial and mock initializer standing in for the not-yet-ported
  NavToolkit plugins; covers init, config round trip, VSB-chain aux routing, one channel to many processors,
  outage propagation, alignment after N messages, request_solutions in all forms, and an end-to-end position
  update with inertial feedback (bias correction, reset, Pinson states zeroed, covariance behaviour).
- **12/12 suites green.** Pushed to github.com/rreper/Cobra.
- Next: NavToolkit as a meson subproject, `StandardInertialPlugin` (mechanization) and the alignment plugins
  (manual, static, manual-heading), then preprocessors and the LCM log transport, then apps.

## 2026-10-03 — Controller layer ported (11/11 suites green)

- Added `ControllerConfig`, `BufferMode`/`Stream`/`StreamConfig` (+ `default_stream_config()`), plugin sorting and
  validation helpers, `StandardMessageStreamConfig`, `StandardMediator` with a shared `MediatorContext` (replaces
  Python's class-attribute singleton), `ExitEvent`, a slim `UiMediatorInterface` (per-source enable gate via the
  `ui/channel/<source>` registry group; rate/jitter stats not ported yet), `StandardControllerPlugin` (SIGINT via
  `install_sigint_handler()`, exit code via `exit_code()` instead of `sys.exit`), and the dummy plugins
  (mediator, stream config, orchestration, transport, controller).
- Tests: `test_message_stream_config` (9, port of the Python 8 + source-specific overrides) and `test_controller`
  (12: mediator routing/buffering/publishing/broadcast/logging/UI gate, exit event, controller wiring, missing-plugin
  validation, ready_to_shutdown flag, error exit code, dummy controller end to end, config round trips).
- GitHub: pushed to https://github.com/rreper/Cobra (classic token with `repo` scope, stored in git's credential
  store). All commits are now mirrored there; every further commit is pushed.
- Next: `StandardOrchestrationPlugin` + orchestration utils + remaining configs, preprocessors, inertial/alignment.

## 2026-10-03 — Fusion engine, VSB manager, fusion plugin ported (9/9 suites green)

- Added `VirtualStateBlockManager` (deep-copyable forest of VSB nodes with path/root caches),
  `StandardFusionEngine` (block bookkeeping, block-diagonal dynamics assembly, full-width measurement models,
  VSB routing, clone-based `peek_ahead`, diagnostics save to the `diagnostics` registry group) and
  `StandardFusionPlugin` + `FusionEngineConfig`.
- Tests: `test_vsb_manager` (4, port of the Python manager tests + deep-copy check) and `test_fusion_engine`
  (8, new: add/get/set/remove with cross-covariances and re-indexing, non-generic EWC rejection, propagate against
  FOGM/constant closed forms, update through real and virtual blocks, aux routing, peek_ahead/clone isolation,
  registry diagnostics, plugin config). **9/9 suites green.**
- New finding (COBRA_ANALYSIS §12 #14): Python `update()` slices H by the real block width for virtual blocks,
  which breaks for size-reducing VSBs; the port uses the virtual width and the chain-rule Jacobian.
- GitHub: `gh` is authenticated but the fine-grained token cannot create repositories and is scoped to a few
  existing repos; no SSH key. Remote `origin` is preset to `https://github.com/rreper/pntos-cpp.git`. Rich needs
  to create that repo (or grant the token "Administration: write" / run `gh auth refresh -s repo`), then
  `git push -u origin main` works.
- Next: `StandardMediator` + `StandardMessageStreamConfig` + `StandardControllerPlugin`, dummy plugins,
  `StandardOrchestrationPlugin` and its configs, preprocessors.

## 2026-10-03 — State-modeling layer ported and tested; repo pushed to GitHub

- Added `nav::` (NavToolkit formulas in Eigen), `utils::aspn` helpers, all state-block / measurement-processor /
  virtual-state-block configs, `Pinson15NedBlock`, 9 measurement processors, 2 virtual state blocks, and
  `StandardStateModelingPlugin`.
- Ported `test_state_modeling.py` (17 tests) and the VSB half of `test_virtual_state_blocks.py` (5 tests).
  The Pinson `generate_dynamics` golden Phi/Qd from the Python test match at rtol 1e-5 / atol 1e-20.
  A finite-difference check covers the `PinsonErrorToStandard` jacobian.
- Suite: **7/7 gtest binaries green** (smoke, api, ekf_fusion_strategy, simple_state_blocks, registry,
  state_modeling, virtual_state_blocks).
- Deviations from Python recorded in COBRA_ANALYSIS §13: Pinson Q no longer mutated in place (regression test
  `QIsNotMutatedAcrossCalls`); MSL altitude unsupported until a geoid model is wired in (logs error, returns nullopt).
- GitHub: repository created and pushed (see README for URL). Progress and docs are pushed with every commit.
- Next: `VirtualStateBlockManager`, `StandardFusionEngine` + `StandardFusionPlugin` (clone-based peek_ahead),
  controller/mediator/message-stream, orchestration, remaining configs, preprocessors, dummy plugins.

## 2026-10-03 — Repo created, build scaffold, Tier 1 started

- Local git repo initialized at `~/orin/work/pntos`; Cobra added as a submodule; analysis moved to `docs/`.
- Decision: **Eigen** for all filter math (per Rich). Messages: `aspn23_eigen` from aspn-generated.
  Build: **meson** (both upstreams are meson projects; they drop in as wraps).
- Decision: C++ API mirrors the Python `pntos.api` / pntOS-C 1:1 as abstract classes under `pntos::api`;
  a C-ABI shim (`PntosManagedMemory` etc.) is deferred to a later tier.
- Next: pull eigen/gtest/aspn-generated/pntos-c wraps, write the API headers, registry, logging, EKF strategy,
  fusion engine, state blocks, measurement processors, dummy plugins, and port the first unit tests.

## 2026-10-02 — Analysis of the Python original

See `docs/COBRA_ANALYSIS.md`. 366/367 unit tests and 13/13 integration apps reproduced; profile and
baselines recorded; latent Pinson15 process-noise mutation bug found.
