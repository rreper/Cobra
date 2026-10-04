# Progress log

Newest first. Each entry: what changed, what was verified, what is next.

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
