# Cobra C++ Roadmap

As of 2026-10-04. Shared copy: the "Cobra C++ Roadmap" page on claude.ai (same content).

Goal: turn the C++ port of Cobra (pntOS) from twelve hard-coded example apps into a configurable
starting point for users with their own GNSS receivers, IMUs and aiding sensors. Three phases, each
gated by the full unit suite and the 12-app acceptance run before a push.

## Where we stand

The port reproduces the Python reference on the 43-minute example log: all 12 log-replay apps pass the
Python integration-test limits in the default Python-compatible mode, with Python's exact epoch counts,
at roughly ten times the speed.

| Item | State on 4 Oct 2026 |
|---|---|
| Unit tests | 19 suites, 186 tests, all green |
| Apps ported | 12 of 15 Python integration apps (all that replay a log) |
| Acceptance, legacy Pinson-Q (default) | 12 / 12 pass |
| Acceptance, corrected Pinson-Q | 3 / 12 pass (tilt limits were set with the inflated yaw noise) |
| Wall time per app | 1.4 to 5.5 s, Python 22 to 40 s |
| Dependencies | Eigen, aspn-generated, gtest only. No NavToolkit, liblcm or libhdf5 |
| Not ported | network LCM app, ROS app, UI server app |

What is missing for other sensors: every app hard-codes one vehicle, one VN-100 IMU, one u-blox receiver
and one dataset in C++. There is no config file, no sensor preset, no outlier rejection, and no way to feed
the filter other than an LCM log.

## Phase 1, Foundation — DONE 2026-10-04

Makes the repository a starting point at all: one binary, one config file, named sensor presets, a
defensible default filter, and CI that proves it on every push. Gate held: 20 suites green, 12 / 12 in both
modes from config files (identical to the compiled apps), CI workflow committed (PROGRESS.md 2026-10-04).

| Work item | What it delivers | Done when |
|---|---|---|
| Config file loader and generic runner | `cobra_run config.json` builds transport, alignment, IMU model, state blocks, processors and preprocessors from a file. The 12 apps become 12 example config files. | Every app's config file reproduces that app's acceptance numbers |
| Sensor presets | Named IMU models (VN-100, STIM300, HG1700, HG4930, ADIS16488, a consumer MEMS class) and GNSS receiver profiles (message types emitted, covariance handling, time bias, reference frame). | Presets selectable by name in the config; unit test per preset |
| Corrected Pinson-Q as the default | Retune the yaw gyro random-walk sigma so the corrected filter is consistent (68 % of yaw errors inside one sigma), re-derive the limits, keep `--legacy-q` as a reproduction switch only. | 12 / 12 pass in corrected mode with the retuned limits |
| CI | GitHub Actions: build, unit suite, and the acceptance run on a short cut of the example log. | Green badge on main |

Gate to Phase 2: unit suite green, 12 / 12 acceptance from config files, CI running.

## Phase 2, Robustness

Proves the presets instead of asserting them, and makes the filter survive real receivers.

| Work item | What it delivers | Done when |
|---|---|---|
| Innovation gating | A chi-square gate per measurement processor, configurable threshold, rejected-measurement counters in the registry. The Python has none. | Gate unit tests; an injected 50 m position jump is rejected and logged |
| Degraded-sensor test matrix | A noise-and-bias injector preprocessor plus the existing downsampler and outage preprocessors emulate tactical, industrial and consumer IMU grades and 1 Hz, 5 Hz and gapped GNSS from the one dataset, with truth. | Acceptance table with a row per grade and rate, limits recorded |
| Geoid model | MSL altitude support for barometer and altitude users (currently rejected). | Altitude processor accepts MSL with a geoid lookup |

Gate to Phase 3: unit suite green, acceptance across the degraded matrix recorded in the test matrix document.

## Phase 3, Integration

Lets integrators embed the filter without LCM logs.

| Work item | What it delivers | Done when |
|---|---|---|
| Library push API | `cobra::Filter f(config); f.push(message); f.solution(t)`. Every transport becomes a thin adapter. | The LCM log runner is reimplemented on top of the push API with identical acceptance numbers |
| C ABI | A C header over the push API (pntOS-C style). | C example program links and runs the example log |
| Transport adapters | Network LCM (UDP multicast, no liblcm), CSV reader, then ROS 2 when a ROS environment is available. | `pos_ins_network` reproduces `pos_ins`; CSV example documented |
| Packaging and docs | Install targets, pkg-config file, versioning, a getting-started guide built around the config files and presets. | A new user runs their own log from the guide alone |

Gate: unit suite green, acceptance unchanged through the push API, CI green.

## Timeline

Phase 1 → gate (tests green, 12/12 from config files) → Phase 2 → gate (tests green, degraded matrix
recorded) → Phase 3. Before every push: full unit suite, then the 12-app acceptance run. No calendar dates
are attached: each phase ends when its gate holds.

## Decisions needed

- [ ] Who the first users are. Embedded C integrators move the push API and C ABI from Phase 3 into Phase 1.
- [ ] Config file format: JSON first (vendored single-header parser); YAML once a header-only parser is vendored.
- [ ] Whether corrected Pinson-Q becomes the default now or after the upstream authors respond. Plan: now, keeping the legacy switch.
- [ ] Additional datasets for the degraded-sensor matrix; without them it is synthetic degradation of the example log.

## Out of scope for now

- ROS 2 transport (no ROS environment on the build machine; a thin adapter once the push API exists).
- The UI server plugin and registry views.
- Buscat controller and other multi-process concurrency models.
- Per-vehicle tuning beyond presets.
