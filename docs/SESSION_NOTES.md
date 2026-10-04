# Cobra C++ Port Session Notes

As of 2026-10-04. Shared copy: the "Cobra C++ Port Session Notes" page on claude.ai (same content).

The full record of the sessions (2 to 4 October 2026) that analysed the IS4S Cobra Python reference
implementation of pntOS and rewrote it in C++20 with Eigen: what was decided, in what order it was built,
every Python bug and quirk found, how the last accuracy gaps were closed, and where everything lives.

## 1. Outcome

A C++20 port of Cobra that reproduces the Python reference on its own example dataset, with no NavToolkit,
liblcm or libhdf5 dependency, about ten times faster and twenty-five times smaller in memory.

| What | State on 4 Oct 2026 |
|---|---|
| Repository | github.com/rreper/Cobra, branch main, built with meson and ninja from the Cobra venv |
| Code | pntOS API as abstract C++ classes; every Cobra plugin used by the log-replay apps; 12 apps; 3 tool binaries |
| Unit tests | 19 GoogleTest suites, 186 tests, all green; ported from the Python pytest files plus new C++ coverage |
| Acceptance | 12 of 12 apps pass the Python integration-test limits in the default Python-compatible mode, with Python's exact epoch counts |
| Speed | 1.4 to 5.5 s per app on the 43-minute log; Python 22 to 40 s. pos_ins peak RSS 8 MB vs 196 MB |
| Python bugs found | 2 real bugs and 13 quirks (COBRA_ANALYSIS.md §12); the Pinson process-noise bug reported upstream |
| Documents | docs/DESIGN.md, TESTING.md, TEST_MATRIX.md (generated), PROGRESS.md, COBRA_ANALYSIS.md, ROADMAP.md, this file |

## 2. The Python original

Cobra is IS4S's reference implementation of pntOS, a plugin architecture for navigation filters. Everything
is a plugin behind a small API: controller, mediator, transport, registry, logging, fusion engine, fusion
strategy, state modeling, inertial, initialization, preprocessor, orchestration, UI and utility plugins. The
standard orchestration implements a closed-loop GNSS/INS filter: a 15-state Pinson error-state EKF in the NED
frame (position, velocity, tilt, accelerometer bias, gyro bias) wrapped around a strapdown inertial
mechanization, with first-order Gauss-Markov blocks for sensor errors, nine measurement processors, two
virtual state blocks, and inertial feedback after every update.

How it was analysed:

- The repository was cloned into a venv with its pinned dependencies (navtk, lcm, aspn23, h5py). The pytest
  suite was run and read; the ASPN 2023 message set and the pntOS-C header set were read to understand the
  contract the C++ had to honour.
- The twelve standard, tutorial and extras apps were run on the shipped example log: 474 MB, 43 minutes of a
  drive with a VectorNav VN-100 IMU at 100 Hz, a u-blox ZED-F9T receiver at 1 Hz (position, velocity, PVA), a
  barometer, simulated body-velocity and direction-to-feature channels, and an ins-d PVA truth channel.
- Baselines were recorded against truth. pos_ins: position RMS 0.92 / 1.24 / 1.67 m N/E/D, velocity
  0.084 / 0.093 / 0.043 m/s, attitude 0.074 / 0.092 / 0.811 deg roll/pitch/yaw, 2570 solution epochs, 22.6 s
  wall, 196 MB peak memory.
- Proof-of-concept probes isolated the numerics: the Pinson F and Q matrices at a rotated attitude, the EKF
  update arithmetic, the virtual-state-block Jacobians, and the buffered inertial. These probes became the
  golden values and the parity harnesses of the port.

Data flow of one GNSS epoch, as the port reproduces it: the transport decodes an LCM event into an ASPN
message; the mediator routes IMU messages immediately to the orchestration and buffers everything else for
2 s in time order; the orchestration runs the preprocessor chain (IMU rotation into the platform frame,
timestamp repair, a 150 ms GNSS time bias), feeds the inertial, propagates the filter in chunks of at most
`max_prop_interval` to the measurement time, updates, then resets the inertial to the corrected solution and
zeroes the error states; once a second of message time the mediator asks the orchestration for a solution and
the transport writes it to the output log.

## 3. Architecture and decisions

The port keeps Cobra's structure and names one-to-one so that the Python source stays the documentation of
intent, and departs only where C++ or correctness demands it. Every departure is numbered in DESIGN.md §8.

| Decision | Choice | Why |
|---|---|---|
| Language and libraries | C++20, Eigen 5.0.1, aspn-generated (pinned commit), GoogleTest; meson 1.12 and ninja from the Cobra venv | Rich asked for Eigen; everything else is a meson subproject |
| API layer | `include/pntos/api`: abstract classes mirroring pntos.api and the pntOS-C headers; a C ABI shim planned on top | Plugins written against the C++ API read like the Python ones |
| Messages | Immutable, shared by pointer | Safe sharing between buffer, orchestration and registry; the one Python behaviour this breaks (in-place preprocessing) is reproduced by the effective-time hook |
| Registry values | `std::variant` of string, string array, int64, bool, double, Matrix, Message; notify tokens | Same conversion table as the Python store |
| Mediator state | `MediatorContext` shared by every mediator of one controller | Testable, several controllers per process |
| Inertial layer | NavToolkit's mechanization, BufferedImu and alignment ported into Eigen | No binary dependency; verified against navtk at machine precision |
| LCM | Event-log format read and written directly; lcm-gen ASPN headers vendored | No liblcm; network transport deferred |
| HDF5 | A minimal writer for the diagnostic log | No libhdf5 and no sudo on the build machine; files verified with h5py |
| EKF | Joseph form and LDLT solve by default; Python arithmetic via `set_joseph_form(false)` | Robustness; parity tests use the Python arithmetic |
| Pinson process noise | Rotated from a copy when `legacy_q_rotation` is false; apps default to true | Parity with the shipped Python; see §6 |
| Threads | Single-threaded controller, log reader thread, recursive mutex around the mediator | Matches every app in use |
| Tracing | `PNTOS_TRACE_FILE` hook in the fusion engine and orchestration | Used to find the first diverging step against Python |

## 4. Build chronology

| Date | Step | Verified by |
|---|---|---|
| 2 Oct | Analysis of the Python original, baselines, COBRA_ANALYSIS.md | Python pytest suite and apps run in the venv |
| 3 Oct | Repository on GitHub, meson scaffold, API headers, registry, EKF strategy, simple state blocks | smoke, api, registry, ekf_fusion_strategy, simple_state_blocks |
| 3 Oct | State-modeling layer: Pinson15, nine processors, virtual state blocks, provider plugin | state_modeling, virtual_state_blocks with Python goldens |
| 3 Oct | Fusion engine with clone-based peek-ahead, VSB manager, fusion plugin | fusion_engine, vsb_manager (9 suites) |
| 3 Oct | Controller layer: mediator, stream config, controller, dummy plugins | message_stream_config, controller (11) |
| 3 Oct | Orchestration plugin, remaining configs, solution cache, feedback | orchestration incl. end-to-end update (12) |
| 3 Oct | Design and testing guides; sanitizer run | ASan/UBSan/LSan clean |
| 3 Oct | Inertial mechanization, BufferedImu, alignment in Eigen | inertial, initialization (14); inertial parity vs navtk |
| 3 Oct | Six preprocessors | preprocessors (15) |
| 3 Oct | LCM log transport, conversions, pos_ins, compare_to_truth | lcm_transport (16); position within 0.4 % of Python |
| 3 Oct | Test matrix generator | docs/TEST_MATRIX.md |
| 3 Oct | Seven more apps, yaw investigation, two-mode acceptance runner | 170 tests; legacy 7 of 8 apps pass |
| 3 Oct | Legacy Pinson-Q default; tutorial, diagnostics (HDF5), extras plugins; four apps; effective-time hook | 19 suites, 186 tests; legacy 12 of 12 with Python's epoch counts |
| 4 Oct | Roadmap and these notes; Phase 1 started | |

## 5. Python bugs and quirks found

Two are real bugs with measurable effect (1 and 14). Numbers match COBRA_ANALYSIS.md §12.

| # | Finding | Impact | In the port |
|---|---|---|---|
| 1 | Pinson15 process-noise matrix rotated in place every propagation | Yaw process noise inflated; yaw std 0.805 vs 0.845 deg; the Python tilt limits were tuned on it | Both behaviours selectable; apps default to the Python one; reported upstream |
| 2 | EKF update uses (I-KH)P and an explicit inverse | Numerical robustness | Joseph form and LDLT by default |
| 3 | Mediator is a de-facto singleton via class attributes | Tests must reset class state | MediatorContext |
| 4 | Stream config enable flag ignored (Cobra TODO 66) | None in practice | Same, documented |
| 5 | has_virtual_state_block true for real labels too | None | Same, documented |
| 6 | Size-mismatch log prints the wrong variable | Cosmetic | Fixed |
| 7 | Slice setters log an error but still write | Latent corruption | Rejected |
| 8 | request_solutions: one time; out-of-range silently replaced | Surprising | Same, DEBUG log |
| 9 | Key-value store oddities; permanency uses pickle | Not portable | Text format, no Message persistence |
| 10 | Aligner status polled twice per message | Performance | Once |
| 11 | peek_ahead deep-copies the whole engine every second | Performance | clone(); clones excluded from traces and diagnostics |
| 12 | Body-velocity processor masks missing axes with object arrays | Needs a representation | NaN marks an absent axis |
| 13 | navtk geoid lookup crashes here (SIGFPE) | MSL altitude untestable | MSL rejected until a geoid is wired in |
| 14 | Fusion engine slices H by the real block width for virtual-block targets | Works only for size-preserving VSBs | Virtual width mapped back through the Jacobian; tested |
| 15 | Preprocessors mutate messages in place; the mediator reads the mutated IMU timestamp | Shifts the 1 Hz solution grid; visible in outage_sim | Effective-time hook |

Environment findings: the pinned aspn-generated commit changed the image-feature descriptor type after the
log was recorded (vendored LCM header patched back to int16); the gh CLI cannot use the classic token git
accepts (pushes use plain git with the credential store).

## 6. The yaw investigation

After the first full pos_ins run the C++ matched Python's position to 0.4 % and velocity to 2 %, but yaw RMS
was 0.847 deg against 0.810, a 4.6 % gap that failed the Python tilt limit.

1. Rule out the EKF arithmetic: `--no-joseph` (Python's arithmetic) gave identical results.
2. Prove the components identical: `parity_check.py` (nav helpers, Pinson dynamics, processors, VSB, error
   application) and `inertial_parity_check.py` (alignment + BufferedImu over 25 s, incl. resets) agree with
   Python/navtk at machine precision. The 20 ms solution-grid offset was explained (quirk 15) and shown not
   to affect the filter.
3. Trace step by step: `PNTOS_TRACE_FILE` and `trace_python_pos_ins.py` give one line per propagate/update
   from each implementation; on a 60 s cut the sequences and inputs were identical and the first divergence
   was inside the second Pinson propagation.
4. Read that one function: Python's `generate_q_pinson15` rotates its stored sensor-frame matrix in place,
   so every call re-rotates it. With anisotropic gyro random-walk sigmas (9.9e-4, 9.9e-4, 6.7e-5) the small
   yaw-axis noise is progressively mixed with the large roll/pitch noise. `legacy_q_rotation` reproduces it
   and the C++ matched Python to three digits (yaw std 0.805 deg, 68.1 % inside 1σ).

Is legacy wrong? Relative to the author's intent, yes; but neither mode is ground truth. The corrected
filter is slightly overconfident in heading on this dataset (65 % inside 1σ instead of 68 %), so the
configured yaw gyro noise is optimistic and the bug compensated. The apps default to legacy for parity; the
roadmap's Phase 1 retunes the sigma so the corrected mode can take over.

A second gap closed the same way: outage_sim gave a north position std of 307.2 m against Python's 305.5 m.
An epoch-by-epoch comparison showed the errors identical to 1 cm at every shared time; only the solution
grid differed in phase (up to 0.4 s) because the Python mediator reads an IMU timestamp after the time
adjuster has replaced it. The effective-time hook (`utils/effective_time.hpp`) reports the post-preprocessing
time; every app then produced Python's exact epoch count and outage_sim matched.

## 7. Acceptance results (legacy mode, 3 Oct 2026)

| App | Epochs | Position std, m | Velocity std, m/s | Tilt std, deg | Legacy | Corrected |
|---|---|---|---|---|---|---|
| pos_ins | 2570 | 0.92 / 0.71 / 1.39 | 0.084 / 0.093 / 0.043 | 0.074 / 0.090 / 0.805 | pass | fail, tilt |
| pos_ins_leverarm | 2570 | 1.07 / 0.97 / 1.38 | 0.077 / 0.086 / 0.040 | 0.083 / 0.095 / 0.735 | pass | pass |
| pos_ins_bodyvel | 2570 | 0.91 / 0.89 / 1.28 | 0.097 / 0.109 / 0.046 | 0.094 / 0.098 / 1.70 | pass | fail, tilt |
| pos_vel_ins | 2570 | 1.02 / 0.82 / 1.38 | 0.127 / 0.137 / 0.042 | 0.195 / 0.185 / 1.57 | pass | fail, vel and tilt |
| posvel_ins | 2570 | 1.02 / 0.82 / 1.38 | 0.127 / 0.137 / 0.042 | 0.195 / 0.185 / 1.57 | pass | fail, vel and tilt |
| outage_sim | 2570 | 305.5 / 118.0 / 30.8 | 4.05 / 0.56 / 0.13 | 0.083 / 0.183 / 1.31 | pass | pass |
| pos_ins_vsb | 2570 | 0.92 / 0.71 / 1.39 | 0.083 / 0.093 / 0.043 | 0.074 / 0.090 / 0.809 | pass | fail, tilt |
| direction_to_points | 2570 | 15.0 / 9.0 / 4.4 | 0.53 / 0.46 / 0.09 | 0.179 / 0.175 / 0.373 | pass | fail, tilt |
| pos_ins_record_states | 2570 | 0.92 / 0.71 / 1.38 | 0.091 / 0.100 / 0.043 | 0.076 / 0.090 / 0.783 | pass, HDF5 read back | fail, tilt |
| pos_ins_zerovel2d | 2570 | 0.92 / 0.71 / 1.39 | 0.083 / 0.092 / 0.043 | 0.074 / 0.089 / 0.762 | pass | fail, tilt |
| tutorial_pos_ins | 2593 | 0.81 / 0.69 / 1.38 | 0.083 / 0.094 / 0.043 | 0.075 / 0.086 / 0.818 | pass | pass |
| tutorial_pos_vel_ins | 2593 | 1.19 / 1.56 / 0.85 | 0.131 / 0.136 / 0.040 | 0.249 / 0.233 / 1.66 | pass | fail, tilt |

The Python outage_sim run on the same machine gives 305.5 / 118.0 / 30.8 m and 2570 epochs.

## 8. Tooling

| Tool | Where | What it does |
|---|---|---|
| Acceptance runner | `tools/run_acceptance.py` (Cobra venv) | Every app in both modes against the Python checks; h5py check for record_states; writes `docs/acceptance.json` |
| Matrix generator | `tools/test_matrix.py` | Renders `docs/TEST_MATRIX.md` from meson JUnit output and acceptance.json |
| Truth comparison | `tools/compare_to_truth.py` | NED position / velocity / RPY RMS vs the truth channel |
| Component parity | `build/tools/parity_dump` + `tools/parity_check.py` | Same inputs through the numerics in C++ and Python |
| Inertial parity | `build/tools/inertial_parity_dump` + `tools/inertial_parity_check.py` | 25 s through alignment and BufferedImu in both languages |
| Step trace | `PNTOS_TRACE_FILE`, `PNTOS_TRACE_FULL`; `tools/trace_python_pos_ins.py` | One line per propagate/update from each implementation |
| HDF5 writer | `include/pntos/cobra/utils/hdf5.hpp` | Dependency-free writer for the diagnostic log |
| UI summary | `UiLogPlottingPlugin` | RMS errors logged and a per-epoch CSV instead of matplotlib |
| Effective time | `include/pntos/cobra/utils/effective_time.hpp` | Mediator sees post-preprocessing timestamps |

## 9. Repository map and how to continue

| Path | Contents |
|---|---|
| `include/pntos/api` | The pntOS API as abstract classes |
| `include/pntos/cobra`, `src/cobra` | Plugin implementations: config, controller, fusion, state_modeling, orchestration, inertial, initialization, preprocessing, transport, tutorial, diagnostics, extras, dummy, utils |
| `apps` | dummy/minimal, standard (9), tutorial (2), extras (1) |
| `tests` | `test_<suite>.cpp` + `test_support.hpp` |
| `tools` | Acceptance runner, matrix generator, parity and trace tools |
| `third_party` | Vendored header-only code with NOTICE.md |
| `docs` | DESIGN.md, TESTING.md, TEST_MATRIX.md, PROGRESS.md, COBRA_ANALYSIS.md, ROADMAP.md, SESSION_NOTES.md |
| `Cobra` | The Python original as a submodule; its venv holds meson, ninja, lcm, aspn23_lcm, h5py and the example log |

Working loop:

1. `.venv/bin/meson compile -C build && .venv/bin/meson test -C build`; all suites must stay green.
2. Port a Python class and its pytest file side by side (TESTING.md §6), keep names identical, note any
   departure in DESIGN.md §8.
3. After anything touching filter math or message flow: `Cobra/.venv/bin/python tools/run_acceptance.py`
   and `python3 tools/test_matrix.py`. All 12 apps must pass in legacy mode.
4. Dated entry in PROGRESS.md, commit, `git push`.

When a result moves without an obvious cause: parity harnesses first, then the step trace, then an
epoch-by-epoch comparison of solution logs.

## 10. Roadmap execution, 4 October 2026

After these notes were first written, the three roadmap phases were executed in the same session, each gated by
the full unit suite and the acceptance run before its push (details in PROGRESS.md and ROADMAP.md).

| Phase | Delivered | Verified by |
|---|---|---|
| 1 Foundation | JSON config files (Python class and field names), `cobra_run`, 12 example configs written by the apps, IMU / GNSS presets, corrected Pinson-Q default with yaw gyro RW retuned to 6.0e-4 rad/√s, derived corrected-mode limits, CI script + 60 s log | registry byte-identical between every compiled app and its config file in both modes; acceptance 12 / 12 in both modes, also through the config files; 20 suites |
| 2 Robustness | chi-square innovation gating with registry counters, sensor-degradation preprocessor, 11-row degraded matrix with recorded limits, EGM96 geoid for MSL altitudes, small acceptance logs | 50 m outlier rejected with the solution unchanged; consumer-grade IMU emulation raises yaw std 0.83° → 4.1°; barometer altitudes accepted; 195 tests |
| 3 Integration | `cobra::Filter` push API on a start/stop controller, log runner rebuilt on it, C ABI + C example, LCM over UDP multicast (no liblcm) + log player, CSV transport + exporter, `meson install` + pkg-config, GETTING_STARTED.md | push replay identical to the log transport in both modes; network replay of the full log reproduces pos_ins exactly (2570 epochs); CSV run equals the LCM run; 22 suites, 202 tests |

CI: the GitHub token was given the `workflow` scope on 4 Oct and `.github/workflows/ci.yml` is live (first run:
see §11). Still deferred: the ROS 2 adapter (no ROS environment here) and the UI server plugin.

## 11. Open items and limitations

- Corrected Pinson-Q mode is the default with derived limits (`docs/limits_corrected.json`); the legacy switch
  reproduces Python; the upstream answer to the reported bug decides where Python parity moves.
- Not ported: ROS transport (thin adapter over the push API when a ROS environment exists), UI server plugin and
  registry views, Buscat controller.
- Datasheet IMU presets are untuned starting points; validate them on new datasets.
- HDF5 writer: booleans as uint8 without the 'bool' attribute; Message values skipped.
- UI log plugin writes a summary and CSV rather than figures.
- Registry permanency does not persist Message values.
- One dataset only; the Phase 2 degraded-sensor matrix is synthetic until more logs exist.
