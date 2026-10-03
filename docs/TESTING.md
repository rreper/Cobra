# pntos-cpp testing guide

How the test suite is organised, how to run it, how to port a Python test, and how the port will be
accepted against the Python original. Companion to `docs/DESIGN.md`.

1. [Running the tests](#1-running-the-tests)
2. [Test support helpers](#2-test-support-helpers)
3. [Suite inventory](#3-suite-inventory)
4. [Mocks and fakes](#4-mocks-and-fakes)
5. [Golden values and tolerances](#5-golden-values-and-tolerances)
6. [Porting a Python test](#6-porting-a-python-test)
7. [Reproducing the Python baselines](#7-reproducing-the-python-baselines)
8. [Acceptance plan for the applications](#8-acceptance-plan-for-the-applications)
9. [Python tests not yet ported](#9-python-tests-not-yet-ported)
10. [Debugging](#10-debugging)

---

## 1. Running the tests

```bash
.venv/bin/meson compile -C build          # build everything
.venv/bin/meson test -C build             # run all suites (one process per suite)
.venv/bin/meson test -C build state_modeling -v        # one suite, verbose gtest output
./build/tests/test_fusion_engine --gtest_filter='*Peek*'  # one test, gtest filter syntax
./build/tests/test_orchestration --gtest_repeat=20 --gtest_shuffle   # flakiness check
```

Suites are registered in `tests/meson.build` in the `test_sources` dictionary; each entry becomes an
executable `build/tests/test_<name>` linked against `cobra_dep`, `gtest_main` and `gmock`. The meson test
protocol is `gtest`, so individual test names show up in `build/meson-logs/testlog.txt` and in
`testlog.junit.xml`.

Current status: **16 suites, 169 tests, all green** (2026-10-03). A test run takes well under a second.

## 2. Test support helpers

`tests/test_support.hpp` (namespace `pntos::test`) is included by every suite:

| Helper | Purpose |
|---|---|
| `TestMediator` | `api::Mediator` that records `logs` (`LogEntry{level, message}`), `processed` and `broadcast` messages; `count(level)`, `has_error()`, `last_message()`; optional registry via `set_registry()` (throws if used without one) |
| `header(type)` | an `aspn23_eigen::TypeHeader` with zero ids |
| `dyn({...})` / `DynVector` | dynamic Eigen column vector literal (the aspn classes take `Eigen::Matrix<double,-1,1>`) |
| `rm(matrix)` / `RowMajorMatrix` | row-major copy (the aspn classes store covariances row-major) |
| `make_pva(tov, lat, lon, alt, vn, ve, vd, quat, cov = 0₉ₓ₉)` | geodetic `MeasurementPositionVelocityAttitude`; pass a 4-vector of `NaN` for "no quaternion" |
| `make_position(tov, lat, lon, alt, cov, frame = GEODETIC)` | `MeasurementPosition` |
| `make_imu(tov, accel, gyro)` | `MeasurementImu` (sampled type) |
| `allclose(a, b, rtol = 1e-5, atol = 1e-8)` | numpy semantics, shape-checked |
| `vec({...})`, `mat({{...}})` | `api::Vector` / `api::Matrix` literals |
| `EXPECT_ALLCLOSE(a, b)` | prints both operands on failure |

When a helper is missing, add it here rather than in a suite; keep it free of behaviour (no filtering,
no config).

## 3. Suite inventory

| Suite (C++) | Tests | Ported from (Python, `Cobra/pntos-cobra/tests/`) | What it pins down |
|---|---|---|---|
| `smoke` | 2 | — | aspn-generated + Eigen link and basic message construction |
| `api` | 3 | — | `Timestamp`, `Message::as<T>`, `registry_value_as` conversion table |
| `ekf_fusion_strategy` | 31 | `test_standard_ekf_fusion_strategy.py` (all) | add/remove states, cross-covariance placement, slices, propagate, update (Python arithmetic via `set_joseph_form(false)`), error paths |
| `simple_state_blocks` | 17 | `test_fogm_block.py`, `test_constant_block.py`, `test_clock_bias_block.py` | closed-form `Φ`, `Qd`, `g`, bad inputs, clone |
| `registry` | 22 | `test_registry.py` (all but pickled `Message` permanency) | key order, types, batches, notify (key / any-key / new group), permanency round trip, `end_all_batches` |
| `state_modeling` | 17 | `test_state_modeling.py` (all) | provider indices and rejection paths; `H`/`z`/`h`/`R` of every processor; Pinson `Φ`/`Qd` golden matrices (rtol 1e-5, atol 1e-20); Q-not-mutated regression; clone independence |
| `virtual_state_blocks` | 5 | `test_virtual_state_blocks.py` (VSB half) | `StateExtractor` valid/invalid; `PinsonErrorToStandard` conversion, Jacobian vs central differences, invalid cases |
| `vsb_manager` | 4 | `test_virtual_state_blocks.py` (manager half) | 25-deep random-order chain, caching, pruning on removal, aux delivery, invalid ops, deep copy |
| `fusion_engine` | 8 | — (Python covers the engine only via orchestration) | bookkeeping with cross-covariances and re-indexing, non-generic EWC rejection, block-diagonal propagate, update through real and virtual blocks, aux routing, `peek_ahead`/`clone` isolation, diagnostics save, plugin config |
| `message_stream_config` | 9 | `test_message_stream_config.py` (all) + source-specific overrides | buffer-mode resolution rules |
| `controller` | 13 | `test_single_threaded_controller.py` (all) + mediator behaviour | mediator routing/buffering/publishing/broadcast/logging/UI gate; `ExitEvent`; controller wiring order, validation, `ready_to_shutdown`, error exit code; dummy controller end to end; config round trips |
| `inertial` | 5 | `test_inertial_plugin.py` (all) + mechanization checks | plugin contract (ranges, reset, continuous vs best solutions, forces/rates, sensor errors); stationary mechanization drift bound; error-model correction; helper closed forms; ring eviction/interpolation |
| `initialization` | 5 | `test_manual_initialization_plugin.py`, `inertial_alignment/test_static_align_initialization_plugin.py`, `inertial_alignment/test_manual_heading_align_initialization_plugin.py` | manual solution fields; static and manual-heading alignment on 120 s of synthetic data (level, yaw -pi/2, diagonal covariances); gyro-compass recovers a known attitude; PVA-message initialization with start time and sigma override |
| `preprocessors` | 9 | `test_preprocessor_plugin.py` (all) + time bias | plugin indices/config errors; downsampler counting; IMU rotation (input untouched); time adjuster synthesis within/outside tolerance; baro→altitude value/variance/channel rename/sigma override; time bias; outage window with INFO logs |
| `lcm_transport` | 7 | `test_transport_plugin.py` (log parts) + round trips | log write/read; encode/decode of all six supported types incl. absent quaternion; decoding the first 3000 events of the real example log; replay with shutdown flag and UI gate group; channel filter + output recording; threaded listen/stop; same input/output error |
| `orchestration` | 12 | `test_orchestration.py` (standard cases) | init with real fusion/EKF/state-modeling plugins and mock inertial/initializer; config round trip; one channel → many processors; VSB-chain aux; outage propagation; alignment after N messages; `request_solutions` in all forms; end-to-end position update with feedback |

Python tests that correspond to components not yet ported are listed in §9.

## 4. Mocks and fakes

All mocks live inside the test files that use them (there is deliberately no shared mock library yet;
promote one to `test_support.hpp` when a second suite needs it).

| Mock | File | Stands in for | Contract it honours |
|---|---|---|---|
| `DirectProcessor` | `test_fusion_engine.cpp` | a measurement processor | `z = x + v` on one (real or virtual) block; records aux count and the state size it saw |
| `RecordingOrchestration`, `RecordingTransport`, `RecordingLogger` | `test_controller.cpp` | orchestration / transport / logging plugins | record every call with arguments; orchestration configures the stream config (sequenced default, IMU immediate) like the real one |
| `DummyInertialPlugin`, `DummyInitializationPlugin`, `DummyUiPlugin` | `test_controller.cpp` | plugin kinds the controller only needs to wire | store the mediator; UI returns from `run_main_thread` immediately |
| `MockInertial` / `MockInertialPlugin` | `test_orchestration.cpp` | `StandardInertialMechanization` | constant PVA; latest time follows processed IMU; zero-covariance dead-reckoning solutions; constant specific force `(0,0,-9.81)`; counts resets and bias corrections; `reset_solution` adopts the given PVA |
| `MockInitializer` / `MockInitializationPlugin` | `test_orchestration.cpp` | `InertialInitializationStrategy` | `INITIALIZED_GOOD` immediately or after N alignment messages; initial solution from the manual alignment numbers; 6×6 bias covariance |
| `MockMP`, `MockVSB`, `MockProvider`, `MockStateModelingPlugin` | `test_orchestration.cpp` | a second state-modeling plugin | identifiers `mock_mp` / `mock_vsb`; record "received message" / "got aux" events (Python prints them) |
| `cobra::Dummy*` | `include/pntos/cobra/dummy/` | production dummies | ported from Python's `dummy_plugins`; used by the minimal app and by `test_controller` |

The real inertial and alignment plugins now exist; `test_orchestration.cpp` should gain a second fixture
that uses `StandardInertialPlugin` + `TutorialInitializationPlugin` with `ManualAlignmentConfig`, keeping the mock
fixture for fast, deterministic routing tests (this is the first item in §9 of DESIGN.md's roadmap after the
preprocessors).

## 5. Golden values and tolerances

- **Ported expectations** are copied verbatim from the Python tests (numbers, matrices, tolerances). Where
  Python used `np.allclose` defaults, use `EXPECT_ALLCLOSE`; where it used explicit `rtol`/`atol`, pass
  them to `allclose`.
- **Pinson `Φ`/`Qd` golden matrices** in `test_state_modeling.cpp` are the literal arrays from
  `test_state_modeling.py::test_generate_dynamics` (zero PVA at the origin, force `(0,0,-9.8)`, 1 s step),
  compared with `rtol=1e-5, atol=1e-20`. They pass with the Van Loan / matrix-exponential implementation in
  `nav::`; do not loosen them.
- **EKF parity** tests run with Joseph form off so that the arithmetic matches Python's
  `(I−KH)P` to round-off.
- **Hand-derived expectations** (fusion engine, controller, orchestration) state the derivation in a
  comment next to the assertion, e.g. the north-variance bounds in
  `EndToEndPositionUpdateWithFeedback`.
- **Regenerating goldens from Python.** Use the submodule's venv:

  ```bash
  cd Cobra/pntos-cobra
  ../.venv/bin/python -m pytest tests/test_state_modeling.py -k generate_dynamics -s
  ```

  To dump new vectors, add a temporary `print(repr(array.tolist()))` in the Python test (or a small
  script importing `pntos.cobra`) and paste the output into a `mat({...})` literal. Record the generating
  command in a comment above the literal.

## 6. Porting a Python test

Pattern mapping that was used for every suite so far:

| Python | C++ |
|---|---|
| `def test_x(fixture):` | `TEST_F(Fixture, X)` (or `TEST(Group, X)` when no fixture) |
| `conftest.py` fixtures (`mediator`, `gxp`, …) | fixture `SetUp()` members and small free functions at the top of the suite |
| `assert a == b` on arrays | `EXPECT_EQ` for exact, `EXPECT_ALLCLOSE` for numeric |
| `assert np.allclose(a, b, rtol, atol)` | `EXPECT_TRUE(allclose(a, b, rtol, atol)) << a << b` |
| `with pytest.raises(RuntimeError):` | `EXPECT_THROW(..., std::runtime_error)` |
| `assert x is None` (API failure) | `EXPECT_FALSE(opt.has_value())` or `EXPECT_EQ(ptr, nullptr)` |
| checking a log was emitted | `med.has_error()`, `med.count(LoggingLevel::WARN)`, `med.last_message()` |
| `print(...)` captured with `redirect_stdout` | record into a vector on the mock and assert on it |
| `copy.deepcopy(obj)` | `obj.clone()` |
| mutating a message in place | build a new one (`make_*`, `utils::copy_pva` + setters) |
| `isinstance(x, SomeType)` | `dynamic_cast<SomeType*>(x.get()) != nullptr` |
| a Python-only behaviour (pickle, textual `print` output, `sys.exit`) | note the deviation in a comment and in `DESIGN.md` §8 |

Procedure:

1. Read the Python test and the fixture code it uses; list every expectation.
2. Create `tests/test_<name>.cpp`, register it in `tests/meson.build`.
3. Port fixture state into a `::testing::Test` subclass; prefer real plugins over mocks wherever the real
   one exists in C++.
4. Port tests one by one, keeping the Python test name as the C++ test name in PascalCase so the mapping
   in §3 stays obvious.
5. Run the suite; when an expectation fails, decide *which side is wrong* before changing either. If the
   C++ is wrong, fix it. If the Python expectation encodes a bug, fix the C++ behaviour, keep a test for
   the correct behaviour, and add a row to `DESIGN.md` §8 and a numbered item to `COBRA_ANALYSIS.md` §12.
6. Add a line to `docs/PROGRESS.md`.

## 7. Reproducing the Python baselines

The analysis baselines (what the C++ must eventually match) were produced like this and can be re-run
at any time from the submodule:

```bash
cd Cobra
python3 -m venv .venv && .venv/bin/pip install -e pntos-cobra[dev]   # see Cobra's README for extras
cd pntos-cobra
../.venv/bin/python -m pytest tests -q            # 366 passed, 1 failed (geoid env issue, see analysis §12 #13)
```

Integration matrix (13 apps on the 43-minute example log, ~11 minutes total):

```bash
../.venv/bin/python -m pytest tests/manual -q      # or the per-app commands in COBRA_ANALYSIS.md §14
```

Reference numbers for `pos_ins` (COBRA_ANALYSIS.md §2): position RMS 0.92 / 1.24 / 1.67 m NED, velocity
RMS 0.084 / 0.093 / 0.043 m/s, tilt RMS 0.074 / 0.092 / 0.811°, 2570 epochs, 37.75 s CPU in Python.

## 8. Acceptance plan for the applications

1. **Unit parity** — every Python test file in §3 and §9 ported and green. Done for everything except the
   network LCM transport, the UI/diagnostics layer and the tutorial/Buscat plugins.
2. **Golden replay** — DONE: `build/apps/pos_ins OUT.log` replays the example log (2.1 s).
3. **Compare** — DONE with `tools/compare_to_truth.py` (see DESIGN.md §9.5 for the table): position and
   velocity RMS within about 1 % of the Python baseline. Re-run after any change to the filter math.
4. **Repeat for the other 12 apps** in the integration matrix (`COBRA_ANALYSIS.md` §14) as their plugins
   are ported.
5. **Performance** — record wall and CPU time next to Python's 37.75 s; expect an order of magnitude less.

## 9. Python tests not yet ported

| Python test | Component to port first | Planned C++ suite |
|---|---|---|
| `test_transport_plugin.py` (network LCM parts) | `LcmTransportPlugin` (needs liblcm) | `transport` |
| `test_orchestration.py` (tutorial cases, `test_process_pntos_message_before_aligned`) | tutorial plugins, static alignment | `orchestration` (second fixture) |
| `test_configutils.py` | — (no generic config utils in C++) | covered by per-config round-trip tests; add any missing cases to the owning suite |
| `test_registry_views.py`, `test_ui_utils.py`, `test_cobra_ui_plugin.py` | UI layer (Tier 2) | `ui` |
| `test_diagnostic_log_plugin.py`, `test_hdf5utils.py` | diagnostic log plugin (Tier 2) | `diagnostics` |
| `test_buscat_controller.py` | Buscat controller (Tier 3) | `buscat` |
| `test_aspn_ros.py` | ROS transport (Tier 3) | `ros` |

## 10. Debugging

- `meson test -C build <suite> --gdb` drops into gdb on failure; `--gtest_break_on_failure` works too.
- Sanitizers: `meson configure build -Db_sanitize=address,undefined` then rebuild. The suite is clean under
  ASan/UBSan as of the orchestration layer (verified 2026-10-03, `build-asan`); keep it that way.
- Verbose logging: `TestMediator` keeps every log line; dump with
  `for (auto& l : med.logs) std::cerr << int(l.level) << ' ' << l.message << '\n';`.
- Printing Eigen in gtest messages: `EXPECT_TRUE(cond) << "\n" << matrix;` (Eigen streams directly).
- A wrong `Φ`/`Qd` almost always traces back to a frame or unit slip in `navutils.hpp`; compare the
  individual helper against `navtk.navutils` in the venv before touching the block.
