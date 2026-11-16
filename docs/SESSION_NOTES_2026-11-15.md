# Session notes, 15 November 2026: inertial review for live sensors, v0.2.9

Record of one Claude Code session on `/home/rreper/orin/work/pntos`. Everything described here is released:
tag `v0.2.9` = `9f5ffeb`, CI and release workflows green. Nothing is in progress. These notes were committed
after the tag and are not part of it.

## 1. What was asked

The brief, verbatim:

> Review src/cobra/inertial (Mechanization, BufferedImu, StandardInertialPlugin) for live-sensor robustness:
> IMU gaps/jitter vs timestamp-derived dt, SAMPLED IMU throwing, NaN/inf input, unwindowed estimated_dt, the
> two Earth-rate constants, RPY-based interpolation, per-sample allocations. Write a failing test for each real
> defect first. Keep the default mode bit-identical to navtk (inertial_parity_check.py) and 12/12 acceptance;
> make behaviour changes opt-in via InertialConfig. Run the full unit suite and acceptance before any commit.

Later in the session: "merge it, tag v0.2.9 and push". The machine rebooted once mid-session; the work was
picked up from the conversation record and the scratch baselines were rebuilt.

## 2. State at the end

| What | State |
|---|---|
| Release | v0.2.9, commit `9f5ffeb` on `main`, pushed; https://github.com/rreper/Cobra/releases/tag/v0.2.9 (tarball + sha256) |
| GitHub workflows | `ci` (build-test, sanitizers) and `release` (package) green on `9f5ffeb` |
| Unit tests | 23 suites / 238 tests, 0 failed; also under ASan + UBSan + LSan, 0 reports |
| Default mode | parity dump byte-identical to v0.2.8; `inertial_parity_check.py` 12 ok; acceptance 24/24; all 61,772 output messages identical |
| Local branch | `inertial-live-robustness` deleted after the fast-forward merge |
| Consumers | not bumped: `pnt-edge` pins v0.2.8, `cobra-integrity` pins v0.2.7 |

The commit reviewed before the merge was `404dc73`. `9f5ffeb` is that commit amended with the `meson.build`
version bump to 0.2.9 and the `docs/PROGRESS.md` entry (20 lines in 2 files), because the project bumps its
version with every tag. The gates were re-run on that tree before committing.

## 3. Findings

Measured on the port; NavToolkit behaves the same except for SAMPLED data.

| Item | Verdict | Measured | Opt-in key in `inertial_config` |
|---|---|---|---|
| Lost IMU samples | Defect | 0.098 m/s of free fall per lost 10 ms sample; 9.8 m/s and 4.9 m after one lost second | `imu_gap_policy: "hold"` |
| Time-stamp jitter | Benign | 0.5 ms rms gives a bounded 0.023 m/s, no drift. An out-of-order stamp drops the sample, same cost as a lost one | covered by `hold` |
| One future-stamped sample | Defect | solution jumps (35 km/s for a one-hour stamp), every later sample rejected for good | `max_imu_gap` |
| SAMPLED IMU | Defect | throws `std::invalid_argument` out of `Filter::push`. The Python apps run navtk with `ErrorMode.OFF` and drop silently, so the port is stricter | `sampled_imu: "drop"` or `"integrate"` |
| NaN / inf | Defect | every later solution is NaN | `reject_non_finite` |
| Unwindowed `estimated_dt()` | Defect | 13.3 ms for a 10 ms IMU two seconds after a one-second gap, so averaged forces and rates are 25 % low | `dt_window` |
| RPY interpolation | Defect, narrow | attitude error within about 1e-6 rad of pitch ±90°, 90° at the singularity | `attitude_interpolation: "quaternion"` |
| Two Earth-rate constants | Not a defect | 1.5e-12 rad/s apart, 5e-9 rad per hour; left alone to keep parity | none |
| Per-sample allocations | Not a defect | 2 allocations and 2 µs per sample; about 414 allocations / 250 µs for a 1 s averaged force query | none |

The recorded VN-100 log has jitter up to 1.6 intervals: 3 samples above 1.5× in 43 minutes, none above 1.75×.
That is why the gap threshold defaults to 1.75.

## 4. What was built

All in `inertial::ImuRobustness` (`include/pntos/cobra/inertial/ImuRobustness.hpp`), carried in
`InertialConfig::robustness`. Defaults reproduce NavToolkit; the keys are written to the registry and to JSON
only when they differ from the default.

| Key | Default | `live_sensor()` | Meaning |
|---|---|---|---|
| `sampled_imu` | `"throw"` | `"integrate"` | `"drop"` rejects, `"integrate"` uses rate × stamped dt |
| `reject_non_finite` | `false` | `true` | refuses non-finite samples, resets and bias corrections |
| `imu_gap_policy` | `"navtk"` | `"hold"` | hold the late sample's rate across the lost ones |
| `imu_gap_factor` | 1.75 | 1.75 | a sample is late when dt exceeds this times the nominal interval; must be > 1 |
| `max_imu_gap` | 0 (unlimited) | 1.0 s | a stamp further ahead is refused until three in a row agree |
| `dt_window` | 0 (mean since start) | 100 | median of the last N intervals; 0 or ≥ 3 |
| `attitude_interpolation` | `"rpy"` | `"quaternion"` | slerp on the quaternions |

How `hold` works: once more than 10 intervals have been seen, a sample with dt above `imu_gap_factor` × nominal
is mechanized `max(2, round(dt / nominal))` times in sub-steps of one nominal interval. Counting whole samples
makes it neutral to jitter. A first attempt bridged the gap in one step and left 2.8e-4 m/s for a one-second
gap, because the ½ dθ × dv term grows with the step squared. Replays in `reset()` and
`calc_pva_no_reset_since()` use the same `plan()` / `run()` helpers, so a replay bridges a gap as the first pass
did. Sub-steps are capped at 1000 and never shorter than 1 µs.

Reporting: `BufferedImu::add_sample()` returns an `AddStatus` (`ACCEPTED`, `GAP_FILLED`, `NOT_NEWER`,
`SAMPLED_DROPPED`, `NON_FINITE`, `GAP_TOO_LONG`), `stats()` counts each kind and the longest gap, and
`StandardInertialPlugin` logs the 1st, 10th, 100th, ... occurrence of each kind at WARN. `add()` is unchanged.

Two changes are not opt-in, because both were undefined behaviour: a zero, negative or NaN `expected_dt` /
`inertial_buffer_length` is refused with an ERROR (no inertial is created), and interpolating next to a
solution without a quaternion no longer reads an empty optional.

Files: `ImuRobustness.hpp` (new), `BufferedImu.hpp/.cpp`, `StandardInertialPlugin.hpp/.cpp`,
`config/configs.hpp/.cpp`, `config/JsonConfig.cpp`, `tests/test_inertial_live.cpp` (new, 28 tests),
`tests/test_json_config.cpp` (`JsonConfig.InertialRobustnessKeys`), `tests/test_filter.cpp`
(`Filter.LiveImuFaults`), `tools/compare_logs.py` (new), `tools/test_matrix.py`. Docs: DESIGN.md §7.22 and
deviations 19–20, TESTING.md, GETTING_STARTED.md, README.md, TEST_MATRIX.md (regenerated), PROGRESS.md.

## 5. Evidence

- **Failing first.** Before any implementation: 19 of the 26 tests in `inertial_live` failed (two by
  crashing), 7 passed (the `Default...` tests that pin today's behaviour, and the jitter guard); the JSON test
  and the end-to-end test failed. `HeldRateUnderChangingDynamics` and `OutOfOrderStampCostsNothingWhenHeld`
  were added after the implementation, so those two were not failing-first.
- **End to end** (`Filter.LiveImuFaults`, 60 s of the example log, 4 % of the IMU messages lost, TimeAdjuster
  removed): default 5.33 m vertical and 0.34 m horizontal from the clean run; live settings 0.06 m and 0.15 m.
- **Under dynamics** (1 Hz motion, 47 lost samples): held 2.7 mm/s, 10 mm, 2.9e-4 rad; default 4.6 m/s, 11 m.
- **Live settings on clean data:** acceptance 24/24, largest statistic difference 4.68e-4 (outage_sim).
- **Raw stamps** (TimeAdjuster removed, 10 standard apps): default and live pass and fail the same rows, 10 of
  20, against Python limits that were tuned on the adjusted stream. No WARN in live mode over the full log.

## 6. Found outside the inertial, not fixed

1. **Mediator publish timer** (`StandardMediator.cpp:173-183`). A future-stamped message is rejected by the
   inertial but still latches the publish timer; periodic solutions stop until message time catches up.
   `Filter::solution()` keeps answering.
2. **TimeAdjuster** (in every standard config, 100 µs tolerance, 10 ms expected). Each lost IMU sample becomes a
   permanent 10 ms time slip, so the inertial never sees a gap and `hold` does nothing until the adjuster is
   removed from the filter config. An IMU at another rate is re-stamped onto a wrong time base.
3. **pnt-edge, before the bench run** (read from source and simulated on the Cobra side; pnt-edge not run):
   - `configs/pnt-edge-serial.json` streams the VN-100 at 200 Hz into Cobra's `pos_ins.json`, whose adjuster
     expects 10 ms. In simulation IMU time runs at twice real time: 19.55 m/s downward after 2 s.
   - A VN-100 on rate output is pushed as SAMPLED; `SerialInput::pusher()` has no catch around
     `filter_.push`, so the daemon would terminate.
4. **`inertial_parity_check.py`**: the tolerance is 1e-9 × max|value| per quantity, time stamps included
   (1.7e9 s), so it accepts differences up to 1.7. Real differences are 1e-12 and below. It also hides that the
   averaged force/rate message is stamped 0.49 s differently from navtk; values agree to 4e-15 and nothing
   reads the stamp.
5. Smaller: the ring capacity derives from `expected_dt`, so a rate mismatch shrinks the history;
   `calc_pva_no_reset_since` returns the solution at `since` when no IMU record lies between `since` and `t`
   (inherited, unused by the standard orchestration); there is no plausibility bound on huge but finite IMU
   values; `inertial_parity_dump <log> 60` aborts (only the 25 s dump works).
6. Not addressed: pnt-edge's open item "a position fix ahead of the inertial leaves the filter in error".

One correction made during the session: I first wrote that pnt-edge does not run the TimeAdjuster. It does,
through Cobra's `pos_ins.json`. Docs, the test comment and the dashboard carry the corrected statement.

## 7. Open decisions

1. Bump the `pntos-cpp` wraps to v0.2.9 in `pnt-edge` and `cobra-integrity`. No behaviour change by itself.
2. Should `sampled_imu` default to `"drop"`? It matches what the Python apps do, but changes the port's
   default of throwing.
3. Tighten `inertial_parity_check.py`, and match or whitelist the averaged-force time stamp?
4. Fix the mediator publish-timer latch? Change or document the TimeAdjuster behaviour?
5. pnt-edge: a filter config without `TimeAdjusterConfig` and with the live keys; the 200 Hz mismatch; a
   catch around `filter_.push`.

## 8. How to pick this up again

Build and unit tests (the system meson 1.5.1 refuses the build directory; use the venv's 1.12.1):

```bash
cd /home/rreper/orin/work/pntos
export PATH=$PWD/Cobra/.venv/bin:$PATH
meson compile -C build && meson test -C build
```

Gates:

```bash
Cobra/.venv/bin/python tools/inertial_parity_check.py
Cobra/.venv/bin/python tools/run_acceptance.py --build build --out /tmp/acc.json --workdir /tmp/acc
```

Proving the default mode unchanged against a release needs a pristine build of that release. The scratch
copies under `/tmp` do not survive a reboot; this rebuilds them in about 1.5 minutes plus the acceptance run:

```bash
R=/home/rreper/orin/work/pntos; S=<scratch dir>; REF=v0.2.9
mkdir -p $S/ref-src $S/baseline
git -C $R archive $REF | tar -x -C $S/ref-src
for d in aspn-generated eigen-5.0.1 googletest-1.18.0 packagecache; do
  ln -s $R/subprojects/$d $S/ref-src/subprojects/$d; done
meson setup $S/ref-build $S/ref-src -Dtests=false --wrap-mode=nodownload
meson compile -C $S/ref-build
LOG=$R/Cobra/.venv/lib/python3.12/site-packages/pntos_python_datasets_lcm/cobra_gps_ins_example_data.log
$S/ref-build/tools/inertial_parity_dump $LOG 25 > $S/baseline/parity_dump_25.json
cd $R && Cobra/.venv/bin/python tools/run_acceptance.py --build $S/ref-build \
  --out $S/baseline/acceptance.json --workdir $S/baseline/acc
# then, for the build under test:
build/tools/inertial_parity_dump $LOG 25 | cmp - $S/baseline/parity_dump_25.json
Cobra/.venv/bin/python tools/run_acceptance.py --build build --out $S/new.json --workdir $S/new
Cobra/.venv/bin/python tools/compare_logs.py $S/baseline/acc $S/new
```

Things that cost time in this session:

- The parity dump of v0.2.8 and v0.2.9 has sha256 `add099b1...`.
- Raw `.log` hashes always differ: LCM stores the wall-clock time of writing with every message.
  `tools/compare_logs.py` compares channel and payload only.
- Do not rebuild while an acceptance run is in the background; it executes `build/apps/*` and fails with
  "Permission denied" mid-link.
- `gh` is not logged in on this machine, although `git push` works. The repository is public, so
  `https://api.github.com/repos/rreper/Cobra/actions/runs` answers without a token. A push to `main` starts
  `ci` (about 8 minutes); a `v*` tag starts `release`.
- Two warnings in the build predate this work: `Mechanization.cpp:92` (`-Wmaybe-uninitialized`, Eigen) and
  `tests/test_json_config.cpp:358` (`-Wdangling-else`).

## 9. Where the rest is

- Design and limits: `docs/DESIGN.md` §7.22. Progress entry: `docs/PROGRESS.md`, 2026-11-15.
- Dashboard: `~/projects/_dashboard/projects/pntos.{json,md}` and `pnt-edge.{json,md}`, both updated.
- Full transcript:
  `/home/rreper/.claude/projects/-home-rreper-orin-work/390f72b7-9ad8-4396-8ce9-ea4300851024.jsonl`
