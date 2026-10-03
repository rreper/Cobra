# pntos-cpp — C++ rewrite of the pntOS "Cobra" reference implementation

Repository: https://github.com/rreper/Cobra (C++ rewrite; the Python original is the `Cobra/` submodule).

A C++20 implementation of the [pntOS](https://github.com/Open-PNT/pntOS-C) plugin architecture for
PNT (position, navigation, timing) sensor fusion, ported from IS4S's pure-Python reference
implementation [Cobra](https://github.com/is4s/Cobra). Linear algebra is [Eigen](https://eigen.tuxfamily.org);
ASPN-23 messages come from [aspn-generated](https://github.com/is4s/aspn-generated) (`aspn23_eigen`);
inertial mechanization and alignment come from [NavToolkit](https://github.com/is4s/NavToolkit).

| | |
|---|---|
| Status | **Tier 1 complete**: `pos_ins` runs end to end on the example log and reproduces the Python position/velocity accuracy at ~20× the speed (see [docs/PROGRESS.md](docs/PROGRESS.md), [docs/DESIGN.md](docs/DESIGN.md) §9.5) |
| Design guide (read first) | [docs/DESIGN.md](docs/DESIGN.md) — build, layout, type mapping, every component, deviations, roadmap, recipes |
| Testing guide | [docs/TESTING.md](docs/TESTING.md) — running, suite inventory, mocks, goldens, porting a Python test, acceptance |
| Test pass/fail matrix | [docs/TEST_MATRIX.md](docs/TEST_MATRIX.md) — every test with its result, Python-test coverage, app acceptance; regenerate with `tools/test_matrix.py` |
| Analysis of the Python original | [docs/COBRA_ANALYSIS.md](docs/COBRA_ANALYSIS.md) — architecture, baselines, findings, app matrix |
| Reference implementation | `Cobra/` (git submodule, upstream, Apache-2.0) |
| License | Apache-2.0 (same as upstream) |

## Layout

```
include/pntos/api/      the pntOS plugin API in C++ (mirrors pntOS-C headers / Cobra's pntos.api)
include/pntos/cobra/    public headers of the Cobra plugin implementations
src/                    implementation
tests/                  GoogleTest unit tests (ported from Cobra's pytest suite) + golden vectors
apps/                   runnable apps: dummy/minimal, standard/* (9), tutorial/* (2), extras/pos_ins_zerovel2d
docs/                   analysis, progress log, diagrams
subprojects/            meson wraps: eigen, gtest, aspn-generated, pntos-c, (navtoolkit)
Cobra/                  the Python original, as a submodule, used for golden-vector generation
```

## Building

```bash
# one-time: meson + ninja (any recent meson works; the Cobra venv has one)
python3 -m venv .venv && .venv/bin/pip install meson ninja
.venv/bin/meson setup build
.venv/bin/meson compile -C build
.venv/bin/meson test -C build
```

All dependencies are fetched as meson subprojects on first `setup`; nothing needs to be installed system-wide.

## Working on the port

1. Read `docs/DESIGN.md` §9 for the next item and the Python source it comes from.
2. Port the class and its Python test side by side (`docs/TESTING.md` §6), keeping names identical.
3. `meson test -C build` must stay green; add a dated entry to `docs/PROGRESS.md`; commit and push.

## Running the apps

```bash
.venv/bin/meson compile -C build
./build/apps/pos_ins out.log                      # input defaults to the Cobra example dataset in Cobra/.venv
./build/apps/pos_ins out.log in.log --corrected-q # Pinson Q rotated from a copy (default: --legacy-q, Python-compatible)
./build/apps/pos_ins_record_states out.log        # also writes out.hdf5 (filter states after every propagate/update)
./build/apps/tutorial_pos_ins out.log             # tutorial stack; logs RMS errors vs truth and writes out/pva_errors.csv
Cobra/.venv/bin/python tools/compare_to_truth.py out.log   # NED position / velocity / RPY RMS vs truth
Cobra/.venv/bin/python tools/run_acceptance.py             # all 12 apps, both modes, Python integration limits
```

Apps: `pos_ins`, `pos_vel_ins`, `posvel_ins`, `pos_ins_leverarm`, `pos_ins_bodyvel`, `outage_sim`, `pos_ins_vsb`,
`direction_to_points`, `pos_ins_record_states`, `tutorial_pos_ins`, `tutorial_pos_vel_ins`, `pos_ins_zerovel2d`.
In the default (Python-compatible) mode all 12 pass the Python integration-test limits with Python's epoch counts
(`docs/TEST_MATRIX.md` §4).

## Acceptance

The port is measured against the Python original: identical unit-test expectations (ported),
golden vectors generated from the Python code, and the 13-app integration matrix on the 43-minute
example log with the same truth-error bounds (see `docs/COBRA_ANALYSIS.md` §2 and §14).
