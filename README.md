# pntos-cpp — C++ rewrite of the pntOS "Cobra" reference implementation

A C++20 implementation of the [pntOS](https://github.com/Open-PNT/pntOS-C) plugin architecture for
PNT (position, navigation, timing) sensor fusion, ported from IS4S's pure-Python reference
implementation [Cobra](https://github.com/is4s/Cobra). Linear algebra is [Eigen](https://eigen.tuxfamily.org);
ASPN-23 messages come from [aspn-generated](https://github.com/is4s/aspn-generated) (`aspn23_eigen`);
inertial mechanization and alignment come from [NavToolkit](https://github.com/is4s/NavToolkit).

| | |
|---|---|
| Status | **early — Tier 1 in progress** (see [docs/PROGRESS.md](docs/PROGRESS.md)) |
| Analysis of the Python original | [docs/COBRA_ANALYSIS.md](docs/COBRA_ANALYSIS.md) |
| Reference implementation | `Cobra/` (git submodule, upstream, Apache-2.0) |
| License | Apache-2.0 (same as upstream) |

## Layout

```
include/pntos/api/      the pntOS plugin API in C++ (mirrors pntOS-C headers / Cobra's pntos.api)
include/pntos/cobra/    public headers of the Cobra plugin implementations
src/                    implementation
tests/                  GoogleTest unit tests (ported from Cobra's pytest suite) + golden vectors
apps/                   runnable apps (dummy/minimal, standard/pos_ins, ...)
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

## Acceptance

The port is measured against the Python original: identical unit-test expectations (ported),
golden vectors generated from the Python code, and the 13-app integration matrix on the 43-minute
example log with the same truth-error bounds (see `docs/COBRA_ANALYSIS.md` §2 and §14).
