# Progress log

Newest first. Each entry: what changed, what was verified, what is next.

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
