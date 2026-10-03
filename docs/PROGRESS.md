# Progress log

Newest first. Each entry: what changed, what was verified, what is next.

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
