# Progress log

Newest first. Each entry: what changed, what was verified, what is next.

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
