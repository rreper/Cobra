# Contributing

This repository is a C++20 port of the Python Cobra reference implementation of pntOS. The Python source in
the `Cobra/` submodule is the specification; the port keeps its structure and names and documents every
departure in `docs/DESIGN.md` §8. Read `docs/DESIGN.md` §1–3 once before changing anything.

## The loop

1. Build and test: `.venv/bin/meson compile -C build && .venv/bin/meson test -C build`. Every suite must stay green.
2. Change one thing. Port a Python class together with its pytest file (`docs/TESTING.md` §6 has the idiom
   table), or add a C++-only feature behind a config field that is off by default so a Python-equivalent
   configuration still produces the same registry contents.
3. If the change touches the filter math, the message flow, a config class or a transport, run the acceptance:
   `Cobra/.venv/bin/python tools/run_acceptance.py` (all 12 apps, both Pinson-Q modes; must stay 24 / 24) and
   `python3 tools/test_matrix.py`. For sensor-model or robustness changes also run
   `Cobra/.venv/bin/python tools/run_degraded_matrix.py`.
4. Document: a dated entry at the top of `docs/PROGRESS.md`; `docs/DESIGN.md` for a new component or deviation;
   `docs/TESTING.md` for a new suite or tool; `README.md` if a user-visible command changed.
5. Commit with a message that says what and why; push. CI (`.github/workflows/ci.yml`) builds, runs the suites,
   the short acceptance on `testdata/example_60s.log` three ways and the C example. The sanitizer job must be
   clean too.

## Adding things

| To add | Where | Also |
|---|---|---|
| A measurement processor | `state_modeling/MeasurementProcessors.hpp/.cpp`, identifier in `StandardStateModelProvider` | a `*MPConfig` factory in `config/configs.hpp`, the JSON type name in `config/JsonConfig.cpp` (`mp_kinds`), a test in `tests/test_state_modeling.cpp` with `z`, `H`, `h` and a Jacobian check |
| A state block | `state_modeling/SimpleStateBlocks.hpp` or its own file, provider index | a `*StateBlockConfig`, JSON support, tests for `Phi` / `Qd` against a closed form |
| A preprocessor | `preprocessing/StandardPreprocessorPlugin.hpp` (standard) or `extras/AdvancedPreprocessorPlugin.hpp` | a `PreprocessorConfig` subclass with `kIdentifier`, `PNTOS_PP_IMPL` in `configs.cpp`, JSON support, a test in `tests/test_preprocessors.cpp` or `test_extras.cpp` |
| A transport | `transport/`, an `AppSpec.transport` name in `app/AppBuilder.cpp` | a config class with registry and JSON support, a test that feeds a few messages through a `TestMediator`, an example config in `configs/` |
| A sensor preset | `presets/Presets.cpp` | the datasheet source in the entry; `tests/test_json_config.cpp` checks every preset |
| A config field | the struct, `to_registry` / `from_registry` (optional fields written only when set), `config/JsonConfig.cpp` both directions | the JSON round-trip test compares registry contents, so a missing direction fails there |

Messages are immutable: never modify a received ASPN object, build a new one. Registry keys and groups follow
the Python layout exactly (`config/BaseConfig.hpp` explains the convention). Logging goes through the mediator
(`ERROR` sets the process exit code); use `WARN` for recoverable conditions.

## Style

clang-format (`.clang-format`, Google-based, 120 columns), `-Wall -Wextra -Wpedantic` clean, no exceptions
across the plugin API except for programmer errors, `std::optional` for "may be absent", `shared_ptr<const T>`
for shared messages. clang-tidy with the checks in `.clang-tidy` runs in CI.

## Reporting results

`tools/run_acceptance.py` writes `docs/acceptance.json`, `tools/test_matrix.py` renders `docs/TEST_MATRIX.md`;
commit both when the numbers change and say in `PROGRESS.md` why they changed.
