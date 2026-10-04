# CI

The GitHub Actions workflow lives at `.github/workflows/ci.yml`: build with meson, unit suites,
`tools/ci_acceptance.py` on `testdata/example_60s.log` (compiled apps, config files through `cobra_run`,
the push API) and the C example.

Locally, the same checks are
`.venv/bin/meson test -C build && python3 tools/ci_acceptance.py && python3 tools/ci_acceptance.py --runner && python3 tools/ci_acceptance.py --via-push`.
