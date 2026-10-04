# CI

`github-ci.yml` is the GitHub Actions workflow (build with meson, unit suites, `tools/ci_acceptance.py` on
`testdata/example_60s.log`, compiled apps and config files). It lives here instead of `.github/workflows/`
because the token used for pushing lacks the `workflow` scope that GitHub requires to create or change
workflow files. To enable it: give the token the `workflow` scope (classic PAT settings), then

```bash
git mv ci/github-ci.yml .github/workflows/ci.yml && git commit -m "Enable CI" && git push
```

Locally, the same checks are `.venv/bin/meson test -C build && python3 tools/ci_acceptance.py && python3 tools/ci_acceptance.py --runner`.
