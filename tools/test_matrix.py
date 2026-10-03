#!/usr/bin/env python3
"""Generate docs/TEST_MATRIX.md from the last `meson test` run.

Usage:
    meson test -C build                      # produces build/meson-logs/testlog.junit.xml
    python3 tools/test_matrix.py [--build build] [--pos-ins-errors out_errors.json] [--out docs/TEST_MATRIX.md]

Sections: summary, per-suite table (with the Python test file each suite ports), every individual test
with its status, Python-test coverage matrix, and the application acceptance matrix. Only standard
library; no venv needed. `--pos-ins-errors` takes the JSON written by tools/compare_to_truth.py.
"""
import argparse, datetime, json, os, subprocess, xml.etree.ElementTree as ET

# C++ suite -> (Python origin, notes)
SUITES = {
    'smoke': ('—', 'toolchain: Eigen + aspn-generated link and message construction'),
    'api': ('—', 'Timestamp, Message::as<T>, registry value conversions'),
    'ekf_fusion_strategy': ('test_standard_ekf_fusion_strategy.py', 'all tests ported; Python arithmetic via set_joseph_form(false)'),
    'simple_state_blocks': ('test_fogm_block.py, test_constant_block.py, test_clock_bias_block.py', 'all tests ported'),
    'registry': ('test_registry.py', 'all but pickled Message permanency'),
    'state_modeling': ('test_state_modeling.py', 'all tests ported; Pinson Phi/Qd goldens'),
    'virtual_state_blocks': ('test_virtual_state_blocks.py (VSB half)', 'plus Jacobian finite-difference check'),
    'vsb_manager': ('test_virtual_state_blocks.py (manager half)', 'plus deep-copy check'),
    'fusion_engine': ('— (Python covers the engine via orchestration)', 'new C++ tests'),
    'message_stream_config': ('test_message_stream_config.py', 'all tests ported + source overrides'),
    'controller': ('test_single_threaded_controller.py', 'all tests ported + mediator behaviour'),
    'orchestration': ('test_orchestration.py (standard cases)', 'mock inertial/initializer; tutorial cases not ported'),
    'inertial': ('test_inertial_plugin.py', 'all tests ported + mechanization checks'),
    'initialization': ('test_manual_initialization_plugin.py, inertial_alignment/*', 'all tests ported + PVA-message strategy'),
    'preprocessors': ('test_preprocessor_plugin.py', 'all tests ported + time bias'),
    'lcm_transport': ('test_transport_plugin.py (log parts)', 'network LCM transport not ported'),
}

# Python test file -> (C++ suite or None, coverage note)
PYTHON_TESTS = [
    ('test_standard_ekf_fusion_strategy.py', 'ekf_fusion_strategy', 'complete'),
    ('test_fogm_block.py', 'simple_state_blocks', 'complete'),
    ('test_constant_block.py', 'simple_state_blocks', 'complete'),
    ('test_clock_bias_block.py', 'simple_state_blocks', 'complete'),
    ('test_registry.py', 'registry', 'complete except pickled Message permanency (deviation #5)'),
    ('test_state_modeling.py', 'state_modeling', 'complete'),
    ('test_virtual_state_blocks.py', 'virtual_state_blocks, vsb_manager', 'complete'),
    ('test_message_stream_config.py', 'message_stream_config', 'complete'),
    ('test_single_threaded_controller.py', 'controller', 'complete'),
    ('test_orchestration.py', 'orchestration', 'standard cases; tutorial-plugin cases and static-align-before-aligned pending'),
    ('test_inertial_plugin.py', 'inertial', 'complete'),
    ('test_manual_initialization_plugin.py', 'initialization', 'complete'),
    ('inertial_alignment/test_static_align_initialization_plugin.py', 'initialization', 'complete'),
    ('inertial_alignment/test_manual_heading_align_initialization_plugin.py', 'initialization', 'complete'),
    ('test_preprocessor_plugin.py', 'preprocessors', 'complete'),
    ('test_transport_plugin.py', 'lcm_transport', 'log transport only; network transport needs liblcm'),
    ('test_configutils.py', None, 'no generic config utils in C++; covered by per-config round-trip tests'),
    ('test_registry_views.py', None, 'UI layer (Tier 2)'),
    ('test_ui_utils.py', None, 'UI layer (Tier 2)'),
    ('test_cobra_ui_plugin.py', None, 'UI layer (Tier 2)'),
    ('test_diagnostic_log_plugin.py', None, 'diagnostic log plugin (Tier 2)'),
    ('test_hdf5utils.py', None, 'diagnostic log plugin (Tier 2)'),
    ('test_buscat_controller.py', None, 'Buscat controller (Tier 3)'),
    ('test_aspn_ros.py', None, 'ROS transport (Tier 3)'),
]

# Integration apps (COBRA_ANALYSIS.md section 14) -> (C++ status, note)
APPS = [
    ('test_dummy_app', 'PASS', 'apps/dummy/minimal runs and exits cleanly'),
    ('test_standard_pos_ins_app', 'NOT RUN', 'apps/standard/pos_ins'),
    ('test_tutorial_pos_ins_app', 'NOT PORTED', 'tutorial plugins'),
    ('test_tutorial_pos_ins_vel_app', 'NOT PORTED', 'tutorial plugins'),
    ('test_standard_pos_ins_record_states_app', 'NOT PORTED', 'needs the HDF5 diagnostic log plugin'),
    ('test_standard_pos_ins_leverarm_app', 'NOT RUN', 'apps/standard/pos_ins_leverarm'),
    ('test_standard_pos_bodyvel_ins_app', 'NOT RUN', 'apps/standard/pos_ins_bodyvel'),
    ('test_extras_pos_zerovel2d_ins_app', 'NOT PORTED', 'needs the extras zero-velocity preprocessor'),
    ('test_standard_pos_ins_vel_app', 'NOT RUN', 'apps/standard/pos_vel_ins'),
    ('test_standard_posvel_ins_app', 'NOT RUN', 'apps/standard/posvel_ins'),
    ('test_standard_outage_sim_app', 'NOT RUN', 'apps/standard/outage_sim'),
    ('test_standard_pos_ins_vsb_app', 'NOT RUN', 'apps/standard/pos_ins_vsb'),
    ('test_standard_direction_to_points_app', 'NOT RUN', 'apps/standard/direction_to_points'),
]


def git(*args):
    try:
        return subprocess.check_output(['git', *args], text=True).strip()
    except Exception:
        return '?'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', default='build')
    ap.add_argument('--out', default='docs/TEST_MATRIX.md')
    ap.add_argument('--pos-ins-errors', default=None)
    ap.add_argument('--acceptance', default='docs/acceptance.json')
    a = ap.parse_args()

    xml_path = os.path.join(a.build, 'meson-logs', 'testlog.junit.xml')
    root = ET.parse(xml_path).getroot()
    suites = {}
    for ts in root.iter('testsuite'):
        # name: pntos-cpp.pntos-cpp:<suite>.<Fixture>
        name = ts.get('name')
        suite = name.split(':', 1)[1].split('.', 1)[0] if ':' in name else name
        fixture = name.rsplit('.', 1)[-1]
        for tc in ts.iter('testcase'):
            status = 'PASS'
            if tc.find('failure') is not None or tc.find('error') is not None:
                status = 'FAIL'
            elif tc.find('skipped') is not None or tc.get('status') == 'notrun':
                status = 'SKIP'
            suites.setdefault(suite, []).append((fixture, tc.get('name'), status, tc.get('time', '')))

    total = sum(len(v) for v in suites.values())
    npass = sum(1 for v in suites.values() for t in v if t[2] == 'PASS')
    nfail = sum(1 for v in suites.values() for t in v if t[2] == 'FAIL')
    nskip = total - npass - nfail
    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    commit = git('rev-parse', '--short', 'HEAD')

    L = []
    L.append('# Test pass/fail matrix\n')
    L.append(f'Generated by `tools/test_matrix.py` from `{xml_path}` on {stamp}, commit `{commit}`. '
             'Regenerate with `meson test -C build && python3 tools/test_matrix.py`.\n')
    verdict = 'ALL GREEN' if nfail == 0 else f'{nfail} FAILING'
    L.append(f'**{len(suites)} suites, {total} tests: {npass} passed, {nfail} failed, {nskip} skipped — {verdict}.**\n')

    L.append('## 1. Suites\n')
    L.append('| Suite | Tests | Pass | Fail | Skip | Result | Ported from (Cobra `tests/`) | Notes |')
    L.append('|---|---:|---:|---:|---:|---|---|---|')
    for suite in SUITES:
        tests = suites.get(suite, [])
        p = sum(1 for t in tests if t[2] == 'PASS'); f = sum(1 for t in tests if t[2] == 'FAIL'); s = len(tests) - p - f
        res = 'PASS' if tests and f == 0 else ('FAIL' if f else 'MISSING')
        origin, note = SUITES[suite]
        L.append(f'| `{suite}` | {len(tests)} | {p} | {f} | {s} | {res} | {origin} | {note} |')
    for suite in suites:
        if suite not in SUITES:
            tests = suites[suite]; f = sum(1 for t in tests if t[2] == 'FAIL')
            L.append(f'| `{suite}` | {len(tests)} | {len(tests)-f} | {f} | 0 | {"FAIL" if f else "PASS"} | (not in SUITES table) | add to tools/test_matrix.py |')
    L.append('')

    L.append('## 2. Every test\n')
    for suite, tests in suites.items():
        f = sum(1 for t in tests if t[2] == 'FAIL')
        L.append(f'### `{suite}` — {len(tests)} tests, {"all passing" if f == 0 else f"{f} failing"}\n')
        L.append('| Fixture | Test | Result |')
        L.append('|---|---|---|')
        for fixture, name, status, _ in tests:
            L.append(f'| {fixture} | {name} | {status} |')
        L.append('')

    L.append('## 3. Python test coverage\n')
    L.append('Status of every Python test file in `Cobra/pntos-cobra/tests/` relative to the C++ suites.\n')
    L.append('| Python test file | C++ suite(s) | C++ result | Coverage |')
    L.append('|---|---|---|---|')
    for pyfile, cpp, note in PYTHON_TESTS:
        if cpp is None:
            L.append(f'| `{pyfile}` | — | NOT PORTED | {note} |')
            continue
        names = [c.strip() for c in cpp.split(',')]
        fails = sum(1 for n in names for t in suites.get(n, []) if t[2] == 'FAIL')
        present = all(n in suites for n in names)
        res = 'PASS' if present and fails == 0 else ('FAIL' if fails else 'MISSING')
        L.append(f'| `{pyfile}` | {", ".join("`"+n+"`" for n in names)} | {res} | {note} |')
    L.append('')

    L.append('## 4. Application acceptance matrix\n')
    L.append('The 13 Python integration apps (`COBRA_ANALYSIS.md` section 14) and their C++ status. PASS means the C++ app '
             'runs the full example log and `tools/compare_to_truth.py` reproduces the Python accuracy.\n')
    L.append('| Python integration test | C++ status | Notes |')
    L.append('|---|---|---|')
    acc = json.load(open(a.acceptance))['results'] if os.path.exists(a.acceptance) else {}
    by_test = {}
    for v in acc.values():
        if 'python_test' in v:
            by_test.setdefault(v['python_test'], {})[v.get('mode', 'corrected')] = v
    L[-2] = '| Python integration test | C++ corrected Q | C++ legacy Q (Python-compatible) | Notes |'
    L[-1] = '|---|---|---|---|'
    def cell(r):
        if r is None: return '—'
        if not r.get('checks'): return f"FAIL ({r.get('reason', '?')})"
        c = r['checks']
        return (f"{'PASS' if r['passed'] else 'FAIL'}: pos {'/'.join(f'{v:.2f}' for v in c['pos']['std'])} m, "
                f"vel {'/'.join(f'{v:.3f}' for v in c['vel']['std'])}, tilt {'/'.join(f'{v:.3f}' for v in c['tilt']['std'])} deg"
                + (f" ({r['reason']})" if not r['passed'] and r.get('reason') else ''))
    for name, status, note in APPS:
        modes = by_test.get(name)
        if modes:
            r = modes.get('corrected') or next(iter(modes.values()))
            note = f"epochs {r.get('epochs')} (py {r.get('expected_epochs')}), wall {r.get('wall_s', '?')} s"
            L.append(f'| {name} | {cell(modes.get("corrected"))} | {cell(modes.get("legacy"))} | {note} |')
        else:
            L.append(f'| {name} | {status} | {status} | {note} |')
    L.append('')
    L.append('Values are per-axis error standard deviations (N/E/D or roll/pitch/yaw) checked against the Python '
             "integration-test limits together with max-error and sigma-coverage checks. 'Legacy Q' reproduces the Python "
             'process-noise rotation bug (`PinsonStateBlockConfig::legacy_q_rotation`, app flag `--legacy-q`) and is the '
             'apples-to-apples comparison; the limits were tuned on that behaviour.\n')
    if acc:
        L.append(f'App results from `{a.acceptance}` (generated {json.load(open(a.acceptance))["generated"]}; '
                 'regenerate with `Cobra/.venv/bin/python tools/run_acceptance.py`). Limits are the Python integration test limits; '
                 'the epoch count is allowed to differ by up to 5 (see DESIGN.md section 8, deviation 14).\n')
    if a.pos_ins_errors and os.path.exists(a.pos_ins_errors):
        e = json.load(open(a.pos_ins_errors))
        L.append('### pos_ins acceptance numbers (C++, from `--pos-ins-errors`)\n')
        L.append('| Metric | N | E | D |')
        L.append('|---|---:|---:|---:|')
        L.append('| Position RMS [m] | ' + ' | '.join(f'{v:.3f}' for v in e['pos_rms_ned_m']) + ' |')
        L.append('| Position max abs [m] | ' + ' | '.join(f'{v:.2f}' for v in e['pos_max_ned_m']) + ' |')
        L.append('| Velocity RMS [m/s] | ' + ' | '.join(f'{v:.4f}' for v in e['vel_rms_ned_mps']) + ' |')
        L.append('| Attitude RMS r/p/y [deg] | ' + ' | '.join(f'{v:.3f}' for v in e['att_rms_rpy_deg']) + ' |')
        L.append(f'\n{e["epochs"]} epochs over {e["span_s"]:.0f} s. Python baseline: position 0.921 / 1.235 / 1.667 m, '
                 'velocity 0.0837 / 0.0930 / 0.0431 m/s, attitude 0.089 / 0.078 / 0.810 deg (same tool).\n')
    else:
        L.append('pos_ins numbers: see `docs/DESIGN.md` section 9.5 (pass `--pos-ins-errors <out>_errors.json` to embed the latest run).\n')

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write('\n'.join(L))
    print(f'wrote {a.out}: {len(suites)} suites, {total} tests, {npass} pass, {nfail} fail, {nskip} skip')


if __name__ == '__main__':
    main()
