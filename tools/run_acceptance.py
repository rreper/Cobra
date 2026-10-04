#!/usr/bin/env python3
"""Run the C++ apps on the example log and validate them like Cobra's integration tests.

Usage (with the Cobra venv's python):
    Cobra/.venv/bin/python tools/run_acceptance.py [--build build] [--out docs/acceptance.json] [--only pos_ins,...]
                                                  [--no-run] [--workdir /tmp/pntos_acceptance]

For every app in LIMITS it runs `build/apps/<app> <workdir>/<app>.log`, reads the solution channel, and
applies the same checks as pntos-cobra-apps/test/integration_test.py::validate_results: exact point count,
no NaNs, start/end within 3 s of truth, and for position / velocity / tilt errors: percent of |err| within
1/2/3 sigma, std below threshold, max below threshold. Tilt error is the RPY of C_sol * C_truth^T.
Writes a JSON summary consumed by tools/test_matrix.py.

Known deviation: the C++ mediator publishes on raw message times while Python publishes on preprocessor-
adjusted times (Python mutates messages in place), so the C++ epoch count differs by a few from Python's
`num_points`; the count check here accepts +-5.
"""
import argparse, json, math, os, subprocess, sys, time
import numpy as np
from lcm import EventLog
from aspn23_lcm import measurement_position_velocity_attitude as PVA

sys.path.insert(0, os.path.dirname(__file__))
from compare_to_truth import quat_to_rpy, radii  # noqa: E402

TRUTH = '/sensor/ins-d/pva'
SOLUTION = '/solution/pntos/pva'

def L(std, mx, p1=68.0, p2=95.0, p3=99.0):
    return dict(std=std, max=mx, p1=p1, p2=p2, p3=p3)

# app -> (python integration test, num_points, start offset, pos limits, vel limits, tilt limits)
LIMITS = {
    'pos_ins': ('test_standard_pos_ins_app', 2570, 10.0, L(1.4, 3.8, 63), L(0.1, 0.8), L(0.81, 3.5)),
    'pos_ins_leverarm': ('test_standard_pos_ins_leverarm_app', 2570, 10.0, L(2.0, 4.1, 60), L(0.11, 1.0), L(0.81, 2.5, 53, 91)),
    'pos_ins_bodyvel': ('test_standard_pos_bodyvel_ins_app', 2570, 10.0, L(1.3, 4.8, 59, 90), L(0.11, 0.8, 59, 88, 97), L(1.7, 12.0, 53, 91, 98)),
    'pos_vel_ins': ('test_standard_pos_ins_vel_app', 2570, 10.0, L(1.4, 4.7, 5, 12, 24), L(0.14, 1.1, 47, 77, 87), L(1.6, 5.2, 30, 52, 68)),
    'posvel_ins': ('test_standard_posvel_ins_app', 2570, 10.0, L(1.4, 4.7, 5, 12, 24), L(0.14, 1.1, 47, 77, 87), L(1.6, 5.2, 30, 52, 68)),
    'outage_sim': ('test_standard_outage_sim_app', 2570, 10.0, L(306, 2441, 64), L(8, 35, 68, 91, 98), L(1.32, 5.3, 68, 95, 98)),
    'pos_ins_vsb': ('test_standard_pos_ins_vsb_app', 2570, 10.0, L(1.4, 3.8), L(0.1, 0.8), L(0.82, 3.55)),
    'direction_to_points': ('test_standard_direction_to_points_app', 2570, 10.0, L(20.0, 200.0, 55, 85, 95), L(1.0, 8.0, 55, 85, 95), L(0.6, 3.5, 55, 85, 95)),
    'pos_ins_record_states': ('test_standard_pos_ins_record_states_app', 2570, 10.0, L(1.4, 3.8, 63), L(0.1, 0.8), L(0.81, 3.5)),
    'pos_ins_zerovel2d': ('test_extras_pos_zerovel2d_ins_app', 2570, 10.0, L(1.4, 3.8), L(0.1, 0.8), L(0.8, 3.5)),
    'tutorial_pos_ins': ('test_tutorial_pos_ins_app', 2593, 0.0, L(2.0, 4.0, 60), L(0.11, 1.0), L(0.85, 3.5, 48, 91)),
    'tutorial_pos_vel_ins': ('test_tutorial_pos_ins_vel_app', 2593, 0.0, L(2.0, 4.5, 40, 70, 95), L(0.2, 1.5, 55, 75, 85), L(2.0, 6.0, 20, 50, 60)),
}

def rpy_to_dcm(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # C_platform_to_nav (ZYX)
    return np.array([[cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
                     [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
                     [-sp,   sr*cp,            cr*cp]])

def dcm_to_rpy(C):
    return np.array([math.atan2(C[2,1], C[2,2]), math.asin(max(-1.0, min(1.0, -C[2,0]))), math.atan2(C[1,0], C[0,0])])

def read_pva(path, channel):
    fp = PVA._get_packed_fingerprint()
    rows, sigs = [], []
    for ev in EventLog(path):
        if ev.channel != channel or ev.data[:8] != fp:
            continue
        m = PVA.decode(ev.data)
        rows.append([m.time_of_validity.elapsed_nsec / 1e9, m.p1, m.p2, m.p3, m.v1, m.v2, m.v3, *m.quaternion])
        cov = np.array(m.covariance)
        sigs.append(np.sqrt(np.diag(cov)) if cov.size == 81 else np.full(9, np.nan))
    return np.array(rows), np.array(sigs)

def check(err, sig, lim):
    a = np.abs(err)
    pct = [float(np.mean(a <= sig * k, axis=0).min() * 100) for k in (1, 2, 3)]
    std = np.std(err, axis=0); mx = np.max(a, axis=0)
    ok = (pct[0] >= lim['p1'] and pct[1] >= lim['p2'] and pct[2] >= lim['p3'] and
          np.all(std < lim['std']) and np.all(mx < lim['max']))
    return dict(passed=bool(ok), std=[float(x) for x in std], max=[float(x) for x in mx],
                rms=[float(x) for x in np.sqrt(np.mean(err**2, axis=0))],
                pct_within_1_2_3_sigma=pct, limits=lim)

CORRECTED_LIMITS = {}


def limits_for(app, mode):
    name, n_expected, start_off, pl, vl, tl = LIMITS[app]
    if mode == 'corrected' and app in CORRECTED_LIMITS:
        c = CORRECTED_LIMITS[app]
        pick = lambda m: dict(std=m['std'], max=m['max'], p1=m['p1'], p2=m['p2'], p3=m['p3'])
        pl, vl, tl = (pick(c['pos']), pick(c['vel']), pick(c['tilt']))
    return name, n_expected, start_off, pl, vl, tl


def evaluate(app, log_path, truth, tsig_unused, mode='legacy'):
    name, n_expected, start_off, pl, vl, tl = limits_for(app, mode)
    sol, sig = read_pva(log_path, SOLUTION)
    out = dict(python_test=name, epochs=int(len(sol)), expected_epochs=n_expected, checks={})
    if len(sol) == 0:
        out['passed'] = False; out['reason'] = 'no solutions'; return out
    t = sol[:, 0]; tt = truth[:, 0]
    nan_free = not (np.isnan(sol).any() or np.isnan(sig).any())
    starts_ok = abs(t[0] - start_off - tt[0]) < 3.0 and abs(t[-1] - tt[-1]) < 3.0
    count_ok = abs(len(sol) - n_expected) <= 5
    interp = lambda col: np.interp(t, tt, truth[:, col])
    lat_t, alt_t = interp(1), interp(3)
    rn, re = np.vectorize(radii)(lat_t)
    ned_err = np.stack([(sol[:, 1] - lat_t) * (rn + alt_t), (sol[:, 2] - interp(2)) * (re + alt_t) * np.cos(lat_t),
                        -(sol[:, 3] - alt_t)], axis=1)
    vel_err = sol[:, 4:7] - np.stack([interp(4), interp(5), interp(6)], axis=1)
    rpy_s = np.array([quat_to_rpy(q) for q in sol[:, 7:11]])
    rpy_t_all = np.array([quat_to_rpy(q) for q in truth[:, 7:11]])
    rpy_t = np.stack([np.interp(t, tt, np.unwrap(rpy_t_all[:, k])) for k in range(3)], axis=1)
    tilt_err = np.degrees(np.array([dcm_to_rpy(rpy_to_dcm(a) @ rpy_to_dcm(b).T) for a, b in zip(rpy_s, rpy_t)]))
    out['checks']['pos'] = check(ned_err, sig[:, 0:3], pl)
    out['checks']['vel'] = check(vel_err, sig[:, 3:6], vl)
    out['checks']['tilt'] = check(tilt_err, np.degrees(sig[:, 6:9]), tl)
    out['checks']['nan_free'] = nan_free
    out['checks']['start_end_within_3s'] = bool(starts_ok)
    out['checks']['epoch_count_within_5'] = bool(count_ok)
    out['passed'] = bool(nan_free and starts_ok and count_ok and all(out['checks'][k]['passed'] for k in ('pos', 'vel', 'tilt')))
    return out

def check_hdf5(path):
    """Opens the diagnostic log written by pos_ins_record_states with h5py and checks its datasets."""
    try:
        import h5py
        with h5py.File(path, 'r') as f:
            keys = sorted(f.keys())
            n = int(f['time'].shape[0])
            est = f['estimate'].shape
            labels = [l.decode() for l in f['state_labels'][0]]
            ok = keys == ['estimate', 'sigma', 'state_labels', 'time'] and est[0] == n and est[1] == len(labels) and n > 1000
            return dict(ok=bool(ok), path=path, records=n, states=len(labels), keys=keys)
    except Exception as e:  # noqa: BLE001
        return dict(ok=False, path=path, error=f'{type(e).__name__}: {e}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', default='build')
    ap.add_argument('--out', default='docs/acceptance.json')
    ap.add_argument('--workdir', default='build/acceptance',
                    help='output logs go here; with --no-record-input (default) each is ~1 MB instead of a copy of the input log')
    ap.add_argument('--record-input', action='store_true', help='keep the input channels in the output logs (476 MB each)')
    ap.add_argument('--only', default=None)
    ap.add_argument('--no-run', action='store_true')
    ap.add_argument('--input-log', default=None)
    ap.add_argument('--runner', default=None,
                    help='run every app through this generic runner with configs/<app>.json instead of the compiled app '
                         '(e.g. build/apps/cobra_run); results must match the compiled apps')
    ap.add_argument('--configs', default='configs')
    ap.add_argument('--modes', default='corrected,legacy',
                    help='comma list of: corrected (app default: Q rotated from a copy, retuned VN-100 model, --corrected-q), '
                         'legacy (Python-compatible Pinson Q rotation and tuning, --legacy-q)')
    ap.add_argument('--corrected-limits', default='docs/limits_corrected.json',
                    help='limits for the corrected mode (derived from a corrected run; the Python limits are used where it is missing)')
    ap.add_argument('--derive-corrected-limits', action='store_true',
                    help='after running, write --corrected-limits from the corrected results: std limits max(Python, 1.05 x measured), '
                         'max limits max(Python, 1.10 x measured), sigma-coverage limits min(Python, measured - 2)')
    a = ap.parse_args()
    if a.corrected_limits and os.path.exists(a.corrected_limits) and not a.derive_corrected_limits:
        CORRECTED_LIMITS.update(json.load(open(a.corrected_limits))['limits'])
    if a.input_log is None:
        from pntos_python_datasets_lcm import EXAMPLE_LCM_LOG
        a.input_log = EXAMPLE_LCM_LOG
    os.makedirs(a.workdir, exist_ok=True)
    apps = a.only.split(',') if a.only else list(LIMITS)
    print('reading truth ...', flush=True)
    truth, _ = read_pva(a.input_log, TRUTH)
    results = {}
    modes = a.modes.split(',')
    for app in apps:
      for mode in modes:
        key = f'{app}@{mode}'
        exe = os.path.join(a.build, 'apps', app)
        log = os.path.join(a.workdir, key.replace('@', '_') + '.log')
        rec = dict(app=app, mode=mode)
        if a.runner:
            exe = a.runner; rec['runner'] = a.runner; rec['config'] = os.path.join(a.configs, app + '.json')
        if not os.path.exists(exe):
            rec.update(passed=False, reason='binary not built'); results[key] = rec; continue
        if not a.no_run:
            t0 = time.time()
            cmd = [exe] + ([rec['config']] if a.runner else []) + [log, a.input_log] + (['--legacy-q'] if mode == 'legacy' else ['--corrected-q']) \
                + ([] if a.record_input else ['--no-record-input', '--quiet'])
            p = subprocess.run(cmd, capture_output=True, text=True)
            rec['wall_s'] = round(time.time() - t0, 2)
            rec['exit_code'] = p.returncode
            errs = [l for l in p.stdout.splitlines() + p.stderr.splitlines() if '[ERROR]' in l]
            warns = [l for l in p.stdout.splitlines() + p.stderr.splitlines() if '[WARN]' in l]
            rec['errors'] = len(errs); rec['warnings'] = len(warns)
            if errs: rec['first_error'] = errs[0][:200]
            if p.returncode != 0:
                rec['passed'] = False; rec['reason'] = f'exit code {p.returncode}'; results[key] = rec; continue
        rec.update(evaluate(app, log, truth, None, mode))
        rec['limits_source'] = 'derived (docs/limits_corrected.json)' if mode == 'corrected' and app in CORRECTED_LIMITS else 'python'
        if app == 'pos_ins_record_states' and not a.no_run:
            rec['hdf5'] = check_hdf5(log[:-4] + '.hdf5')
            if not rec['hdf5'].get('ok'):
                rec['passed'] = False; rec['reason'] = 'hdf5: ' + rec['hdf5'].get('error', '?')
        if rec.get('errors'):
            rec['passed'] = False; rec['reason'] = 'ERROR logged'
        if not rec['passed'] and 'reason' not in rec:
            c = rec.get('checks', {})
            rec['reason'] = 'failed: ' + ', '.join(k for k in ('pos', 'vel', 'tilt') if k in c and not c[k]['passed'])
        results[key] = rec
        c = rec.get('checks', {})
        fmt = lambda k, kk: ' / '.join(f'{v:.3g}' for v in c[k][kk]) if k in c else '-'
        print(f"{key:30s} {'PASS' if rec['passed'] else 'FAIL':4s} epochs={rec.get('epochs','-')} wall={rec.get('wall_s','-')}s "
              f"pos std {fmt('pos','std')} max {fmt('pos','max')} | vel std {fmt('vel','std')} | tilt std {fmt('tilt','std')}"
              + (f"  ({rec.get('reason')})" if not rec['passed'] else ''), flush=True)
    if a.derive_corrected_limits:
        derived = {}
        for key, rec in results.items():
            if rec.get('mode') != 'corrected' or not rec.get('checks'):
                continue
            app = rec['app']; _, _, _, pl, vl, tl = LIMITS[app]; out = {}
            for metric, py in (('pos', pl), ('vel', vl), ('tilt', tl)):
                c = rec['checks'][metric]
                meas_std = max(c['std']); meas_max = max(c['max']); p1, p2, p3 = c['pct_within_1_2_3_sigma']
                out[metric] = dict(std=round(max(py['std'], 1.05 * meas_std), 4), max=round(max(py['max'], 1.10 * meas_max), 3),
                                   p1=min(py['p1'], math.floor(p1) - 2), p2=min(py['p2'], math.floor(p2) - 2), p3=min(py['p3'], math.floor(p3) - 2),
                                   measured=dict(std=[round(v, 4) for v in c['std']], max=[round(v, 3) for v in c['max']],
                                                 pct=[round(v, 1) for v in (p1, p2, p3)]))
            derived[app] = out
        with open(a.corrected_limits, 'w') as f:
            json.dump(dict(generated=time.strftime('%Y-%m-%dT%H:%M:%S%z'), rule='std: max(python, 1.05*measured); max: max(python, 1.10*measured); '
                           'pct: min(python, floor(measured)-2); measured with the retuned VN-100 model (preset vn100_corrected)',
                           limits=derived), f, indent=2)
        print('wrote', a.corrected_limits)
    summary = dict(generated=time.strftime('%Y-%m-%dT%H:%M:%S%z'), input_log=a.input_log, results=results)
    with open(a.out, 'w') as f:
        json.dump(summary, f, indent=2)
    print('wrote', a.out)

if __name__ == '__main__':
    main()
