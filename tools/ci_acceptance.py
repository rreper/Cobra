#!/usr/bin/env python3
"""Short acceptance for CI: every app on the 60 s cut of the example log (testdata/example_60s.log).

Checks exit code 0, no [ERROR] log lines, a plausible epoch count and loose RMS bounds (the cut is too
short for the Python limits; the full run is tools/run_acceptance.py). Needs only the C++ binaries:
    python3 tools/ci_acceptance.py [--build build] [--log testdata/example_60s.log] [--runner]
"""
import argparse, json, os, subprocess, sys

APPS = {  # app -> (min solution epochs, max pos RMS m, max yaw RMS deg)
    'pos_ins': (40, 5.0, 3.0), 'pos_vel_ins': (40, 5.0, 3.0), 'posvel_ins': (40, 5.0, 3.0),
    'pos_ins_leverarm': (40, 10.0, 3.0), 'pos_ins_bodyvel': (40, 5.0, 3.0), 'outage_sim': (40, 5.0, 3.0),
    'pos_ins_vsb': (40, 5.0, 3.0), 'direction_to_points': (40, 30.0, 3.0), 'pos_ins_record_states': (40, 5.0, 3.0),
    'pos_ins_zerovel2d': (40, 5.0, 3.0), 'tutorial_pos_ins': (50, 5.0, 3.0), 'tutorial_pos_vel_ins': (50, 5.0, 3.0),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', default='build')
    ap.add_argument('--log', default='testdata/example_60s.log')
    ap.add_argument('--workdir', default='build/ci_acceptance')
    ap.add_argument('--runner', action='store_true', help='run through build/apps/cobra_run with configs/<app>.json')
    ap.add_argument('--via-push', action='store_true', help='replay through the push API (cobra::Filter) instead of the log transport')
    ap.add_argument('--modes', default='corrected,legacy')
    a = ap.parse_args()
    os.makedirs(a.workdir, exist_ok=True)
    stats_exe = os.path.join(a.build, 'tools', 'log_stats')
    failures = []
    for app, (min_epochs, max_pos, max_yaw) in APPS.items():
        for mode in a.modes.split(','):
            out = os.path.join(a.workdir, f'{app}_{mode}.log')
            flag = '--legacy-q' if mode == 'legacy' else '--corrected-q'
            if a.runner:
                cmd = [os.path.join(a.build, 'apps', 'cobra_run'), os.path.join('configs', app + '.json'), out, a.log, flag, '--quiet', '--no-record-input']
            else:
                cmd = [os.path.join(a.build, 'apps', app), out, a.log, flag, '--quiet', '--no-record-input']
            if a.via_push:
                cmd.append('--via-push')
            p = subprocess.run(cmd, capture_output=True, text=True)
            errors = [l for l in (p.stdout + p.stderr).splitlines() if '[ERROR]' in l]
            s = json.loads(subprocess.check_output([stats_exe, out, '--truth-log', a.log], text=True)) if p.returncode == 0 else {}
            ok = (p.returncode == 0 and not errors and s.get('nan_epochs', 1) == 0 and s.get('solution_epochs', 0) >= min_epochs
                  and max(s.get('rms_pos_ned_m', [1e9])) <= max_pos and s.get('rms_rpy_deg', [1e9])[-1] <= max_yaw)
            print(f"{app:24s} {mode:9s} {'PASS' if ok else 'FAIL'} exit={p.returncode} errors={len(errors)} "
                  f"epochs={s.get('solution_epochs')} pos_rms={[round(v, 2) for v in s.get('rms_pos_ned_m', [])]} "
                  f"yaw_rms={round(s.get('rms_rpy_deg', [0, 0, 0])[-1], 3)}", flush=True)
            if not ok:
                failures.append(f'{app}@{mode}')
                if errors: print('   ', errors[0][:200])
    print('CI acceptance:', 'PASS' if not failures else 'FAIL ' + ', '.join(failures))
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
