#!/usr/bin/env python3
"""Network acceptance: play the example log over UDP multicast with build/tools/lcm_log_player while
cobra_run runs configs/pos_ins_network.json, then evaluate the recorded solutions exactly like pos_ins.
    Cobra/.venv/bin/python tools/run_network_acceptance.py [--speed 25] [--out docs/network_acceptance.json]
Expect the pos_ins numbers (same pipeline, same order); differences point at UDP loss on this machine.
"""
import argparse, json, os, subprocess, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_acceptance as ra  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', default='build')
    ap.add_argument('--speed', type=float, default=25.0)
    ap.add_argument('--workdir', default='build/network')
    ap.add_argument('--input-log', default=None)
    ap.add_argument('--out', default='docs/network_acceptance.json')
    ap.add_argument('--mode', default='corrected')
    a = ap.parse_args()
    if a.input_log is None:
        from pntos_python_datasets_lcm import EXAMPLE_LCM_LOG
        a.input_log = EXAMPLE_LCM_LOG
    os.makedirs(a.workdir, exist_ok=True)
    out_log = os.path.join(a.workdir, 'pos_ins_network.log')
    flag = '--legacy-q' if a.mode == 'legacy' else '--corrected-q'
    t0 = time.time()
    app = subprocess.Popen([os.path.join(a.build, 'apps', 'cobra_run'), 'configs/pos_ins_network.json', out_log, flag, '--quiet'],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(1.0)  # let the receiver join the group
    player = subprocess.run([os.path.join(a.build, 'tools', 'lcm_log_player'), a.input_log, '--speed', str(a.speed),
                             '--channels', '/sensor/vn-100/imu,/sensor/ublox-ZED-F9T/position', '--start-delay', '0.5'],
                            capture_output=True, text=True)
    try:
        out, _ = app.communicate(timeout=60)
    except subprocess.TimeoutExpired:
        app.kill(); out, _ = app.communicate()
    wall = time.time() - t0
    lines = out.splitlines()
    rec = dict(app='pos_ins_network', mode=a.mode, speed=a.speed, wall_s=round(wall, 1), exit_code=app.returncode,
               errors=sum('[ERROR]' in l for l in lines), warnings=sum('[WARN]' in l for l in lines),
               player=player.stderr.strip()[-120:])
    print('reading truth ...', flush=True)
    truth, _ = ra.read_pva(a.input_log, ra.TRUTH)
    if os.path.exists(a.corrected_limits if hasattr(a, 'corrected_limits') else 'docs/limits_corrected.json') and a.mode == 'corrected':
        ra.CORRECTED_LIMITS.update(json.load(open('docs/limits_corrected.json'))['limits'])
    rec.update(ra.evaluate('pos_ins', out_log, truth, None, a.mode))
    if rec['errors']:
        rec['passed'] = False; rec['reason'] = 'ERROR logged: ' + next(l for l in lines if '[ERROR]' in l)[:160]
    c = rec.get('checks', {})
    print(f"pos_ins_network@{a.mode} {'PASS' if rec['passed'] else 'FAIL'} epochs={rec.get('epochs')} wall={rec['wall_s']}s "
          f"pos std {'/'.join(f'{v:.3g}' for v in c['pos']['std'])} vel std {'/'.join(f'{v:.3g}' for v in c['vel']['std'])} "
          f"tilt std {'/'.join(f'{v:.3g}' for v in c['tilt']['std'])} errors={rec['errors']} ({rec['player']})")
    json.dump(dict(generated=time.strftime('%Y-%m-%dT%H:%M:%S%z'), results={f'pos_ins_network@{a.mode}': rec}), open(a.out, 'w'), indent=2)
    print('wrote', a.out)
    return 0 if rec['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
