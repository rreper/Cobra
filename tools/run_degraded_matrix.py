#!/usr/bin/env python3
"""Degraded-sensor acceptance matrix (roadmap Phase 2).

Emulates worse sensors from the one example log: every row patches configs/pos_ins.json (or another base
config) with a SensorDegradationConfig preprocessor and, for the IMU grades, a correspondingly widened IMU
model, runs it through build/apps/cobra_run and evaluates the solution against truth exactly as
tools/run_acceptance.py does (std / max / sigma coverage of position, velocity and tilt errors).

    Cobra/.venv/bin/python tools/run_degraded_matrix.py                 # run, compare to docs/limits_degraded.json
    Cobra/.venv/bin/python tools/run_degraded_matrix.py --derive-limits  # (re)write the limits from this run

Writes docs/degraded_matrix.json and docs/DEGRADED_MATRIX.md.
"""
import argparse, copy, json, math, os, subprocess, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_acceptance as ra  # noqa: E402

IMU = '/sensor/vn-100/imu'
POS = '/sensor/ublox-ZED-F9T/position'

# IMU grades: injected noise density / bias, and the factor applied to the filter's random-walk and bias
# sigmas so that the model stays consistent with the data it sees.
IMU_GRADES = {
    'vn100': dict(accel_nd=0, gyro_nd=0, accel_bias=0, gyro_bias=0, rw_scale=1.0, bias_scale=1.0,
                  note='the recorded VN-100, unchanged'),
    'industrial_x3': dict(accel_nd=1.0e-5, gyro_nd=1.7e-3, accel_bias=2e-3, gyro_bias=2e-4, rw_scale=3.0, bias_scale=2.0,
                          note='white noise about 3x the tuned VN-100 random walk, 0.2 mg / 40 deg/h biases; model widened 3x / 2x'),
    'consumer_x10': dict(accel_nd=3.5e-5, gyro_nd=6.0e-3, accel_bias=1e-2, gyro_bias=1e-3, rw_scale=10.0, bias_scale=5.0,
                         note='white noise about 10x, 1 mg / 200 deg/h biases; model widened 10x / 5x'),
}
# GNSS conditions: position rate (downsampling of the 1 Hz stream), noise and outages.
GNSS = {
    '1hz': dict(downsample=1, pos_sigma=0, cov_scale=1.0, outages=[], note='as recorded, 1 Hz'),
    '0.2hz': dict(downsample=5, pos_sigma=0, cov_scale=1.0, outages=[], note='one fix every 5 s'),
    '0.1hz': dict(downsample=10, pos_sigma=0, cov_scale=1.0, outages=[], note='one fix every 10 s'),
    'noisy_x3': dict(downsample=1, pos_sigma=3.0, cov_scale=9.0, outages=[], note='3 m NED noise added, covariance x9'),
    'gaps_3x60s': dict(downsample=1, pos_sigma=0, cov_scale=1.0, outages=[(600, 660), (1200, 1260), (1800, 1860)],
                       note='three 60 s outages'),
    'jump_50m': dict(downsample=1, pos_sigma=0, cov_scale=1.0, outages=[], jumps=[[900.0, 50.0, 0.0, 0.0]],
                     note='one 50 m north outlier at 900 s, gate 0.999 on the position processor'),
}
# Rows actually run: all IMU grades at 1 Hz, all GNSS conditions with the VN-100, plus two sensor sets.
ROWS = [(g, '1hz') for g in IMU_GRADES] + [('vn100', c) for c in GNSS if c != '1hz']
# Two position sources (GNSS + a synthetic 5 m source derived from truth, both gated at 0.999): what gating alone
# does against a spoofed GNSS. Faults are applied by an extra degradation instance on the GNSS channel.
TWO_SOURCE_FAULTS = {
    'two_sources_clean': dict(note='GNSS + synthetic second source, no fault'),
    'ramp2_gnss': dict(ramps=[[600.0, 1.4, 1.4, 0.0, 0.0]], note='GNSS pulled at 2 m/s (diagonal) from 600 s: the slow-pull spoofer'),
    'ramp05_gnss': dict(ramps=[[600.0, 0.35, 0.35, 0.0, 0.0]], note='GNSS pulled at 0.5 m/s from 600 s: too slow for the gate alone, the solution follows GNSS (hundreds of m) until the second source disagrees; the motivating case for solution separation'),
    'step30_gnss': dict(jumps=[[600.0, 30.0, 0.0, 0.0]], note='one 30 m GNSS step at 600 s'),
    'cell_fault': dict(cell_ramps=[[600.0, 1.4, 1.4, 0.0, 0.0]], note='the second source pulled at 2 m/s instead of GNSS'),
}
SENSOR_SETS = {  # extra rows from other base configs
    'pos+vel (pos_vel_ins)': 'configs/pos_vel_ins.json',
    'pos+zero-velocity (pos_ins_zerovel2d)': 'configs/pos_ins_zerovel2d.json',
    'pos+baro MSL via geoid (pos_ins_baro)': 'configs/pos_ins_baro.json',
}


def patched_config(base, grade, cond, gate=None):
    cfg = copy.deepcopy(base)
    g, c = IMU_GRADES[grade], GNSS[cond]
    if 'advanced' not in cfg['app'].get('preprocessors', []):  # the degradation preprocessor lives in the extras plugin
        cfg['app']['preprocessors'] = cfg['app'].get('preprocessors', ['standard']) + ['advanced']
    for e in cfg['configs']:
        if e['type'] != 'StandardOrchestrationConfig':
            continue
        pp = e['preprocessor_configs']
        deg = dict(type='SensorDegradationConfig', group='config/degradation', channels=[IMU, POS], seed=1, imu_expected_dt=0.01,
                   accel_noise_density=[g['accel_nd']] * 3, gyro_noise_density=[g['gyro_nd']] * 3,
                   accel_bias=[g['accel_bias']] * 3, gyro_bias=[g['gyro_bias']] * 3,
                   position_noise_sigma_ned=[c['pos_sigma']] * 3, position_covariance_scale=c['cov_scale'],
                   position_jumps=c.get('jumps', []))
        # after rotation / time adjustment / time bias so that it sees what the filter would see
        pp.append(deg)
        if c['downsample'] > 1:
            pp.append(dict(type='DownsamplerConfig', group='config/degrade_downsampler', channels=[POS], downsampling_factors=[c['downsample']]))
        for i, (s, t) in enumerate(c['outages']):
            pp.append(dict(type='OutageConfig', group=f'config/degrade_outage{i}', channels=[POS], start_time=s, end_time=t))
        # widen the IMU model consistently with the injected errors
        for imu_holder in (e['pinson_sb_config'], e['alignment_config']):
            m = imu_holder['imu_model']
            if 'preset' in m:
                from_preset = {'vn100': None}
                # expand the preset so that it can be scaled: the runner would otherwise re-resolve the name
                m.update(expand_preset(m['preset']))
                del m['preset']
            for k in ('accel_random_walk_sigma', 'gyro_random_walk_sigma'):
                m[k] = [v * g['rw_scale'] for v in m[k]]
            for k in ('accel_bias_sigma', 'gyro_bias_sigma'):
                m[k] = [v * g['bias_scale'] for v in m[k]]
        if c.get('jumps'):
            for mp in e['mp_configs']:
                if mp['channel'] == POS:
                    mp['innovation_gate_probability'] = 0.999
    return cfg


_PRESETS = None


def expand_preset(name):
    """The numeric fields of an IMU preset, read once from `cobra_run --list-presets --json`."""
    global _PRESETS
    if _PRESETS is None:
        _PRESETS = json.loads(subprocess.check_output([RUNNER, '--list-presets', '--json'], text=True))['imu']
    p = dict(_PRESETS[name])
    p.pop('name', None); p.pop('description', None); p.pop('source', None)
    return p


RUNNER = 'build/apps/cobra_run'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', default='build')
    ap.add_argument('--workdir', default='build/degraded')
    ap.add_argument('--input-log', default=None)
    ap.add_argument('--out', default='docs/degraded_matrix.json')
    ap.add_argument('--md', default='docs/DEGRADED_MATRIX.md')
    ap.add_argument('--limits', default='docs/limits_degraded.json')
    ap.add_argument('--derive-limits', action='store_true')
    ap.add_argument('--only', default=None, help='comma list of row names')
    a = ap.parse_args()
    global RUNNER
    RUNNER = os.path.join(a.build, 'apps', 'cobra_run')
    if a.input_log is None:
        from pntos_python_datasets_lcm import EXAMPLE_LCM_LOG
        a.input_log = EXAMPLE_LCM_LOG
    os.makedirs(a.workdir, exist_ok=True)
    limits = json.load(open(a.limits))['limits'] if os.path.exists(a.limits) and not a.derive_limits else {}
    print('reading truth ...', flush=True)
    truth, _ = ra.read_pva(a.input_log, ra.TRUTH)
    base = json.load(open('configs/pos_ins.json'))
    rows = []
    for grade, cond in ROWS:
        rows.append((f'imu={grade} gnss={cond}', patched_config(base, grade, cond), IMU_GRADES[grade]['note'] + '; ' + GNSS[cond]['note']))
    for name, path in SENSOR_SETS.items():
        rows.append((name, json.load(open(path)), f'base config {path}'))
    two = json.load(open('configs/pos_ins_two_sources.json'))
    for name, f in TWO_SOURCE_FAULTS.items():
        cfg = copy.deepcopy(two)
        for e in cfg['configs']:
            if e['type'] != 'StandardOrchestrationConfig':
                continue
            if f.get('ramps') or f.get('jumps'):
                e['preprocessor_configs'].append(dict(type='SensorDegradationConfig', group='config/gnss_fault', channels=[POS], seed=3,
                                                      position_ramps=f.get('ramps', []), position_jumps=f.get('jumps', [])))
            if f.get('cell_ramps'):
                e['preprocessor_configs'].append(dict(type='SensorDegradationConfig', group='config/cell_fault', channels=['/synthetic/cell/position'],
                                                      seed=5, position_ramps=f['cell_ramps']))
        rows.append((name, cfg, f['note']))
    results = {}
    for name, cfg, note in rows:
        if a.only and name not in a.only.split(','):
            continue
        slug = name.replace(' ', '_').replace('=', '-').replace('/', '_').replace('(', '').replace(')', '').replace('+', 'p')
        cfg_path = os.path.join(a.workdir, slug + '.json')
        json.dump(cfg, open(cfg_path, 'w'), indent=1)
        log = os.path.join(a.workdir, slug + '.log')
        t0 = time.time()
        p = subprocess.run([RUNNER, cfg_path, log, a.input_log, '--quiet', '--no-record-input'], capture_output=True, text=True)
        rec = dict(name=name, note=note, wall_s=round(time.time() - t0, 2), exit_code=p.returncode)
        lines = (p.stdout + p.stderr).splitlines()
        rec['errors'] = sum('[ERROR]' in l for l in lines)
        rec['gate_rejections'] = sum('Innovation gate rejected' in l for l in lines)
        if p.returncode != 0:
            rec.update(passed=False, reason=f'exit {p.returncode}: ' + (lines[-1][:160] if lines else ''))
            results[name] = rec
            print(f'{name:45s} FAIL {rec["reason"]}', flush=True)
            continue
        ev = ra.evaluate('pos_ins', log, truth, None, 'legacy')  # python limits as a reference only
        rec.update(epochs=ev['epochs'], checks=ev['checks'])
        if name in limits:
            lim = limits[name]
            ok = True
            for m in ('pos', 'vel', 'tilt'):
                c = ev['checks'][m]
                ok &= max(c['std']) <= lim[m]['std'] and max(c['max']) <= lim[m]['max'] and c['pct_within_1_2_3_sigma'][0] >= lim[m]['p1']
            rec['passed'] = bool(ok and rec['errors'] == 0)
            rec['limits_source'] = a.limits
        else:
            rec['passed'] = bool(rec['errors'] == 0 and ev['checks']['nan_free'] if 'nan_free' in ev['checks'] else rec['errors'] == 0)
            rec['limits_source'] = 'none (run with --derive-limits to record)'
        results[name] = rec
        c = ev['checks']
        print(f"{name:45s} {'PASS' if rec['passed'] else 'FAIL'} epochs={ev['epochs']} pos std {'/'.join(f'{v:.2f}' for v in c['pos']['std'])} "
              f"vel std {'/'.join(f'{v:.3f}' for v in c['vel']['std'])} tilt std {'/'.join(f'{v:.3f}' for v in c['tilt']['std'])} "
              f"pct1 {c['pos']['pct_within_1_2_3_sigma'][0]:.0f}/{c['tilt']['pct_within_1_2_3_sigma'][0]:.0f} gate_rej={rec['gate_rejections']} errors={rec['errors']}", flush=True)
    if a.derive_limits:
        derived = {}
        for name, rec in results.items():
            if not rec.get('checks'):
                continue
            derived[name] = {}
            for m in ('pos', 'vel', 'tilt'):
                c = rec['checks'][m]
                derived[name][m] = dict(std=round(1.1 * max(c['std']), 4), max=round(1.2 * max(c['max']), 3),
                                        p1=max(0, math.floor(c['pct_within_1_2_3_sigma'][0]) - 5),
                                        measured=dict(std=[round(v, 4) for v in c['std']], max=[round(v, 3) for v in c['max']],
                                                      pct=[round(v, 1) for v in c['pct_within_1_2_3_sigma']]))
        json.dump(dict(generated=time.strftime('%Y-%m-%dT%H:%M:%S%z'), rule='std: 1.10 x measured; max: 1.20 x measured; p1: floor(measured) - 5',
                       limits=derived), open(a.limits, 'w'), indent=2)
        print('wrote', a.limits)
    json.dump(dict(generated=time.strftime('%Y-%m-%dT%H:%M:%S%z'), input_log=a.input_log, results=results), open(a.out, 'w'), indent=2)
    with open(a.md, 'w') as f:
        f.write('# Degraded-sensor acceptance matrix\n\n')
        f.write(f'Generated by `tools/run_degraded_matrix.py` on {time.strftime("%Y-%m-%d %H:%M")} from the 43-minute example log. '
                'Every row is `configs/pos_ins.json` (or the named base config) with a `SensorDegradationConfig` preprocessor and a '
                'consistently widened IMU model, run through `cobra_run` in the default (corrected Pinson-Q) mode; errors are against the '
                'ins-d truth channel. Values are per-axis error standard deviations N/E/D or roll/pitch/yaw; "inside 1σ" is the share of '
                'position / tilt errors within one reported sigma (68 % means a consistent filter). PASS compares with '
                '`docs/limits_degraded.json` (recorded from the first run: 1.10 × std, 1.20 × max, coverage − 5).\n\n')
        f.write('| Row | Result | Epochs | Position std [m] | Velocity std [m/s] | Tilt std [deg] | Inside 1σ pos / tilt [%] | Gate rejections | What was done |\n')
        f.write('|---|---|---:|---|---|---|---|---:|---|\n')
        for name, rec in results.items():
            if not rec.get('checks'):
                f.write(f"| {name} | FAIL | — | — | — | — | — | — | {rec.get('reason', '')} |\n")
                continue
            c = rec['checks']
            f.write(f"| {name} | {'PASS' if rec['passed'] else 'FAIL'} | {rec['epochs']} | {' / '.join(f'{v:.2f}' for v in c['pos']['std'])} | "
                    f"{' / '.join(f'{v:.3f}' for v in c['vel']['std'])} | {' / '.join(f'{v:.3f}' for v in c['tilt']['std'])} | "
                    f"{c['pos']['pct_within_1_2_3_sigma'][0]:.0f} / {c['tilt']['pct_within_1_2_3_sigma'][0]:.0f} | {rec['gate_rejections']} | {rec['note']} |\n")
        f.write('\nIMU grades inject white noise and constant biases into the recorded VN-100 data and widen the filter model by the same '
                'factors; they cannot make the data better than the VN-100, only worse. GNSS conditions downsample, add noise, cut outages '
                'or inject one 50 m outlier (with the innovation gate on). The sensor-set rows are the ordinary example configs.\n')
    print('wrote', a.out, 'and', a.md)
    return 0 if all(r.get('passed') for r in results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
