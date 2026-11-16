#!/usr/bin/env python3
"""What the filter does through a complete GNSS outage, by aiding set (docs/AIDING_MATRIX.md).

    Cobra/.venv/bin/python tools/run_aiding_matrix.py [--build build] [--outages 60,120] [--write-configs]

configs/pos_ins.json with the measured VN-100 tuning is run over the full example log with every GNSS channel
withheld for the given duration at six points of the drive; the horizontal error at the end of each outage is
compared with the truth channel, for:

  imu_only                 nothing but the IMU
  constraints              "no sideways, no vertical motion" every second from the IMU clock (pinson_odometry)
  wheel                    a wheel speed with a 2 % scale error, emulated from the log's body velocity, and the
                           constraints with every speed message; scale and misalignment estimated
  wheel_uncalibrated       the same through pinson_body_velocity, which has no calibration states
  velocity3                the log's three-axis body velocity; scale and misalignment estimated
  full                     wheel plus a second, independent receiver (emulated from truth, 3 m) and the barometer;
                           the outage takes the first receiver only

--write-configs also writes configs/pos_ins_odometry.json (wheel) and configs/pos_ins_full.json (full), without
the outages.
"""
import argparse, copy, json, os, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import run_acceptance as ra  # noqa: E402

GNSS_P, GNSS_V, IMU = '/sensor/ublox-ZED-F9T/position', '/sensor/ublox-ZED-F9T/velocity', '/sensor/vn-100/imu'
TRUTH, SOL, VEL, NHC = '/sensor/ins-d/pva', '/solution/pntos/pva', '/sensor/simulated/velocity', '/generated/constraints'
BARO, SECOND = '/sensor/bmp388/baro_pressure', '/sensor/second-receiver/position'
OUTAGE_STARTS = [300.0, 700.0, 1100.0, 1500.0, 1900.0, 2300.0]  # s after the first truth message; the vehicle moves at all of them
AUX = ['INERTIAL_PVA', 'INERTIAL_FORCES_AND_RATES']
# Weights found on this log (docs/DESIGN.md 7.23): the constraints hold to 0.2 m/s once the misalignment is
# estimated; a wheel speed is not better than 0.1 m/s; the calibration states wander with a 10 minute time constant.
NHC_SIGMA, SPEED_COVARIANCE_SCALE, CAL_SIGMA, CAL_TAU = 0.2, 4.0, [0.03, 0.02, 0.02], 600.0


def orch(cfg):
    return next(e for e in cfg['configs'] if e['type'] == 'StandardOrchestrationConfig')


def base(preset='vn100_measured'):
    cfg = json.load(open(os.path.join(ROOT, 'configs', 'pos_ins.json')))
    cfg['app']['logging_level'] = 'WARN'
    o = orch(cfg)
    for h in (o['pinson_sb_config'], o['alignment_config']):
        h['imu_model'] = dict(type='ImuConfig', group='config/inertial_state', preset=preset)
    return cfg


def advanced(cfg):
    cfg['app']['preprocessors'] = ['standard', 'advanced']
    return cfg


def listen(cfg, *channels):
    for e in cfg['configs']:
        if e['type'] == 'LcmLogTransportConfig':
            for ch in channels:
                if ch not in e['channels_to_process']:
                    e['channels_to_process'].append(ch)


def calibration_block(o):
    if not any(b.get('label') == 'odometer_calibration' for b in o['additional_sb_configs']):
        o['additional_sb_configs'].append(dict(
            type='FogmStateBlockConfig', group='config/odometer_calibration_block', label='odometer_calibration',
            estimate_with_covariance=dict(type='EWC_GENERIC', estimate=[0.0, 0.0, 0.0], covariance_diag=[s * s for s in CAL_SIGMA]),
            fogm_model=dict(group='config/odometer_calibration', sigma=CAL_SIGMA, tau=[CAL_TAU] * 3)))


def velocity_processor(o, label, channel, calibrated=True):
    if calibrated:
        calibration_block(o)
    o['mp_configs'].append(dict(type='PinsonOdometryMPConfig' if calibrated else 'PinsonBodyVelocityMPConfig', group='config/%s_processor' % label,
                                label=label, channel=channel, lever_arm=[0.0, 0.0, 0.0], orientation=[1.0, 0.0, 0.0, 0.0],
                                state_block_labels=['pinson15', 'odometer_calibration'] if calibrated else ['pinson15'], aux_channels=AUX))


def constraints(cfg):
    """The constraints alone, clocked by the IMU. Its messages are not buffered by the controller, the fixes are:
    the reorder window has to cover the 2 s buffer and the receiver's time bias."""
    c = advanced(copy.deepcopy(cfg))
    o = orch(c)
    o['measurement_reorder_s'] = 2.5
    o['preprocessor_configs'].append(dict(type='ZeroVelocity2dGeneratorConfig', group='config/constraints', channels=[IMU], trigger_dt_sec=1.0,
                                          lateral_vel_sigma=NHC_SIGMA, vertical_vel_sigma=NHC_SIGMA, output_channel=NHC))
    velocity_processor(o, 'constraints', NHC)
    return c


def wheel(cfg, calibrated=True):
    """A wheel speed (the log's body velocity cut down to its forward axis, 2 % too long) and the constraints with it."""
    c = advanced(copy.deepcopy(cfg))
    listen(c, VEL)
    o = orch(c)
    o['measurement_reorder_s'] = 2.5  # the controller's 2 s buffer and the receiver's time bias
    o['preprocessor_configs'].append(dict(type='SensorDegradationConfig', group='config/wheel_emulation', channels=[VEL], seed=3, velocity_scale=1.02,
                                          velocity_forward_only=True, velocity_covariance_scale=SPEED_COVARIANCE_SCALE))
    o['preprocessor_configs'].append(dict(type='ZeroVelocity2dGeneratorConfig', group='config/constraints', channels=[VEL], trigger_dt_sec=0.5,
                                          lateral_vel_sigma=NHC_SIGMA, vertical_vel_sigma=NHC_SIGMA, output_channel=NHC))
    velocity_processor(o, 'wheel', VEL, calibrated)
    velocity_processor(o, 'constraints', NHC, calibrated)
    return c


def velocity3(cfg):
    c = copy.deepcopy(cfg)
    listen(c, VEL)
    o = orch(c)
    o['measurement_reorder_s'] = 2.5
    velocity_processor(o, 'velocity', VEL)
    return c


def full(cfg):
    """wheel() plus a second receiver and the barometer, every position processor gated."""
    c = wheel(cfg)
    listen(c, TRUTH, BARO)
    o = orch(c)
    o['preprocessor_configs'].append(dict(type='SensorDegradationConfig', group='config/second_receiver_emulation', channels=[TRUTH], seed=17,
                                          derived_position_channel=SECOND, derived_position_sigma_ned=[3.0, 3.0, 5.0], derived_position_rate_hz=1.0))
    o['preprocessor_configs'].append(dict(type='BarometerToAltitudeConfig', group='config/baro', channels=[BARO], alt_sigma=30.0))
    o['additional_sb_configs'].append(dict(type='FogmStateBlockConfig', group='config/second_fogm_block', label='second_sensor_error',
                                           estimate_with_covariance=dict(type='EWC_GENERIC', estimate=[0.0, 0.0, 0.0], covariance_diag=[9.0, 9.0, 25.0]),
                                           fogm_model=dict(group='config/second_sensor_error', sigma=[2.0, 2.0, 3.0], tau=[300.0, 300.0, 200.0])))
    o['additional_sb_configs'].append(dict(type='FogmStateBlockConfig', group='config/alt_fogm_block', label='alt_fogm',
                                           estimate_with_covariance=dict(type='EWC_GENERIC', estimate=[0.0], covariance_diag=[900.0]),
                                           fogm_model=dict(group='config/alt_fogm', sigma=[30.0], tau=[3600.0])))
    o['mp_configs'].append(dict(type='PinsonWithNedFogmPositionMPConfig', group='config/second_processor', label='second', channel=SECOND,
                                state_block_labels=['pinson15', 'second_sensor_error'], lever_arm=[0.0, 0.0, 0.0], aux_channels=['INERTIAL_PVA'],
                                innovation_gate_probability=0.999))
    o['mp_configs'].append(dict(type='AltitudeMPConfig', group='config/alt_processor', label='alt', channel='/sensor/bmp388/altitude',
                                state_block_labels=['pinson15', 'alt_fogm'], lever_arm=[0.0, 0.0, 1.0], aux_channels=['INERTIAL_PVA']))
    for m in o['mp_configs']:
        if m['label'] == 'pos':
            m['innovation_gate_probability'] = 0.999
    return c


ROWS = [('imu_only', lambda b: b, 'the IMU alone'),
        ('constraints', constraints, 'no sideways / vertical motion, every second from the IMU clock; misalignment estimated'),
        ('wheel', wheel, 'wheel speed with a 2 % scale error and the constraints; scale and misalignment estimated'),
        ('wheel_uncalibrated', lambda b: wheel(b, calibrated=False), 'the same through pinson_body_velocity: no calibration states'),
        ('velocity3', velocity3, 'three-axis body velocity (0.05 m/s); scale and misalignment estimated'),
        ('full', full, 'wheel, a second receiver (3 m) and the barometer; the outage takes the first receiver only')]


def with_outages(cfg, duration):
    c = copy.deepcopy(cfg)
    o = orch(c)
    channels = [GNSS_P, GNSS_V]
    for i, t0 in enumerate(OUTAGE_STARTS if duration else []):
        o['preprocessor_configs'].append(dict(type='OutageConfig', group='config/outage%d' % i, channels=channels, start_time=t0, end_time=t0 + duration))
    return c


def run(runner, log, truth, cfg, name, duration, workdir):
    path, out = os.path.join(workdir, name + '.json'), os.path.join(workdir, name + '.log')
    json.dump(cfg, open(path, 'w'))
    p = subprocess.run([runner, path, out, log, '--quiet', '--no-record-input'], capture_output=True, text=True)
    if p.returncode != 0:
        return dict(exit=p.returncode, error=(p.stdout + p.stderr)[-300:])
    sol, sig = ra.read_pva(out, SOL)
    os.remove(out)
    t, tt = sol[:, 0], truth[:, 0]
    itp = lambda col: np.interp(t, tt, truth[:, col])
    lat, alt = itp(1), itp(3)
    rn, re = np.vectorize(ra.radii)(lat)
    h = np.hypot((sol[:, 1] - lat) * (rn + alt), (sol[:, 2] - itp(2)) * (re + alt) * np.cos(lat))
    v = np.abs(sol[:, 3] - alt)
    hs = np.sqrt(sig[:, 0] ** 2 + sig[:, 1] ** 2)
    clean = t > tt[0] + 60
    ends = []
    for o in (OUTAGE_STARTS if duration else []):
        k = min(np.searchsorted(t, tt[0] + o + duration - 0.5), len(t) - 1)
        ends.append((float(h[k]), float(hs[k]), float(v[k])))
        clean &= ~((t >= tt[0] + o) & (t <= tt[0] + o + duration + 30))
    r = dict(exit=0, epochs=int(len(t)), h_rms=float(np.sqrt(np.mean(h[clean] ** 2))), v_rms=float(np.sqrt(np.mean(v[clean] ** 2))))
    if ends:
        r.update(end_h=[round(e[0], 1) for e in ends], end_h_mean=float(np.mean([e[0] for e in ends])), end_h_max=float(max(e[0] for e in ends)),
                 end_sigma_mean=float(np.mean([e[1] for e in ends])), end_ratio_rms=float(np.sqrt(np.mean([(e[0] / e[1]) ** 2 for e in ends]))),
                 end_v_mean=float(np.mean([e[2] for e in ends])))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', default='build')
    ap.add_argument('--input-log', default=None)
    ap.add_argument('--outages', default='60,120')
    ap.add_argument('--workdir', default=None)
    ap.add_argument('--out', default='docs/aiding_matrix.json')
    ap.add_argument('--md', default='docs/AIDING_MATRIX.md')
    ap.add_argument('--write-configs', action='store_true')
    ap.add_argument('--jobs', type=int, default=min(12, os.cpu_count() or 2))
    a = ap.parse_args()
    if a.write_configs:
        for name, make in (('pos_ins_odometry', wheel), ('pos_ins_full', full)):
            cfg = make(base())
            cfg['app']['name'] = name
            cfg['app']['logging_level'] = 'INFO'
            json.dump(cfg, open(os.path.join(ROOT, 'configs', name + '.json'), 'w'), indent=2)
            print('wrote configs/%s.json' % name)
    if a.input_log is None:
        from pntos_python_datasets_lcm import EXAMPLE_LCM_LOG
        a.input_log = EXAMPLE_LCM_LOG
    workdir = a.workdir or os.path.join(a.build, 'aiding_matrix')
    os.makedirs(workdir, exist_ok=True)
    runner = os.path.join(a.build, 'apps', 'cobra_run')
    truth, _ = ra.read_pva(a.input_log, TRUTH)
    durations = [0.0] + [float(x) for x in a.outages.split(',') if x]
    jobs = [(name, d, with_outages(make(base()), d)) for name, make, _ in ROWS for d in durations]
    t0 = time.time()
    with ThreadPoolExecutor(a.jobs) as ex:
        results = list(ex.map(lambda j: run(runner, a.input_log, truth, j[2], '%s_%d' % (j[0], j[1]), j[1], workdir), jobs))
    by = {(j[0], j[1]): r for j, r in zip(jobs, results)}
    failed = [k for k, r in by.items() if r.get('exit')]
    json.dump(dict(generated=time.strftime('%Y-%m-%dT%H:%M:%S%z'), outage_starts_s=OUTAGE_STARTS, rows=[dict(row=k[0], outage_s=k[1], **r) for k, r in by.items()]),
              open(a.out, 'w'), indent=1)
    with open(a.md, 'w') as f:
        f.write('# Aiding matrix: a complete GNSS outage, by what else the filter has\n\n')
        f.write('Generated %s by `tools/run_aiding_matrix.py` in %.0f s. `configs/pos_ins.json` with the `vn100_measured` tuning on the full example log; '
                'every channel of the receiver withheld at %s s of the drive. Horizontal error against the truth channel at the end of each outage, in metres: '
                'mean and worst of the six, the filter\'s own sigma, and their ratio (near 1: the covariance is honest). "No outage": RMS over the drive, '
                'horizontal / vertical.\n\n' % (time.strftime('%Y-%m-%d %H:%M'), time.time() - t0, ', '.join('%g' % o for o in OUTAGE_STARTS)))
        f.write('| aiding | no outage, h / v RMS | ' + ' | '.join('%g s outage: mean (worst), sigma, ratio' % d for d in durations[1:]) + ' | what it is |\n')
        f.write('|---|---|' + '---|' * len(durations[1:]) + '---|\n')
        for name, _, note in ROWS:
            c = by[(name, 0.0)]
            cells = []
            for d in durations[1:]:
                r = by[(name, d)]
                cells.append('failed' if r.get('exit') else '%.1f (%.1f), %.1f, %.2f' % (r['end_h_mean'], r['end_h_max'], r['end_sigma_mean'], r['end_ratio_rms']))
            f.write('| %s | %s | %s | %s |\n' % (name, 'failed' if c.get('exit') else '%.2f / %.2f' % (c['h_rms'], c['v_rms']), ' | '.join(cells), note))
        f.write('\nThe wheel speed and the second receiver do not exist in the recording: the first is the log\'s simulated body velocity (truth plus '
                '0.05 m/s) cut down to its forward axis and scaled by 1.02, the second is the truth position with 3 m of noise once a second. '
                'The constraints assume a vehicle that neither slides sideways nor leaves the ground.\n')
    for name, _, _ in ROWS:
        c = by[(name, 0.0)]
        print('%-20s %s' % (name, 'FAILED ' + c.get('error', '')[-120:] if c.get('exit') else 'h/v %.2f/%.2f  ' % (c['h_rms'], c['v_rms']) + '  '.join(
            '%gs: %.1f (%.1f) r%.2f' % (d, by[(name, d)]['end_h_mean'], by[(name, d)]['end_h_max'], by[(name, d)]['end_ratio_rms']) for d in durations[1:] if not by[(name, d)].get('exit'))))
    print('wrote', a.out, 'and', a.md)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
