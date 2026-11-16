#!/usr/bin/env python3
"""Alignment in motion on the recorded drive, switched on at many points of it (docs/ALIGNMENT_MATRIX.md).

    Cobra/.venv/bin/python tools/run_alignment_matrix.py [--build build] [--every 40] [--write-configs]

configs/pos_ins.json with the VN-100 tuning for a driving car (preset vn100_driving) and initialization "in_motion_align" is run over the full
example log once per switch-on time: everything before that time is ignored (InMotionAlignmentConfig.start_time),
so the filter comes up as it would in a vehicle switched on there, standing or under way. Against the truth
channel:

  time to align         from the switch-on to the first solution
  heading at hand-over  error and the sigma the filter starts with; their ratio (near 1: the sigma is honest)
  afterwards            heading and horizontal position error 30, 60 and 120 s later, beside the run that is
                        given the heading (ManualHeadingAlignment at the start of the log)

Rows: the receiver's velocity at the time bias the example configs give it (150 ms), at the lags that fit the
recording better, velocity from position differences, and the alignment followed by a filter that also has the
ground-vehicle constraints of run_aiding_matrix.py ("no sideways, no vertical motion") and the zero-velocity /
zero-rate updates while the vehicle stands, on the word of the IMU alone and with the receiver's velocity
confirming each standstill. The rows ending in "offset" estimate the receiver's time offset along
(InMotionAlignmentConfig.time_offset_prior_sigma), at several configured time biases.

--write-configs also writes configs/pos_ins_align_in_motion.json (pos_ins without the heading from the
configuration file) and configs/pos_ins_ground_vehicle.json (the same with the constraints and the zero-velocity /
zero-rate updates at standstill, confirmed by the receiver's velocity), no start time; both estimate the receiver's
time offset.
"""
import argparse, copy, json, math, os, re, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import run_acceptance as ra  # noqa: E402
from run_aiding_matrix import GNSS_P, GNSS_V, IMU, TRUTH, SOL, base, orch, listen, constraints, zero_motion  # noqa: E402

LEVER = [-0.5, 0.38, -0.05]  # antenna relative to the IMU, as in the example configs
AFTER = [10.0, 30.0, 60.0, 120.0]
SUMMARY = re.compile(r'Aligned in motion after ([\d.]+) s \(window of (\d+) epochs, ([\d.]+) s, velocity from ([a-z ]+)\): heading (-?[\d.]+) deg, '
                     r'sigma ([\d.]+) deg; gyro bias \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\) mrad/s(?:, ([\d.]+) s of standstill used)?')


OFFSET = re.compile(r'(?:velocity is|fixes are) ([\d.]+) s (older|newer) than (?:its|their) time stamps \(sigma ([\d.]+) s\)')


def aligned(cfg, source='velocity', velocity_bias_ms=150, heading_sigma_deg=3.0, start_time=None, lever=LEVER, **extra):
    """`cfg` with the alignment in motion in place of the heading from the configuration file."""
    c = copy.deepcopy(cfg)
    c['app']['initialization'] = 'in_motion_align'
    c['app']['logging_level'] = 'INFO'
    o = orch(c)
    a = dict(type='InMotionAlignmentConfig', group='config/default/alignment', imu_model=o['pinson_sb_config']['imu_model'],
             heading_sigma=math.radians(heading_sigma_deg), lever_arm=lever, velocity_source=source)
    a.update(extra)
    if start_time is not None:
        a['start_time'] = start_time
    o['alignment_config'] = a
    o['alignment_channels'] = [GNSS_P, IMU] + ([GNSS_V] if source != 'position' else [])
    if source != 'position':
        listen(c, GNSS_V)
        bias = next((p for p in o['preprocessor_configs'] if p['type'] == 'TimeBiasConfig' and GNSS_V in p['channels']), None)
        if bias is None:  # a confirmed zero-motion generator has put one in front of itself already
            bias = dict(type='TimeBiasConfig', group='config/time_bias_velocity', channels=[GNSS_V])
            o['preprocessor_configs'].append(bias)
        bias['time_bias'] = int(velocity_bias_ms * 1e6)
    return c


plain = lambda cfg: cfg  # noqa: E731
ground_vehicle = lambda cfg: zero_motion(constraints(cfg))  # noqa: E731
ground_vehicle_confirmed = lambda cfg: zero_motion(constraints(cfg), confirmed=True)  # noqa: E731
OFFSET_PRIOR = 0.3  # s; sigma of the prior of the receiver's time offset where it is estimated (DESIGN.md 7.29)
ROWS = [('velocity_150ms', dict(source='velocity', velocity_bias_ms=150), "the receiver's velocity, 150 ms time bias as the example configs give it", plain),
        ('velocity_250ms', dict(source='velocity', velocity_bias_ms=250), "the receiver's velocity, 250 ms", plain),
        ('velocity_350ms', dict(source='velocity', velocity_bias_ms=350), "the receiver's velocity, 350 ms", plain),
        ('velocity_0ms', dict(source='velocity', velocity_bias_ms=0), "the receiver's velocity as it is stamped, no time bias configured", plain),
        ('velocity_250ms_2deg', dict(source='velocity', velocity_bias_ms=250, heading_sigma_deg=2.0), '250 ms, declared at a heading sigma of 2 deg instead of 3', plain),
        ('position', dict(source='position'), 'differences of the position fixes, no velocity channel', plain),
        ('velocity_250ms_no_lever', dict(source='velocity', velocity_bias_ms=250, lever=[0.0, 0.0, 0.0]), '250 ms, antenna lever arm left out', plain),
        ('velocity_250ms_constraints', dict(source='velocity', velocity_bias_ms=250),
         '250 ms; afterwards the filter also has the ground-vehicle constraints (same alignment, the difference is in the second table)', constraints),
        ('position_constraints', dict(source='position'), 'position differences; afterwards with the constraints: a vehicle with a position-only receiver', constraints),
        ('velocity_250ms_zero_motion', dict(source='velocity', velocity_bias_ms=250),
         '250 ms; afterwards with zero-velocity and zero-rate updates while the vehicle stands, no constraints', zero_motion),
        ('velocity_250ms_constraints_zero_motion', dict(source='velocity', velocity_bias_ms=250),
         '250 ms; afterwards with the constraints and the zero-velocity / zero-rate updates on the word of the IMU alone', ground_vehicle),
        ('velocity_250ms_constraints_zero_motion_confirmed', dict(source='velocity', velocity_bias_ms=250),
         "the same with the receiver's velocity confirming each standstill", ground_vehicle_confirmed),
        ('velocity_0ms_offset', dict(source='velocity', velocity_bias_ms=0, time_offset_prior_sigma=OFFSET_PRIOR),
         "the receiver's velocity with no time bias configured at all, its time offset estimated along (prior sigma 0.3 s)", plain),
        ('velocity_150ms_offset', dict(source='velocity', velocity_bias_ms=150, time_offset_prior_sigma=OFFSET_PRIOR), '150 ms and the offset estimated', plain),
        ('velocity_250ms_offset', dict(source='velocity', velocity_bias_ms=250, time_offset_prior_sigma=OFFSET_PRIOR),
         '250 ms and the offset estimated: configs/pos_ins_align_in_motion.json', plain),
        ('velocity_350ms_offset', dict(source='velocity', velocity_bias_ms=350, time_offset_prior_sigma=OFFSET_PRIOR), '350 ms and the offset estimated', plain),
        ('position_offset', dict(source='position', time_offset_prior_sigma=OFFSET_PRIOR),
         'differences of the position fixes (150 ms time bias as configured), the offset of the fixes estimated', plain),
        ('velocity_250ms_offset_ground_vehicle', dict(source='velocity', velocity_bias_ms=250, time_offset_prior_sigma=OFFSET_PRIOR),
         '250 ms and the offset estimated; afterwards the constraints and the confirmed zero-motion updates: configs/pos_ins_ground_vehicle.json',
         ground_vehicle_confirmed)]


def dcm(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def errors(sol, truth):
    """Per solution epoch: heading error (deg), tilt error (deg), horizontal position error (m)."""
    t, tt = sol[:, 0], truth[:, 0]
    k = np.clip(np.searchsorted(tt, t), 0, len(tt) - 1)
    lat, alt = truth[k, 1], truth[k, 3]
    rn, re = np.vectorize(ra.radii)(lat)
    h = np.hypot((sol[:, 1] - lat) * (rn + alt), (sol[:, 2] - truth[k, 2]) * (re + alt) * np.cos(lat))
    head, tilt = np.empty(len(t)), np.empty(len(t))
    for i in range(len(t)):
        e = dcm(sol[i, 7:11]) @ dcm(truth[k[i], 7:11]).T
        phi = np.degrees(np.array([e[2, 1] - e[1, 2], e[0, 2] - e[2, 0], e[1, 0] - e[0, 1]]) / 2)
        head[i], tilt[i] = phi[2], math.hypot(phi[0], phi[1])
    return head, tilt, h


def run(runner, log, truth, cfg, name, workdir, t_on=None):
    path, out = os.path.join(workdir, name + '.json'), os.path.join(workdir, name + '.log')
    json.dump(cfg, open(path, 'w'))
    p = subprocess.run([runner, path, out, log, '--no-record-input'], capture_output=True, text=True)
    if p.returncode != 0:
        return dict(exit=p.returncode, error=(p.stdout + p.stderr)[-300:])
    sol, sig = ra.read_pva(out, SOL)
    os.remove(out)
    r = dict(exit=0, aligned=bool(len(sol)))
    if not len(sol):
        return r
    head, tilt, h = errors(sol, truth)
    t = sol[:, 0]
    r.update(t=t, head=head, tilt=tilt, h=h)
    m = SUMMARY.search(p.stdout + p.stderr)
    if m:
        r.update(window_epochs=int(m.group(2)), window_s=float(m.group(3)), source=m.group(4), gyro_bias_mrad_s=[float(m.group(i)) for i in (7, 8, 9)],
                 standstill_s=float(m.group(10) or 0.0))
    o = OFFSET.search(p.stdout + p.stderr)
    if o:  # with time_offset_sigma: how much older than its stamps the alignment found the velocity to be
        r.update(time_offset=(1.0 if o.group(2) == 'older' else -1.0) * float(o.group(1)), time_offset_sigma=float(o.group(3)))
    if t_on is not None:
        r.update(time_to_align=float(t[0] - t_on), heading_error=float(head[0]), heading_sigma=float(np.degrees(sig[0, 8])), tilt_error=float(tilt[0]),
                 position_error=float(h[0]))
    return r


def at(r, when):
    k = np.searchsorted(r['t'], when)
    return None if k >= len(r['t']) or r['t'][k] - when > 2.0 else k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', default='build')
    ap.add_argument('--input-log', default=None)
    ap.add_argument('--every', type=float, default=40.0, help='s between switch-on times')
    ap.add_argument('--rows', default=None, help='comma-separated subset of rows')
    ap.add_argument('--workdir', default=None)
    ap.add_argument('--out', default='docs/alignment_matrix.json')
    ap.add_argument('--md', default='docs/ALIGNMENT_MATRIX.md')
    ap.add_argument('--write-configs', action='store_true')
    ap.add_argument('--jobs', type=int, default=min(12, os.cpu_count() or 2))
    a = ap.parse_args()
    if a.write_configs:
        for name, then in (('pos_ins_align_in_motion', plain), ('pos_ins_ground_vehicle', ground_vehicle_confirmed)):
            cfg = aligned(then(base()), time_offset_prior_sigma=OFFSET_PRIOR, **ROWS[1][1])
            cfg['app']['name'] = name
            json.dump(cfg, open(os.path.join(ROOT, 'configs', name + '.json'), 'w'), indent=2)
            print('wrote configs/%s.json' % name)
    if a.input_log is None:
        from pntos_python_datasets_lcm import EXAMPLE_LCM_LOG
        a.input_log = EXAMPLE_LCM_LOG
    workdir = a.workdir or os.path.join(a.build, 'alignment_matrix')
    os.makedirs(workdir, exist_ok=True)
    runner = os.path.join(a.build, 'apps', 'cobra_run')
    truth, _ = ra.read_pva(a.input_log, TRUTH)
    t0, t_end = truth[0, 0], truth[-1, 0]
    starts = [float(s) for s in np.arange(0.0, t_end - t0 - 80.0, a.every)]
    rows = [r for r in ROWS if a.rows is None or r[0] in a.rows.split(',')]
    began = time.time()
    reference = run(runner, a.input_log, truth, base(), 'reference', workdir)
    if reference.get('exit'):
        print('reference run failed:', reference.get('error'))
        return 1
    jobs = [(name, s, aligned(then(base()), start_time=t0 + s, **kw)) for name, kw, _, then in rows for s in starts]
    with ThreadPoolExecutor(a.jobs) as ex:
        results = list(ex.map(lambda j: run(runner, a.input_log, truth, j[2], '%s_%04d' % (j[0], j[1]), workdir, t0 + j[1]), jobs))
    by = {(j[0], j[1]): r for j, r in zip(jobs, results)}

    table, detail = [], []
    for name, _, note, _ in rows:
        rs = [(s, by[(name, s)]) for s in starts]
        failed = [s for s, r in rs if r.get('exit')]
        ok = [(s, r) for s, r in rs if not r.get('exit') and r.get('aligned')]
        row = dict(row=name, note=note, starts=len(rs), failed=len(failed), aligned=len(ok))
        if ok:
            he = np.array([r['heading_error'] for _, r in ok]); hs = np.array([r['heading_sigma'] for _, r in ok])
            tta = np.array([r['time_to_align'] for _, r in ok]); z = he / hs
            moving = np.array([r.get('standstill_s', 0.0) < 3.0 for _, r in ok])
            row.update(time_median=float(np.median(tta)), time_p90=float(np.percentile(tta, 90)), time_max=float(tta.max()),
                       heading_rms=float(np.sqrt(np.mean(he ** 2))), heading_max=float(np.abs(he).max()), sigma_mean=float(hs.mean()),
                       ratio_rms=float(np.sqrt(np.mean(z ** 2))), ratio_max=float(np.abs(z).max()),
                       tilt_rms=float(np.sqrt(np.mean([r['tilt_error'] ** 2 for _, r in ok]))), tilt_max=float(max(r['tilt_error'] for _, r in ok)),
                       position_rms=float(np.sqrt(np.mean([r['position_error'] ** 2 for _, r in ok]))),
                       under_way=int(moving.sum()), heading_rms_under_way=float(np.sqrt(np.mean(he[moving] ** 2))) if moving.any() else None,
                       heading_rms_from_standstill=float(np.sqrt(np.mean(he[~moving] ** 2))) if (~moving).any() else None)
            off = np.array([r['time_offset'] for _, r in ok if r.get('time_offset') is not None])
            if len(off):  # how much older than its stamps (after the configured time bias) the alignment found the velocity
                row.update(offset_n=int(len(off)), offset_mean=float(off.mean()), offset_std=float(off.std()), offset_min=float(off.min()),
                           offset_max=float(off.max()),
                           offset_sigma_mean=float(np.mean([r['time_offset_sigma'] for _, r in ok if r.get('time_offset') is not None])))
            for dt in AFTER:
                hh, pp, rh, rp = [], [], [], []
                for _, r in ok:
                    k, kr = at(r, r['t'][0] + dt), at(reference, r['t'][0] + dt)
                    if k is None or kr is None:
                        continue
                    hh.append(r['head'][k]); pp.append(r['h'][k]); rh.append(reference['head'][kr]); rp.append(reference['h'][kr])
                if hh:
                    row['after_%d' % dt] = dict(n=len(hh), heading_rms=float(np.sqrt(np.mean(np.square(hh)))), heading_max=float(np.abs(hh).max()),
                                                position_rms=float(np.sqrt(np.mean(np.square(pp)))), position_max=float(np.max(pp)),
                                                reference_heading_rms=float(np.sqrt(np.mean(np.square(rh)))),
                                                reference_position_rms=float(np.sqrt(np.mean(np.square(rp)))))
        table.append(row)
        for s, r in rs:
            d = dict(row=name, switch_on_s=s, exit=r.get('exit', 0), aligned=bool(r.get('aligned')))
            for key in ('time_to_align', 'heading_error', 'heading_sigma', 'tilt_error', 'position_error', 'window_epochs', 'window_s', 'standstill_s',
                        'gyro_bias_mrad_s', 'time_offset', 'time_offset_sigma', 'error'):
                if key in r:
                    d[key] = r[key]
            detail.append(d)

    json.dump(dict(generated=time.strftime('%Y-%m-%dT%H:%M:%S%z'), switch_on_every_s=a.every, rows=table, runs=detail), open(a.out, 'w'), indent=1)
    with open(a.md, 'w') as f:
        f.write('# Alignment matrix: the filter switched on anywhere in the drive\n\n')
        f.write('Generated %s by `tools/run_alignment_matrix.py` in %.0f s. `configs/pos_ins.json` with the `vn100_driving` tuning and initialization '
                '`in_motion_align`, run over the full example log once per switch-on time (every %g s, %d of them): everything before that time is '
                'ignored, so the filter comes up as in a vehicle switched on there, standing or under way. Against the truth channel.\n\n'
                % (time.strftime('%Y-%m-%d %H:%M'), time.time() - began, a.every, len(starts)))
        f.write('## At the hand-over\n\n')
        f.write('| velocity from | aligned | time to align: median, 90 %, longest (s) | heading error: RMS (worst), deg | sigma it starts with, deg | '
                'error / sigma: RMS (worst) | tilt error: RMS (worst), deg | switched on under way: n, heading RMS | from a standstill: heading RMS |\n')
        f.write('|---|---|---|---|---|---|---|---|---|\n')
        for row in table:
            if not row['aligned']:
                f.write('| %s | 0 / %d | | | | | | | |\n' % (row['row'], row['starts']))
                continue
            f.write('| %s | %d / %d | %.0f, %.0f, %.0f | %.2f (%.2f) | %.2f | %.2f (%.2f) | %.2f (%.2f) | %d, %s | %s |\n' % (
                row['row'], row['aligned'], row['starts'], row['time_median'], row['time_p90'], row['time_max'], row['heading_rms'], row['heading_max'],
                row['sigma_mean'], row['ratio_rms'], row['ratio_max'], row['tilt_rms'], row['tilt_max'], row['under_way'],
                '%.2f' % row['heading_rms_under_way'] if row['heading_rms_under_way'] is not None else '-',
                '%.2f' % row['heading_rms_from_standstill'] if row['heading_rms_from_standstill'] is not None else '-'))
        f.write('\n' + '\n'.join('- `%s`: %s' % (name, note) for name, _, note, _ in rows) + '\n')
        f.write('\nThe longest times are switch-on times at which the car stands: nothing shows a heading until it moves.\n')
        with_offset = [row for row in table if row.get('offset_n')]
        if with_offset:
            f.write('\n## The receiver\'s time offset\n\nWhere it is estimated (`time_offset_prior_sigma`): how much older than its time stamps the alignment found '
                    'the velocity to be, after the time bias the configuration applies, over the switch-on times. The last column is the two added: the same '
                    'receiver has to give the same age whatever was configured.\n\n')
            f.write('| velocity from | configured time bias, s | offset found: mean (std), s | least, largest | sigma it reports, mean | age of the velocity, s |\n')
            f.write('|---|---|---|---|---|---|\n')
            for row in with_offset:
                kw = next(k for name, k, _, _ in rows if name == row['row'])
                bias = 0.15 if kw.get('source') == 'position' else kw.get('velocity_bias_ms', 150) / 1000.0
                f.write('| %s | %.2f | %+.3f (%.3f) | %+.3f, %+.3f | %.3f | %.3f |\n' % (row['row'], bias, row['offset_mean'], row['offset_std'], row['offset_min'],
                                                                                     row['offset_max'], row['offset_sigma_mean'], bias + row['offset_mean']))
        f.write('\n## Afterwards\n\nHeading error (deg) and horizontal position error (m), RMS over the switch-on times, a given time after the hand-over; '
                '"given" is the standard run, whose heading comes from the configuration file at the start of the log, at the same moments.\n\n')
        f.write('| velocity from | ' + ' | '.join('+%g s: heading (worst), given / position (worst), given' % dt for dt in AFTER) + ' |\n')
        f.write('|---|' + '---|' * len(AFTER) + '\n')
        for row in table:
            cells = []
            for dt in AFTER:
                c = row.get('after_%d' % dt)
                cells.append('-' if not c else '%.2f (%.2f), %.2f / %.2f (%.2f), %.2f' % (c['heading_rms'], c['heading_max'], c['reference_heading_rms'],
                                                                                          c['position_rms'], c['position_max'], c['reference_position_rms']))
            f.write('| %s | %s |\n' % (row['row'], ' | '.join(cells)))
    for row in table:
        if not row['aligned']:
            print('%-26s aligned 0/%d (failed %d)' % (row['row'], row['starts'], row['failed']))
            continue
        c = row.get('after_60', {})
        print('%-26s aligned %d/%d  t %.0f/%.0f/%.0f s  heading %.2f (%.2f) sigma %.2f ratio %.2f (%.2f)  tilt %.2f  under way %s standstill %s | +60 s: heading %.2f (given %.2f) pos %.2f (given %.2f)' % (
            row['row'], row['aligned'], row['starts'], row['time_median'], row['time_p90'], row['time_max'], row['heading_rms'], row['heading_max'],
            row['sigma_mean'], row['ratio_rms'], row['ratio_max'], row['tilt_rms'],
            '%.2f' % row['heading_rms_under_way'] if row['heading_rms_under_way'] is not None else '-',
            '%.2f' % row['heading_rms_from_standstill'] if row['heading_rms_from_standstill'] is not None else '-',
            c.get('heading_rms', float('nan')), c.get('reference_heading_rms', float('nan')), c.get('position_rms', float('nan')),
            c.get('reference_position_rms', float('nan'))))
    print('wrote', a.out, 'and', a.md, 'in %.0f s' % (time.time() - began))
    return 1 if any(row['failed'] for row in table) else 0


if __name__ == '__main__':
    sys.exit(main())
