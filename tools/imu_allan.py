#!/usr/bin/env python3
"""Allan deviation and bias wander of the IMU in an LCM log, for tuning an ImuConfig.

    Cobra/.venv/bin/python tools/imu_allan.py LOG [--imu /sensor/vn-100/imu] [--truth /sensor/ins-d/pva] [--json out.json]

Static segments are found from the truth channel's speed (< 0.15 m/s for > 15 s). The longest one gives the
overlapping Allan deviation per axis for the specific force and the angular rate (tau 0.01 .. 30 s); the value
at tau = 1 s is the white-noise density (ImuConfig accel_random_walk_sigma in m/s^2/sqrt(Hz), gyro_random_walk_sigma
in rad/s/sqrt(Hz)), the floor at long tau the bias instability. The mean rate/force of every static segment shows
the bias wander over the recording (the horizontal accelerometer means also carry the vehicle's pitch and roll,
and the gyro means the earth rate's projection, so read those with the heading in mind).
"""
import argparse, json
import numpy as np
from lcm import EventLog
from aspn23_lcm import measurement_IMU as IMU, measurement_position_velocity_attitude as PVA

TAUS = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 30]


def allan(x, dt, taus):
    out = []
    for tau in taus:
        m = int(round(tau / dt))
        if m < 1 or 2 * m >= len(x):
            out.append(float('nan'))
            continue
        n = len(x) // m
        y = x[:n * m].reshape(n, m).mean(axis=1)
        out.append(float(np.sqrt(0.5 * np.mean(np.diff(y) ** 2))))
    return out


def analyze(log, imu_channel='/sensor/vn-100/imu', truth_channel='/sensor/ins-d/pva', static_speed=0.15, min_static_s=15.0):
    """Allan deviation and bias wander of the IMU in `log`; a dict (see the module docstring and `main`).

    Static segments come from the truth channel's speed. Without a truth channel every segment where the gyro
    norm stays small is used instead (less reliable: a slow, straight drive looks static to a gyro)."""
    fi, fp = IMU._get_packed_fingerprint(), PVA._get_packed_fingerprint()
    t, dv, dth, tv, v = [], [], [], [], []
    for ev in EventLog(log):
        if ev.channel == imu_channel and ev.data[:8] == fi:
            m = IMU.decode(ev.data)
            t.append(m.time_of_validity.elapsed_nsec / 1e9); dv.append(m.meas_accel); dth.append(m.meas_gyro)
        elif truth_channel and ev.channel == truth_channel and ev.data[:8] == fp:
            m = PVA.decode(ev.data)
            tv.append(m.time_of_validity.elapsed_nsec / 1e9); v.append((m.v1, m.v2, m.v3))
    if len(t) < 1000:
        raise SystemExit(f'too few IMU samples on {imu_channel}: {len(t)}')
    t, dv, dth = map(np.array, (t, dv, dth))
    dt = float(np.median(np.diff(t)))
    f, w = dv / dt, dth / dt  # integrated increments -> rates
    t0 = t[0]
    segs = []
    if tv:
        tv, v = np.array(tv), np.array(v)
        speed = np.linalg.norm(v[:, :2], axis=1)
        static = speed < static_speed
        i = 0
        while i < len(static):
            if static[i]:
                j = i
                while j < len(static) and static[j]:
                    j += 1
                if tv[j - 1] - tv[i] > min_static_s:
                    segs.append((tv[i] - t0, tv[j - 1] - t0))
                i = j
            else:
                i += 1
        static_source = truth_channel
    else:  # gyro-based: 1 s windows whose rate norm (earth rate removed) stays under 0.01 rad/s
        win = max(1, int(round(1.0 / dt)))
        n = len(w) // win
        rate = np.linalg.norm(w[:n * win].reshape(n, win, 3).mean(axis=1), axis=1)
        quiet = rate < 0.01
        i = 0
        while i < n:
            if quiet[i]:
                j = i
                while j < n and quiet[j]:
                    j += 1
                if j - i > min_static_s:
                    segs.append((i * 1.0, j * 1.0))
                i = j
            else:
                i += 1
        static_source = 'gyro norm'
    if not segs:
        raise SystemExit(f'no static segment longer than {min_static_s} s')
    s0, s1 = max(segs, key=lambda s: s[1] - s[0])
    sel = (t - t0 >= s0 + 2) & (t - t0 <= s1 - 2)
    out = dict(log=log, imu_channel=imu_channel, static_source=static_source, imu_dt=dt, samples=int(len(t)),
               static_segments_s=[[round(x, 1), round(y, 1)] for x, y in segs], longest_static_s=[round(s0, 1), round(s1, 1)], taus=TAUS,
               accel_allan=[allan(f[sel][:, k], dt, TAUS) for k in range(3)], gyro_allan=[allan(w[sel][:, k], dt, TAUS) for k in range(3)])
    k1 = TAUS.index(1)
    out['accel_white_noise_density'] = [out['accel_allan'][k][k1] for k in range(3)]
    out['gyro_white_noise_density'] = [out['gyro_allan'][k][k1] for k in range(3)]
    out['accel_allan_floor'] = [float(np.nanmin(out['accel_allan'][k][7:])) for k in range(3)]
    out['gyro_allan_floor'] = [float(np.nanmin(out['gyro_allan'][k][7:])) for k in range(3)]
    means = []
    for a0, a1 in segs:
        m = (t - t0 >= a0 + 2) & (t - t0 <= a1 - 2)
        if m.sum() > 500:
            means.append([0.5 * (a0 + a1), *w[m].mean(axis=0), *f[m].mean(axis=0)])
    means = np.array(means) if means else np.zeros((0, 7))
    out['segment_means'] = means.tolist()
    out['gyro_mean_wander'] = means[:, 1:4].std(axis=0).tolist() if len(means) > 1 else [0.0] * 3
    out['accel_mean_wander'] = means[:, 4:7].std(axis=0).tolist() if len(means) > 1 else [0.0] * 3
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--imu', default='/sensor/vn-100/imu')
    ap.add_argument('--truth', default='/sensor/ins-d/pva', help="truth channel for the static segments ('' = use the gyro norm)")
    ap.add_argument('--json', default=None)
    a = ap.parse_args()
    out = analyze(a.log, a.imu, a.truth or None)
    s0, s1 = out['longest_static_s']
    print(f"IMU dt {out['imu_dt']:.4f} s, {out['samples']} samples; longest static {s0:.0f}-{s1:.0f} s of {len(out['static_segments_s'])} segments ({out['static_source']})")
    print("white noise density at tau = 1 s  accel m/s^2/sqrt(Hz):", ['%.2e' % x for x in out['accel_white_noise_density']],
          " gyro rad/s/sqrt(Hz):", ['%.2e' % x for x in out['gyro_white_noise_density']])
    print("Allan floor (tau >= 2 s)         accel:", ['%.2e' % x for x in out['accel_allan_floor']], " gyro:", ['%.2e' % x for x in out['gyro_allan_floor']])
    print("bias wander across segments      accel:", ['%.2e' % x for x in out['accel_mean_wander']], " gyro:", ['%.2e' % x for x in out['gyro_mean_wander']])
    for name, arr in (('accel', out['accel_allan']), ('gyro', out['gyro_allan'])):
        for k in range(3):
            print(f"  {name} axis {k}: " + ' '.join(f"{tau:g}s={x:.1e}" for tau, x in zip(TAUS, arr[k])))
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)
        print('wrote', a.json)


if __name__ == '__main__':
    main()
