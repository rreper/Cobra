#!/usr/bin/env python3
"""Compare a pntOS solution log against the truth channel of the input log.

Usage: compare_to_truth.py OUTPUT_LOG [INPUT_LOG] [--solution /solution/pntos/pva] [--truth /sensor/ins-d/pva]

Needs the Cobra venv (lcm + aspn23_lcm). Prints NED position, velocity and attitude RMS errors in the
same form as docs/COBRA_ANALYSIS.md section 2 and writes a JSON summary next to the output log.
"""
import argparse, json, math, os, sys
import numpy as np
from lcm import EventLog
from aspn23_lcm import measurement_position_velocity_attitude as PVA

A = 6378137.0
E2 = 6.69437999014e-3

def radii(lat):
    s = math.sin(lat)
    rn = A * (1 - E2) / (1 - E2 * s * s) ** 1.5
    re = A / math.sqrt(1 - E2 * s * s)
    return rn, re

def quat_to_rpy(q):
    q0, q1, q2, q3 = q
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q1 * q3))))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return np.array([roll, pitch, yaw])

def read_pva(path, channel):
    fp = PVA._get_packed_fingerprint()
    rows = []
    for ev in EventLog(path):
        if ev.channel != channel or ev.data[:8] != fp:
            continue
        m = PVA.decode(ev.data)
        rows.append([m.time_of_validity.elapsed_nsec / 1e9, m.p1, m.p2, m.p3, m.v1, m.v2, m.v3, *m.quaternion])
    return np.array(rows)

def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('output_log')
    ap.add_argument('input_log', nargs='?', default=None)
    ap.add_argument('--solution', default='/solution/pntos/pva')
    ap.add_argument('--truth', default='/sensor/ins-d/pva')
    args = ap.parse_args()
    if args.input_log is None:
        from pntos_python_datasets_lcm import EXAMPLE_LCM_LOG
        args.input_log = EXAMPLE_LCM_LOG
    sol = read_pva(args.output_log, args.solution)
    truth = read_pva(args.input_log, args.truth)
    if len(sol) == 0:
        print(f'no {args.solution} messages in {args.output_log}'); sys.exit(1)
    # Keep solutions inside the truth span; interpolate truth at solution times.
    t = sol[:, 0]
    keep = (t >= truth[0, 0]) & (t <= truth[-1, 0])
    sol = sol[keep]; t = sol[:, 0]
    tt = truth[:, 0]
    interp = lambda col: np.interp(t, tt, truth[:, col])
    lat_t, lon_t, alt_t = interp(1), interp(2), interp(3)
    rn, re = np.vectorize(radii)(lat_t)
    dn = (sol[:, 1] - lat_t) * (rn + alt_t)
    de = (sol[:, 2] - lon_t) * (re + alt_t) * np.cos(lat_t)
    dd = -(sol[:, 3] - alt_t)
    dv = sol[:, 4:7] - np.stack([interp(4), interp(5), interp(6)], axis=1)
    rpy_s = np.array([quat_to_rpy(q) for q in sol[:, 7:11]])
    # truth attitude: interpolate rpy (unwrapped) rather than quaternions
    rpy_t_all = np.array([quat_to_rpy(q) for q in truth[:, 7:11]])
    rpy_t = np.stack([np.interp(t, tt, np.unwrap(rpy_t_all[:, k])) for k in range(3)], axis=1)
    datt = wrap(rpy_s - rpy_t)
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    out = {
        'epochs': int(len(sol)),
        'span_s': float(t[-1] - t[0]),
        'pos_rms_ned_m': [rms(dn), rms(de), rms(dd)],
        'pos_max_ned_m': [float(np.max(np.abs(dn))), float(np.max(np.abs(de))), float(np.max(np.abs(dd)))],
        'vel_rms_ned_mps': [rms(dv[:, 0]), rms(dv[:, 1]), rms(dv[:, 2])],
        'att_rms_rpy_deg': [math.degrees(rms(datt[:, k])) for k in range(3)],
    }
    print(json.dumps(out, indent=2))
    with open(os.path.splitext(args.output_log)[0] + '_errors.json', 'w') as f:
        json.dump(out, f, indent=2)

if __name__ == '__main__':
    main()
