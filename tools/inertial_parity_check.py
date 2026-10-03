#!/usr/bin/env python3
"""Same replay as build/tools/inertial_parity_dump, done with navtk, then diffed.

    Cobra/.venv/bin/python tools/inertial_parity_check.py [log] [seconds]
"""
import json, subprocess, sys
import numpy as np
import aspn23_xtensor as ax
from navtk import navutils as nv
from navtk.inertial import BufferedImu, ImuErrors, ManualHeadingAlignment
from pntos.cobra.config import ImuConfig, imu_model_from_config
from pntos.cobra.utils import (convert_imu_to_cpp, convert_message, convert_pva_from_cpp, convert_timestamp_to_cpp)
from pntos.cobra.utils.lcm_utils import decode_aspn_lcm_msg, marshal_from_lcm
from lcm import EventLog
from aspn23 import TypeTimestamp, TypeHeader, MeasurementPositionVelocityAttitude as PVA, MeasurementPositionVelocityAttitudeReferenceFrame as RF, MeasurementPositionVelocityAttitudeErrorModel as EM, MeasurementImu

log = sys.argv[1] if len(sys.argv) > 1 else None
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
if log is None:
    from pntos_python_datasets_lcm import EXAMPLE_LCM_LOG as log
imu_cfg = ImuConfig(group='g', accel_bias_sigma=(2.4e-3,)*3, accel_bias_tau=(300.0,)*3, accel_random_walk_sigma=(3.887e-6,)*3,
                    gyro_bias_sigma=(2e-4,)*3, gyro_bias_tau=(500.0,)*3, gyro_random_walk_sigma=(9.9e-4, 9.9e-4, 6.7e-5),
                    accel_bias_initial_sigma=(0.072,)*3, gyro_bias_initial_sigma=(0.003,)*3)
align = ManualHeadingAlignment(0.06895795874629593, 0.02236067977, imu_model_from_config(imu_cfg), 10.0)
ts = lambda ns: convert_timestamp_to_cpp(TypeTimestamp(ns))
def pva_vec(p):  # aspn23_xtensor PVA
    return np.array([p.get_time_of_validity().get_elapsed_nsec() * 1e-9, p.get_p1(), p.get_p2(), p.get_p3(), p.get_v1(), p.get_v2(), p.get_v3(), *p.get_quaternion()])
def imu_vec(m):
    return np.array([m.get_time_of_validity().get_elapsed_nsec() * 1e-9, *m.get_meas_accel(), *m.get_meas_gyro()])

ref = {}
t0 = None; buf = None; align_t = None
for ev in EventLog(log):
    if ev.channel not in ('/sensor/vn-100/imu', '/sensor/ublox-ZED-F9T/position'):
        continue
    msg = marshal_from_lcm(decode_aspn_lcm_msg(ev.data))
    t = msg.time_of_validity.elapsed_nsec
    if t0 is None: t0 = t
    if (t - t0) * 1e-9 > seconds: break
    if buf is None:
        align.process(convert_message(msg))
        if align.check_alignment_status() == align.check_alignment_status().ALIGNED_GOOD:
            ok, sol = align.get_computed_alignment()
            okc, cov = align.get_computed_covariance()
            oke, err = align.get_imu_errors()
            align_t = sol.time.get_elapsed_nsec()
            ref['align_solution'] = np.array([align_t * 1e-9, *sol.pos, *sol.vel, *nv.dcm_to_rpy(sol.rot_mat.T)])
            ref['align_cov_diag'] = np.diag(cov)
            ref['align_imu_errors'] = np.concatenate([err.accel_biases, err.gyro_biases])
            q = nv.dcm_to_quat(sol.rot_mat.T)
            pva_py = PVA(TypeHeader(0,0,0,0), TypeTimestamp(align_t), RF.GEODETIC, *sol.pos, *sol.vel, q, cov[:9,:9], EM.NONE, np.array([]), [])
            from pntos.cobra.utils import convert_pva_to_cpp
            buf = BufferedImu(convert_pva_to_cpp(pva_py), expected_dt=0.01, buffer_length=10.0)
            buf.reset(imu_errs=ImuErrors(err.accel_biases, err.gyro_biases, np.zeros(3), np.zeros(3), ts(align_t)))
        continue
    if isinstance(msg, MeasurementImu):
        buf.add_data(convert_imu_to_cpp(msg))
span = buf.time_span()
first_t, last_t = span[0].get_elapsed_nsec(), span[1].get_elapsed_nsec()
ref['span'] = np.array([first_t * 1e-9, last_t * 1e-9])
print(f'py: align_t={align_t*1e-9:.6f} span=({first_t*1e-9:.6f}, {last_t*1e-9:.6f}) in_range(align+1s)={buf.in_range(ts(align_t+1_000_000_000))}', file=sys.stderr)
grid = []
t = align_t + ((first_t - align_t) // 1_000_000_000 + 1) * 1_000_000_000
while t <= last_t:
    p = buf.calc_pva(ts(t))
    grid.append(pva_vec(p) if p is not None else np.full(11, np.nan)); t += 1_000_000_000
ref['pva_grid'] = np.array(grid)
ref['pva_latest'] = pva_vec(buf.calc_pva(ts(last_t)))
mid = align_t + 8_000_000_000
ref['force_rate_mid'] = imu_vec(buf.calc_force_and_rate(ts(mid)))
ref['force_rate_avg'] = imu_vec(buf.calc_force_and_rate(ts(mid - 500_000_000), ts(mid + 500_000_000)))
e0 = buf.get_imu_errors(ts(mid))
ref['imu_errors_mid'] = np.concatenate([e0.accel_biases, e0.gyro_biases])
p_mid = buf.calc_pva(ts(mid))
C = nv.quat_to_dcm(p_mid.get_quaternion())
q2 = nv.dcm_to_quat(nv.correct_dcm_with_tilt(C, np.array([0.001, -0.002, 0.003])))
pert = PVA(TypeHeader(0,0,0,0), TypeTimestamp(mid), RF.GEODETIC, p_mid.get_p1()+1e-6, p_mid.get_p2()-1e-6, p_mid.get_p3()+2.0,
           p_mid.get_v1()+0.1, p_mid.get_v2()-0.1, p_mid.get_v3()+0.05, q2, np.zeros((9,9)), EM.NONE, np.array([]), [])
from pntos.cobra.utils import convert_pva_to_cpp
buf.reset(convert_pva_to_cpp(pert))
buf.reset(imu_errs=ImuErrors(e0.accel_biases + np.array([0.01,0,0]), e0.gyro_biases + np.array([0,1e-4,0]), np.zeros(3), np.zeros(3), ts(mid)))
ref['pva_after_reset_latest'] = pva_vec(buf.calc_pva(ts(last_t)))
ref['pva_after_reset_mid_plus_2'] = pva_vec(buf.calc_pva(ts(mid + 2_000_000_000)))
ref['pva_no_reset_since'] = pva_vec(buf.calc_pva_no_reset_since(ts(last_t), ts(mid - 1_000_000_000)))

cpp = json.loads(subprocess.check_output(['build/tools/inertial_parity_dump', log, str(seconds)], text=True))
bad = 0
for k, r in ref.items():
    r = np.atleast_2d(np.asarray(r, dtype=float)); c = np.atleast_2d(np.asarray(cpp[k], dtype=float))
    if r.shape != c.shape and r.T.shape == c.shape: r = r.T
    if r.shape != c.shape:
        print(f'{k:28s} SHAPE py {r.shape} cpp {c.shape}'); bad += 1; continue
    d = np.abs(r - c); i = np.unravel_index(np.argmax(d), d.shape)
    ok = d.max() <= 1e-9 * max(1.0, np.abs(r).max())
    bad += not ok
    print(f'{k:28s} {"ok " if ok else "DIFF"} max|py-cpp| = {d.max():.3e} at {i} (py {r[i]:.9g}, cpp {c[i]:.9g})')
sys.exit(1 if bad else 0)
