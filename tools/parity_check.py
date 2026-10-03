#!/usr/bin/env python3
"""Compare build/tools/parity_dump output with the Python Cobra / navtk implementation.

    Cobra/.venv/bin/python tools/parity_check.py            # runs build/tools/parity_dump itself
Prints the max abs difference per quantity; exits 1 if any exceeds 1e-9 (relative 1e-9 for large values).
"""
import json, subprocess, sys
import numpy as np
from aspn23 import (MeasurementImu, MeasurementImuImuType, MeasurementPosition, MeasurementPositionErrorModel,
                    MeasurementPositionReferenceFrame, MeasurementPositionVelocityAttitude,
                    MeasurementPositionVelocityAttitudeErrorModel, MeasurementPositionVelocityAttitudeReferenceFrame,
                    MeasurementVelocity, MeasurementVelocityErrorModel, MeasurementVelocityReferenceFrame, TypeHeader,
                    TypeTimestamp)
from navtk import navutils as nv
from pntos.api import EstimateWithCovariance, EstimateWithCovarianceType, Message
from pntos.cobra.config import ImuConfig
from pntos.cobra.internal import DummyMediator, PinsonErrorToStandard
from pntos.cobra.standard_plugins.state_modeling.Pinson15NedBlock import Pinson15NedBlock
from pntos.cobra.standard_plugins.state_modeling.PinsonPositionMeasurementProcessor import PinsonPositionMeasurementProcessor
from pntos.cobra.standard_plugins.state_modeling.PinsonWithNedFogmPositionMeasurementProcessor import PinsonWithNedFogmPositionMeasurementProcessor
from pntos.cobra.standard_plugins.state_modeling.PinsonVelocityMeasurementProcessor import PinsonVelocityMeasurementProcessor
from pntos.cobra.utils.orchestration_utils import apply_error_states

lat, lon, alt = 0.69, -0.8, 250.0
rpy = np.array([0.05, -0.1, 0.8]); vel = np.array([5.0, -3.0, 0.5]); q = nv.rpy_to_quat(rpy)
tilt = np.array([0.002, -0.001, 0.003]); force = np.array([0.2, -0.1, -9.7]); rate = np.array([0.01, -0.02, 0.03])
t_ns = 1_000_000_000
ref = {}
ref['quat'] = q
C = nv.quat_to_dcm(q)
ref['quat_to_dcm'] = C
ref['dcm_to_rpy'] = nv.dcm_to_rpy(C)
ref['correct_dcm_with_tilt'] = nv.correct_dcm_with_tilt(C, tilt)
ref['ortho_dcm'] = nv.ortho_dcm(C + 1e-3 * np.ones((3, 3)))
ref['gravity_schwartz'] = nv.calculate_gravity_schwartz(alt, lat)
ref['skew'] = nv.skew(vel)
ref['d_rpy_to_dcm_wrt_r'] = nv.d_rpy_to_dcm_wrt_r(rpy)
ref['d_rpy_to_dcm_wrt_y'] = nv.d_rpy_to_dcm_wrt_y(rpy)
ref['meridian_radius'] = nv.meridian_radius(lat)
ref['north_to_delta_lat'] = nv.north_to_delta_lat(100.0, lat, alt)

med = DummyMediator()
imu = ImuConfig(group='g', accel_bias_sigma=(2.4e-3,)*3, accel_bias_tau=(300.0,)*3, accel_random_walk_sigma=(3.887e-6,)*3,
                gyro_bias_sigma=(2e-4,)*3, gyro_bias_tau=(500.0,)*3, gyro_random_walk_sigma=(9.9e-4, 9.9e-4, 6.7e-5))
pva = MeasurementPositionVelocityAttitude(TypeHeader(0,0,0,0), TypeTimestamp(t_ns), MeasurementPositionVelocityAttitudeReferenceFrame.GEODETIC,
                                          lat, lon, alt, *vel, q, np.zeros((9,9)), MeasurementPositionVelocityAttitudeErrorModel.NONE, np.array([]), [])
imu_msg = MeasurementImu(TypeHeader(0,0,0,0), TypeTimestamp(t_ns), MeasurementImuImuType.SAMPLED, force, rate, [])
pva_m, imu_m = Message(pva, 'pva'), Message(imu_msg, 'imu')
x = np.full((15, 1), 0.01)
def gen(labels):
    n = 15 + 3 * (len(labels) - 1)
    return EstimateWithCovariance(EstimateWithCovarianceType.EWC_GENERIC, np.full((n, 1), 0.01), np.eye(n))

block = Pinson15NedBlock('pinson15', med, imu)
block.receive_aux_data([pva_m, imu_m])
dyn = block.generate_dynamics(gen, TypeTimestamp(t_ns), TypeTimestamp(t_ns + 500_000_000))
ref['pinson_Phi'] = dyn.Phi; ref['pinson_Qd'] = dyn.Qd

la = np.array([-0.5, 0.38, -0.05])
pos = MeasurementPosition(TypeHeader(0,0,0,0), TypeTimestamp(t_ns), MeasurementPositionReferenceFrame.GEODETIC,
                          lat + 2e-6, lon - 3e-6, alt + 1.5, np.eye(3) * 4.0, MeasurementPositionErrorModel.NONE, np.array([]), [])
pmp = PinsonPositionMeasurementProcessor('pos', ['pinson15'], med, la); pmp.receive_aux_data([pva_m])
mm = pmp.generate_model(Message(pos, 'gps'), gen)
ref['pos_H'] = mm.H; ref['pos_z'] = mm.z; ref['pos_h'] = mm.h(x); ref['pos_R'] = mm.R
fmp = PinsonWithNedFogmPositionMeasurementProcessor('pos', ['pinson15', 'f'], med, la); fmp.receive_aux_data([pva_m])
fm = fmp.generate_model(Message(pos, 'gps'), gen)
ref['posfogm_H'] = fm.H; ref['posfogm_h'] = fm.h(np.full((18, 1), 0.01))
velm = MeasurementVelocity(TypeHeader(0,0,0,0), TypeTimestamp(t_ns), MeasurementVelocityReferenceFrame.NED, 5.2, -2.9, 0.4,
                           np.eye(3) * 0.25, MeasurementVelocityErrorModel.NONE, np.array([]), [])
vmp = PinsonVelocityMeasurementProcessor('vel', ['pinson15'], med); vmp.receive_aux_data([pva_m])
vm = vmp.generate_model(Message(velm, 'vel'), gen)
ref['vel_H'] = vm.H; ref['vel_z'] = vm.z; ref['vel_h'] = vm.h(x)
pes = PinsonErrorToStandard(med, 'pinson15', 'direct'); pes.receive_aux_data([pva_m])
ref['pes_convert'] = pes.convert_estimate(x, TypeTimestamp(t_ns)); ref['pes_jacobian'] = pes.jacobian(x, TypeTimestamp(t_ns))
cp = apply_error_states(pva, x)
ref['apply_error_states'] = np.array([cp.p1, cp.p2, cp.p3, cp.v1, cp.v2, cp.v3, *cp.quaternion])

exe = sys.argv[1] if len(sys.argv) > 1 else 'build/tools/parity_dump'
cpp = json.loads(subprocess.check_output([exe], text=True))
bad = 0
for k, r in ref.items():
    r = np.atleast_2d(np.asarray(r, dtype=float)); c = np.atleast_2d(np.asarray(cpp[k], dtype=float))
    if r.shape != c.shape and r.T.shape == c.shape: r = r.T
    if r.shape != c.shape:
        print(f'{k:24s} SHAPE py {r.shape} cpp {c.shape}'); bad += 1; continue
    d = np.max(np.abs(r - c)); scale = max(1.0, np.max(np.abs(r)))
    ok = d <= 1e-9 * scale
    bad += not ok
    print(f'{k:24s} {"ok " if ok else "DIFF"} max|py-cpp| = {d:.3e} (scale {scale:.3g})')
    if not ok and r.size <= 100:
        np.set_printoptions(precision=6, suppress=True, linewidth=160)
        print('   py :', r.ravel()[:30]); print('   cpp:', c.ravel()[:30])
sys.exit(1 if bad else 0)
