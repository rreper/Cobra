#!/usr/bin/env python3
"""Run the Python pos_ins app on a log with a propagate/update trace (same format as PNTOS_TRACE_FILE).
    Cobra/.venv/bin/python tools/trace_python_pos_ins.py <input.log> <output.log> <trace.txt>
"""
import os, sys, importlib.util
import numpy as np
inp, out, tracef = sys.argv[1:4]
import pntos_python_datasets_lcm as ds
ds.EXAMPLE_LCM_LOG = inp
from pntos.cobra.standard_plugins.fusion import StandardFusionPlugin as sfp
tf = open(tracef, 'w')
orig_prop, orig_upd, orig_peek = sfp.StandardFusionEngine.propagate, sfp.StandardFusionEngine.update, sfp.StandardFusionEngine.peek_ahead
IN_PEEK = [0]
def peek_ahead(self, time, labels):
    IN_PEEK[0] += 1
    try:
        return orig_peek(self, time, labels)
    finally:
        IN_PEEK[0] -= 1
sfp.StandardFusionEngine.peek_ahead = peek_ahead
def line(kind, what, a, b, self):
    if IN_PEEK[0]: return
    P = self.strategy.covariance; x = self.strategy.estimate
    extra = ''
    if os.environ.get('PNTOS_TRACE_FULL'):
        extra = ' | ' + ' '.join(f'{v:.17g}' for v in x[:,0]) + ' | ' + ' '.join(f'{v:.17g}' for v in np.diag(P))
    tf.write(f"{kind} {what} {a} {b} {np.trace(P):.17g} {x[6,0]:.17g} {x[7,0]:.17g} {x[8,0]:.17g} {P[8,8]:.17g}{extra}\n")
def propagate(self, time):
    t_from = self.time.elapsed_nsec
    orig_prop(self, time)
    if self.time.elapsed_nsec != t_from and self.strategy.covariance is not None:
        line('P', 'prop', t_from, self.time.elapsed_nsec, self)
def update(self, label, message):
    n_before = tf.tell()
    orig_upd(self, label, message)
    line('U', label, message.wrapped_message.time_of_validity.elapsed_nsec, self.time.elapsed_nsec, self)
sfp.StandardFusionEngine.propagate = propagate
sfp.StandardFusionEngine.update = update
from pntos.cobra.standard_plugins.state_modeling import Pinson15NedBlock as pb
from aspn23 import MeasurementPositionVelocityAttitude as _PVA, MeasurementImu as _IMU
auxf = open(tracef + '.aux', 'w')
orig_aux = pb.Pinson15NedBlock.receive_aux_data
def receive_aux_data(self, aux):
    if not IN_PEEK[0]:
        parts = []
        for m in aux:
            if m is None: continue
            w = m.wrapped_message
            if isinstance(w, _PVA):
                parts.append('pva %d %s' % (w.time_of_validity.elapsed_nsec, ' '.join(f'{v:.17g}' for v in [w.p1, w.p2, w.p3, w.v1, w.v2, w.v3, *w.quaternion])))
            elif isinstance(w, _IMU):
                parts.append('imu %d %s' % (w.time_of_validity.elapsed_nsec, ' '.join(f'{v:.17g}' for v in [*w.meas_accel, *w.meas_gyro])))
        auxf.write(' '.join(parts) + '\n')
    return orig_aux(self, aux)
pb.Pinson15NedBlock.receive_aux_data = receive_aux_data
spec = importlib.util.spec_from_file_location('pos_ins_app', 'Cobra/pntos-cobra-apps/src/pntos/apps/standard/pos_ins.py')
mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
mod.EXAMPLE_LCM_LOG = inp
sys.argv = ['pos_ins', out]
try:
    mod.main()
except SystemExit:
    pass
tf.close(); auxf.close()
