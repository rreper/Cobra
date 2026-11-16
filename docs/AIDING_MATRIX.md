# Aiding matrix: a complete GNSS outage, by what else the filter has

Generated 2026-11-16 10:09 by `tools/run_aiding_matrix.py` in 23 s. `configs/pos_ins.json` with the `vn100_driving` tuning on the full example log; every channel of the receiver withheld at 300, 700, 1100, 1500, 1900, 2300 s of the drive. Horizontal error against the truth channel at the end of each outage, in metres: mean and worst of the six, the filter's own sigma, and their ratio (near 1: the covariance is honest). "No outage": RMS over the drive, horizontal / vertical.

| aiding | no outage, h / v RMS | 60 s outage: mean (worst), sigma, ratio | 120 s outage: mean (worst), sigma, ratio | 40 s outage while standing: error, sigma | what it is |
|---|---|---|---|---|---|
| imu_only | 1.55 / 1.57 | 24.9 (53.6), 28.8, 0.97 | 128.2 (278.8), 138.6, 1.07 | 11.7, 13.4 | the IMU alone |
| constraints | 1.52 / 1.69 | 12.6 (25.4), 14.1, 1.06 | 13.3 (25.5), 20.5, 0.83 | 11.4, 9.9 | no sideways / vertical motion, every second from the IMU clock; misalignment estimated |
| wheel | 1.59 / 1.68 | 4.8 (10.8), 4.4, 1.14 | 7.2 (18.6), 7.5, 1.18 | 2.5, 2.5 | wheel speed with a 2 % scale error and the constraints; scale and misalignment estimated |
| wheel_uncalibrated | 7.88 / 7.18 | 12.3 (24.1), 3.1, 4.58 | 19.4 (30.7), 4.5, 4.85 | 7.0, 2.5 | the same through pinson_body_velocity: no calibration states |
| velocity3 | 1.68 / 1.65 | 6.1 (11.7), 3.6, 1.82 | 7.2 (12.2), 5.8, 1.54 | 2.2, 2.1 | three-axis body velocity (0.05 m/s); scale and misalignment estimated |
| full | 1.16 / 1.27 | 1.6 (3.5), 2.5, 0.77 | 1.2 (1.9), 2.7, 0.51 | 1.5, 2.0 | wheel, a second receiver (3 m) and the barometer; the outage takes the first receiver only |
| zero_motion | 1.57 / 1.61 | 10.7 (23.8), 14.5, 1.05 | 33.8 (65.4), 38.5, 1.14 | 2.1, 2.8 | zero velocity (0.2 m/s) and zero rate once a second while the IMU is quiet |
| constraints_zero_motion | 1.57 / 1.71 | 6.0 (12.5), 6.4, 1.02 | 9.3 (16.5), 12.9, 0.92 | 2.0, 2.7 | the constraints and the zero-motion updates |
| zero_motion_confirmed | 1.53 / 1.61 | 10.3 (22.5), 14.4, 1.01 | 33.8 (66.1), 38.5, 1.13 | 1.8, 2.7 | zero velocity and zero rate where the receiver's velocity confirms the standstill (0.05 m/s north and east, 0.2 m/s down); the IMU alone (0.2 m/s) in an outage |
| constraints_zero_motion_confirmed | 1.55 / 1.71 | 6.1 (12.3), 6.4, 1.03 | 9.6 (16.5), 12.9, 0.94 | 1.9, 2.6 | the constraints and the confirmed zero-motion updates |

The wheel speed and the second receiver do not exist in the recording: the first is the log's simulated body velocity (truth plus 0.05 m/s) cut down to its forward axis and scaled by 1.02, the second is the truth position with 3 m of noise once a second. The constraints assume a vehicle that neither slides sideways nor leaves the ground. The outage while standing is the one stop of the drive that is long enough (1108 to 1158 s); in it the full row keeps its second receiver.
