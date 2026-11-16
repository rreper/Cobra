# Aiding matrix: a complete GNSS outage, by what else the filter has

Generated 2026-11-16 06:35 by `tools/run_aiding_matrix.py` in 16 s. `configs/pos_ins.json` with the `vn100_measured` tuning on the full example log; every channel of the receiver withheld at 300, 700, 1100, 1500, 1900, 2300 s of the drive. Horizontal error against the truth channel at the end of each outage, in metres: mean and worst of the six, the filter's own sigma, and their ratio (near 1: the covariance is honest). "No outage": RMS over the drive, horizontal / vertical.

| aiding | no outage, h / v RMS | 60 s outage: mean (worst), sigma, ratio | 120 s outage: mean (worst), sigma, ratio | what it is |
|---|---|---|---|---|
| imu_only | 1.55 / 1.58 | 25.1 (56.6), 26.3, 1.10 | 130.8 (290.7), 130.7, 1.18 | the IMU alone |
| constraints | 1.52 / 1.77 | 13.3 (25.6), 12.9, 1.19 | 15.6 (27.3), 18.5, 0.99 | no sideways / vertical motion, every second from the IMU clock; misalignment estimated |
| wheel | 1.59 / 1.75 | 4.9 (10.6), 4.2, 1.25 | 7.4 (19.5), 7.0, 1.33 | wheel speed with a 2 % scale error and the constraints; scale and misalignment estimated |
| wheel_uncalibrated | 7.92 / 7.31 | 12.4 (24.0), 3.0, 4.64 | 19.7 (31.0), 4.4, 5.01 | the same through pinson_body_velocity: no calibration states |
| velocity3 | 1.73 / 3.24 | 5.7 (11.2), 3.2, 1.96 | 6.4 (11.0), 5.1, 1.66 | three-axis body velocity (0.05 m/s); scale and misalignment estimated |
| full | 1.16 / 1.33 | 1.8 (4.0), 2.4, 0.85 | 1.2 (2.0), 2.6, 0.53 | wheel, a second receiver (3 m) and the barometer; the outage takes the first receiver only |

The wheel speed and the second receiver do not exist in the recording: the first is the log's simulated body velocity (truth plus 0.05 m/s) cut down to its forward axis and scaled by 1.02, the second is the truth position with 3 m of noise once a second. The constraints assume a vehicle that neither slides sideways nor leaves the ground.
