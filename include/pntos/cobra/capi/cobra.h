/* C ABI over the Cobra C++ push API (cobra::Filter). Roadmap Phase 3.
 *
 *   cobra_filter* f = cobra_filter_create("configs/pos_ins.json", NULL);
 *   cobra_filter_push_imu(f, "/sensor/vn-100/imu", tov_ns, accel, gyro, 1);
 *   cobra_filter_push_position(f, "/sensor/ublox-ZED-F9T/position", tov_ns, lat, lon, alt, cov);
 *   cobra_pva sol; while (cobra_filter_poll_solution(f, &sol)) ...
 *   cobra_filter_stop(f); cobra_filter_destroy(f);
 *
 * All functions return 0 / NULL on failure and leave a message in cobra_last_error(). Angles in radians,
 * positions geodetic (lat, lon, height above the ellipsoid), velocities NED, times in nanoseconds of the
 * log's time base. One filter may be used from one thread at a time.
 */
#ifndef PNTOS_COBRA_CAPI_H
#define PNTOS_COBRA_CAPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cobra_filter cobra_filter;

typedef struct cobra_pva {
  int64_t tov_ns;
  double lat_rad, lon_rad, alt_m;
  double vel_ned_mps[3];
  double quat_wxyz[4];   /* platform -> NED, scalar first */
  double cov[81];        /* 9x9 row-major: position (m, NED), velocity, tilt (rad) */
  char channel[96];
} cobra_pva;

/* Version of the library, e.g. "0.2.0". */
const char* cobra_version(void);
/* Message of the last failed call in this thread ("" if none). */
const char* cobra_last_error(void);

/* Loads a JSON config file (same format as cobra_run) and starts the stack with a push transport.
 * `overrides_json` may be NULL or a JSON object with any of: "legacy_q_rotation" (bool), "joseph_form"
 * (bool), "output_log" / "input_log" (strings, only used by the config, not by the C API), "logging_level". */
cobra_filter* cobra_filter_create(const char* config_json_path, const char* overrides_json);
/* Same from the JSON text itself. */
cobra_filter* cobra_filter_create_from_json(const char* config_json, const char* overrides_json);
/* Stops the stack (if still running) and frees everything. */
void cobra_filter_destroy(cobra_filter* f);
/* Stops the stack; returns the process-style exit code (0 unless a plugin logged an ERROR). */
int cobra_filter_stop(cobra_filter* f);

/* Push one measurement. `integrated` != 0 marks delta-velocity / delta-angle IMU samples (m/s, rad),
 * 0 marks rates (m/s^2, rad/s). Covariances are 3x3 row-major. Return 1 on success. */
int cobra_filter_push_imu(cobra_filter* f, const char* channel, int64_t tov_ns, const double accel[3], const double gyro[3], int integrated);
int cobra_filter_push_position(cobra_filter* f, const char* channel, int64_t tov_ns, double lat_rad, double lon_rad, double alt_m,
                               const double cov[9]);
int cobra_filter_push_velocity_ned(cobra_filter* f, const char* channel, int64_t tov_ns, const double vel_ned[3], const double cov[9]);
/* Push any ASPN-23 message in its LCM encoding (what an LCM log or network carries). */
int cobra_filter_push_lcm(cobra_filter* f, const char* channel, const uint8_t* data, size_t len);

/* Dequeue the next solution the mediator published (once per publish_interval of message time).
 * Returns 1 and fills `out`, or 0 if none is waiting. */
int cobra_filter_poll_solution(cobra_filter* f, cobra_pva* out);
/* The best solution at `tov_ns` on demand (1 = filled, 0 = not available yet / out of range). */
int cobra_filter_solution_at(cobra_filter* f, int64_t tov_ns, cobra_pva* out);
/* 1 if any plugin has logged an ERROR since creation. */
int cobra_filter_error_logged(cobra_filter* f);

#ifdef __cplusplus
}
#endif
#endif
