/* C example: replay an LCM log through the Cobra C ABI and print the solutions.
 *   build/examples/c_run_log configs/pos_ins.json path/to/log.lcmlog
 * Reads the LCM event-log format directly (sync 0xEDA1DA01, big-endian header), pushes the IMU and
 * position channels as raw LCM bytes and prints every published solution. No C++ in this file. */
#include <pntos/cobra/capi/cobra.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t be64(const unsigned char* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
  return v;
}
static uint32_t be32(const unsigned char* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s config.json input.lcmlog [channel ...]\n", argv[0]);
    return 2;
  }
  const char* channels_default[] = {"/sensor/vn-100/imu", "/sensor/ublox-ZED-F9T/position"};
  const char** channels = argc > 3 ? (const char**)&argv[3] : channels_default;
  int nchannels = argc > 3 ? argc - 3 : 2;

  cobra_filter* f = cobra_filter_create(argv[1], "{\"logging_level\": \"WARN\"}");
  if (!f) {
    fprintf(stderr, "cobra_filter_create failed: %s\n", cobra_last_error());
    return 1;
  }
  printf("cobra %s: filter started from %s\n", cobra_version(), argv[1]);

  FILE* in = fopen(argv[2], "rb");
  if (!in) {
    perror(argv[2]);
    return 1;
  }
  unsigned char header[28];
  char channel[256];
  unsigned char* data = NULL;
  size_t cap = 0;
  long pushed = 0, solutions = 0;
  cobra_pva sol, last;
  memset(&last, 0, sizeof last);
  while (fread(header, 1, 28, in) == 28) {
    if (be32(header) != 0xEDA1DA01u) {
      fseek(in, -27, SEEK_CUR);
      continue;
    }
    uint32_t clen = be32(header + 20), dlen = be32(header + 24);
    if (clen >= sizeof channel) break;
    if (fread(channel, 1, clen, in) != clen) break;
    channel[clen] = 0;
    if (dlen > cap) {
      cap = dlen;
      data = (unsigned char*)realloc(data, cap);
    }
    if (fread(data, 1, dlen, in) != dlen) break;
    int wanted = 0;
    for (int i = 0; i < nchannels; ++i) wanted |= strcmp(channel, channels[i]) == 0;
    if (!wanted) continue;
    if (!cobra_filter_push_lcm(f, channel, data, dlen)) {
      fprintf(stderr, "push failed on %s: %s\n", channel, cobra_last_error());
      continue;
    }
    ++pushed;
    while (cobra_filter_poll_solution(f, &sol)) {
      ++solutions;
      last = sol;
      if (solutions % 300 == 1)
        printf("t=%.3f s  lat %.7f  lon %.7f  alt %.2f  vn %.2f ve %.2f vd %.2f  sigma_n %.2f m\n", sol.tov_ns * 1e-9,
               sol.lat_rad * 57.29577951308232, sol.lon_rad * 57.29577951308232, sol.alt_m, sol.vel_ned_mps[0], sol.vel_ned_mps[1],
               sol.vel_ned_mps[2], sol.cov[0] > 0 ? __builtin_sqrt(sol.cov[0]) : 0.0);
    }
  }
  fclose(in);
  free(data);
  int code = cobra_filter_stop(f);
  printf("pushed %ld messages, %ld solutions, last at t=%.3f s on %s, exit code %d%s\n", pushed, solutions, last.tov_ns * 1e-9,
         last.channel, code, cobra_filter_error_logged(f) ? " (errors logged)" : "");
  cobra_filter_destroy(f);
  return code;
}
