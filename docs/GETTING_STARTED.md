# Getting started with the Cobra C++ port

This guide takes you from a clone to running the filter on your own sensors. It assumes a Linux machine with
g++ 13 or newer, Python 3 and about 1 GB of disk for the dependencies meson fetches.

## 1. Build

```bash
git clone https://github.com/rreper/Cobra.git pntos && cd pntos
python3 -m venv .venv && .venv/bin/pip install meson ninja
.venv/bin/meson setup build
.venv/bin/meson compile -C build
.venv/bin/meson test -C build          # 21 suites, all green
```

Everything (Eigen, the ASPN-23 message library, GoogleTest) is a meson subproject; nothing is installed
system-wide. `meson install -C build` installs the library, headers, `cobra_run`, the geoid grid and the example
configs, with a `pntos-cobra.pc` pkg-config file.

## 2. Run an example

The example dataset (43 minutes of a drive with a VN-100 IMU and a u-blox receiver) ships with the Python
Cobra package; `testdata/example_60s.log` is its first minute.

```bash
./build/apps/cobra_run configs/pos_ins.json out.log testdata/example_60s.log
./build/tools/log_stats out.log --truth-log testdata/example_60s.log      # epochs and RMS errors vs truth
```

`configs/` holds one file per example app. Every file is `{"app": ..., "configs": [...]}`: `app` names the
plugins, `configs` is the list of configuration objects with the Python class names as `"type"` and the Python
field names. `DESIGN.md` §7.14 is the reference; `cobra_run --list-presets` prints the sensor presets.

## 3. Describe your sensors

Start from `configs/pos_ins.json` and change, in `StandardOrchestrationConfig`:

| What you have | Where it goes |
|---|---|
| A different IMU | `pinson_sb_config.imu_model` and `alignment_config.imu_model`: `{"preset": "stim300", "group": "config/inertial_state"}` or the six sigma/tau triples explicitly. Presets: vn100, vn100_corrected, vn100_datasheet, stim300, adis16488, hg1700, hg4930, consumer_mems. Datasheet presets are starting points; tune the random-walk sigmas until about 68 % of your errors fall inside one sigma. |
| IMU mounting | `inertial_config.C_imu_to_platform` and the `ImuRotatorConfig` preprocessor (same matrix), `inertial_config.expected_dt` |
| GNSS position only | one `PinsonWithNedFogmPositionMPConfig` with the antenna lever arm (platform frame, metres) and a `FogmStateBlockConfig` for the receiver's correlated error (sigma 1.5 m / tau 300 s for a code receiver; shrink both for RTK) |
| GNSS velocity too | add a `PinsonVelocityMPConfig` on the velocity channel |
| A receiver that outputs PVA | `PosVelMPConfig` on the PVA channel (see `configs/posvel_ins.json`) |
| Receiver latency | `TimeBiasConfig.time_bias` in nanoseconds on the GNSS channels (150 ms for the example u-blox) |
| Barometer | `BarometerToAltitudeConfig` + `AltitudeMPConfig` + an altitude FOGM block; MSL altitudes are converted with the bundled EGM96 geoid (`configs/pos_ins_baro.json`) |
| Wheel / body velocity, zero-velocity | `PinsonBodyVelocityMPConfig` (`configs/pos_ins_bodyvel.json`, `configs/pos_ins_zerovel2d.json`) |
| Multipath, glitches | `"innovation_gate_probability": 0.999` on the processor |
| Initial heading | `ManualHeadingAlignmentConfig` (heading known, vehicle static for `static_time`), `StaticAlignmentConfig` (gyrocompassing-grade IMU), `ManualAlignmentConfig` (everything known), `PvaMessageInitializationConfig` (from a reference PVA) |

The `app` section selects the plugin set: keep `initialization`, `state_modeling: standard`,
`orchestration: standard`; set `transport` to how the data arrives (next section).

## 4. Feed your data

| Source | Transport | Notes |
|---|---|---|
| An LCM log of ASPN-23 messages | `lcm_log` (`LcmLogTransportConfig`) | the default; `cobra_run config.json out.log in.log` |
| Live LCM on the network | `lcm_udp` (`LcmTransportConfig`: url, `subscribe_to` regex, `idle_timeout_sec`, `output_file`) | `configs/pos_ins_network.json`; `build/tools/lcm_log_player in.log --speed 20` replays a log for it |
| CSV files | `csv` (`CsvTransportConfig`: IMU, position, velocity files; solutions to CSV) | `configs/pos_ins_csv.json`; `build/tools/lcm_to_csv` shows the column layout |
| Your own program | the push API (`cobra::Filter`, `include/pntos/cobra/app/Filter.hpp`) or the C ABI (`include/pntos/cobra/capi/cobra.h`) | `examples/c/run_log.c` is a complete C program |

Push API in C++:

```cpp
#include <pntos/cobra/app/Filter.hpp>
auto config = pntos::cobra::jsoncfg::load_app_config("configs/pos_ins.json");
pntos::cobra::Filter filter(config);
filter.push(imu_message);                      // api::Message(shared_ptr<AspnBase>, channel)
for (auto& sol : filter.take_solutions()) ...  // one per second of message time
filter.stop();
```

## 5. Judge the result

- `build/tools/log_stats out.log --truth-log truth.log` or `UiLogPlottingPlugin` (`"ui_log_plotting": true`) give RMS
  errors against a truth PVA channel.
- `tools/run_acceptance.py` and `tools/run_degraded_matrix.py` (Cobra venv) are the full acceptance tools used
  for the port; `docs/DEGRADED_MATRIX.md` shows what to expect from worse IMUs and sparser GNSS.
- `--dump-registry r.json` prints every configuration value the filter actually used.
- `PNTOS_TRACE_FILE=trace.txt` writes one line per propagate / update (`DESIGN.md` §9.5).

## 6. Where to read next

`README.md` (overview), `docs/DESIGN.md` (architecture, every deviation from the Python, config format),
`docs/TESTING.md` (tests and tools), `docs/ROADMAP.md`, `docs/SESSION_NOTES.md` (how the port was built).
