// cobra_run as a library function (see apps/cobra_run.cpp for the executable).
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/presets/Presets.hpp>

#include <iostream>

namespace pntos::cobra::app {

int cobra_run_main(int argc, char** argv, const char* program) {
  if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
    std::cerr << "usage: " << program << " config.json [output.log] [input.log] [--legacy-q|--corrected-q] [--no-joseph]\n"
                 "                 [--dump-config out.json] [--dump-registry out.json] [--no-record-input] [--via-push] [--quiet]\n"
                 "       " << program << " --list-presets\n";
    return argc < 2 ? 2 : 0;
  }
  if (std::string(argv[1]) == "--list-presets") {
    if (argc > 2 && std::string(argv[2]) == "--json") {
      jsoncfg::json out;
      for (const auto& p : presets::imu_presets()) {
        jsoncfg::json e = jsoncfg::config_to_json(p.config);
        e.erase("type");
        e.erase("group");
        if (e.contains("preset")) {  // expand to numbers: the preset form is for config files
          ImuConfig c = p.config;
          jsoncfg::json full;
          full["accel_bias_sigma"] = c.accel_bias_sigma;
          full["accel_bias_tau"] = c.accel_bias_tau;
          full["accel_random_walk_sigma"] = c.accel_random_walk_sigma;
          full["gyro_bias_sigma"] = c.gyro_bias_sigma;
          full["gyro_bias_tau"] = c.gyro_bias_tau;
          full["gyro_random_walk_sigma"] = c.gyro_random_walk_sigma;
          full["accel_bias_initial_sigma"] = c.accel_bias_initial_sigma;
          full["gyro_bias_initial_sigma"] = c.gyro_bias_initial_sigma;
          e = full;
        }
        e["name"] = p.name;
        e["description"] = p.description;
        e["source"] = p.source;
        out["imu"][p.name] = e;
      }
      for (const auto& p : presets::gnss_presets())
        out["gnss"][p.name] = {{"description", p.description}, {"position", p.position}, {"velocity", p.velocity},
                               {"pva", p.pva}, {"time_bias_sec", p.time_bias_sec}};
      std::cout << out.dump(2) << "\n";
      return 0;
    }
    std::cout << "IMU presets:\n";
    for (const auto& p : presets::imu_presets()) std::cout << "  " << p.name << "  " << p.description << " [" << p.source << "]\n";
    std::cout << "GNSS presets:\n";
    for (const auto& p : presets::gnss_presets()) std::cout << "  " << p.name << "  " << p.description << "\n";
    return 0;
  }
  try {
    AppConfig config = jsoncfg::load_app_config(argv[1]);
    app::RunOptions options = app::parse_run_options(argc, argv, 2);
    return app::run_app(config, options);
  } catch (const std::exception& e) {
    std::cerr << program << ": " << e.what() << "\n";
    return 2;
  }
}

}  // namespace pntos::cobra::app
