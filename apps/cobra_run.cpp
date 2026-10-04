// Generic runner: builds the app a JSON config file describes and runs it.
//   cobra_run config.json [output.log] [input.log] [--legacy-q|--corrected-q] [--no-joseph]
//             [--dump-config out.json] [--dump-registry out.json] [--quiet]
// The example files in configs/ reproduce every app in apps/ (they were written by those apps'
// --dump-config); `cobra_run --list-presets` prints the IMU and GNSS preset names.
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/presets/Presets.hpp>

#include <iostream>

int main(int argc, char** argv) {
  using namespace pntos::cobra;
  if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
    std::cerr << "usage: cobra_run config.json [output.log] [input.log] [--legacy-q|--corrected-q] [--no-joseph]\n"
                 "                 [--dump-config out.json] [--dump-registry out.json] [--quiet]\n"
                 "       cobra_run --list-presets\n";
    return argc < 2 ? 2 : 0;
  }
  if (std::string(argv[1]) == "--list-presets") {
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
    std::cerr << "cobra_run: " << e.what() << "\n";
    return 2;
  }
}
