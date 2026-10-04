// Generic runner: builds the app a JSON config file describes and runs it.
//   cobra_run config.json [output.log] [input.log] [--legacy-q|--corrected-q] [--no-joseph]
//             [--dump-config out.json] [--dump-registry out.json] [--no-record-input] [--via-push] [--quiet]
// The example files in configs/ reproduce every app in apps/ (they were written by those apps'
// --dump-config); `cobra_run --list-presets` prints the IMU and GNSS preset names.
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/presets/Presets.hpp>

#include <iostream>

int main(int argc, char** argv) { return pntos::cobra::app::cobra_run_main(argc, argv); }
