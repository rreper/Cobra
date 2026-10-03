// Port of pntos.apps.tutorial.pos_vel_ins: tutorial pos_ins plus NED velocity measurements.
//   tutorial_pos_vel_ins [output.log] [input.log] [--no-joseph] [--legacy-q|--corrected-q]
#include "tutorial_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  return run_tutorial_app("tutorial_pos_vel_ins", parse_args(argc, argv), /*with_velocity=*/true);
}
