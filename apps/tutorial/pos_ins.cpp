// Port of pntos.apps.tutorial.pos_ins: the minimal GPS position + IMU closed-loop filter.
//   tutorial_pos_ins [output.log] [input.log] [--no-joseph] [--legacy-q|--corrected-q]
#include "tutorial_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  return run_tutorial_app("tutorial_pos_ins", parse_args(argc, argv), /*with_velocity=*/false);
}
