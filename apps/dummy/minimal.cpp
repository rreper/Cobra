// Port of pntos.apps.dummy.minimal: dummy controller + dummy orchestration + dummy transport +
// console logging. Runs for half a second and exits.
#include <pntos/cobra/StandardLoggingPlugin.hpp>
#include <pntos/cobra/dummy/DummyPlugins.hpp>

int main() {
  using namespace pntos;
  api::PluginList plugins{
      std::make_shared<cobra::DummyOrchestrationPlugin>("Dummy orchestration"),
      std::make_shared<cobra::DummyTransportPlugin>("Dummy transport"),
      std::make_shared<cobra::StandardLoggingPlugin>("Standard logging"),
  };
  cobra::DummyControllerPlugin controller("Dummy controller");
  controller.init_plugin(std::nullopt, nullptr);
  controller.take_control(plugins);
  return 0;
}
