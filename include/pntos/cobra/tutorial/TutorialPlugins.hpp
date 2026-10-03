// Umbrella header for the tutorial plugin set (Python pntos.cobra.tutorial_plugins).
#pragma once

#include <pntos/cobra/initialization/InitializationPlugins.hpp>  // TutorialInitializationPlugin
#include <pntos/cobra/transport/LcmLogTransportPlugin.hpp>
#include <pntos/cobra/tutorial/TutorialOrchestrationPlugin.hpp>
#include <pntos/cobra/tutorial/TutorialStateModelingPlugin.hpp>
#include <pntos/cobra/tutorial/UiLogPlottingPlugin.hpp>

namespace pntos::cobra {
/// The Python tutorial transport is the standard log transport without the channel filter and
/// with a progress bar; LcmLogTransportPlugin already behaves that way when
/// LcmLogTransportConfig::channels_to_process is unset.
using TutorialLcmLogTransportPlugin = LcmLogTransportPlugin;
}  // namespace pntos::cobra
