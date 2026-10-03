# The pntOS API is header-only (abstract classes + value types).
pntos_api_dep = declare_dependency(
  include_directories: pntos_inc,
  dependencies: [eigen_dep, aspn_eigen_dep, aspn_c_dep, thread_dep])

# Cobra plugin implementations.
cobra_sources = files(
  'cobra/utils/logging.cpp',
  'cobra/utils/arrays.cpp',
  'cobra/utils/navutils.cpp',
  'cobra/utils/aspn.cpp',
  'cobra/config/BaseConfig.cpp',
  'cobra/config/configs.cpp',
  'cobra/EkfFusionStrategyPlugin.cpp',
  'cobra/StandardLoggingPlugin.cpp',
  'cobra/StandardRegistryPlugin.cpp',
  'cobra/state_modeling/SimpleStateBlocks.cpp',
  'cobra/state_modeling/Pinson15NedBlock.cpp',
  'cobra/state_modeling/MeasurementProcessors.cpp',
  'cobra/state_modeling/VirtualStateBlocks.cpp',
  'cobra/state_modeling/StandardStateModelingPlugin.cpp',
  'cobra/fusion/VirtualStateBlockManager.cpp',
  'cobra/fusion/StandardFusionPlugin.cpp',
  'cobra/utils/plugins.cpp',
  'cobra/controller/StandardMessageStreamConfig.cpp',
  'cobra/controller/StandardMediator.cpp',
  'cobra/controller/StandardControllerPlugin.cpp',
  'cobra/dummy/DummyPlugins.cpp',
  'cobra/orchestration/OrchestrationUtils.cpp',
  'cobra/orchestration/StandardOrchestrationPlugin.cpp',
)

cobra_lib = library('pntos-cobra', cobra_sources,
  include_directories: pntos_inc,
  dependencies: [pntos_api_dep],
  install: true)

cobra_dep = declare_dependency(
  link_with: cobra_lib,
  dependencies: [pntos_api_dep])
