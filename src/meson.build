# The pntOS API is header-only (abstract classes + value types).
pntos_api_dep = declare_dependency(
  include_directories: pntos_inc,
  dependencies: [eigen_dep, aspn_eigen_dep, aspn_c_dep, thread_dep])

# Header-only third-party code (see third_party/NOTICE.md).
third_party_inc = include_directories('../third_party/lcm', '../third_party/aspn23_lcm', '../third_party')

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
  'cobra/inertial/Mechanization.cpp',
  'cobra/inertial/BufferedImu.cpp',
  'cobra/inertial/StandardInertialPlugin.cpp',
  'cobra/initialization/Alignment.cpp',
  'cobra/initialization/InitializationPlugins.cpp',
  'cobra/preprocessing/StandardPreprocessorPlugin.cpp',
  'cobra/transport/LcmLog.cpp',
  'cobra/transport/LcmConversions.cpp',
  'cobra/transport/LcmLogTransportPlugin.cpp',
  'cobra/utils/hdf5.cpp',
  'cobra/diagnostics/DiagnosticLogPlugin.cpp',
  'cobra/extras/AdvancedPreprocessorPlugin.cpp',
  'cobra/tutorial/TutorialStateModelingPlugin.cpp',
  'cobra/tutorial/TutorialOrchestrationPlugin.cpp',
  'cobra/tutorial/UiLogPlottingPlugin.cpp',
  'cobra/presets/Presets.cpp',
  'cobra/config/JsonConfig.cpp',
  'cobra/app/AppBuilder.cpp',
  'cobra/app/Filter.cpp',
  'cobra/transport/LcmUdpTransportPlugin.cpp',
  'cobra/transport/CsvTransportPlugin.cpp',
  'cobra/capi/cobra_c.cpp',
  'cobra/utils/geoid.cpp',
)

cobra_lib = library('pntos-cobra', cobra_sources,
  include_directories: [pntos_inc, third_party_inc],
  dependencies: [pntos_api_dep],
  version: meson.project_version(),
  install: true)

# Installation: headers, the geoid grid, the example configs and a pkg-config file.
install_subdir('../include/pntos', install_dir: get_option('includedir'))
install_subdir('../third_party/nlohmann', install_dir: get_option('includedir') / 'pntos' / 'third_party')
install_data('../data/egm96_15min.bin', install_dir: get_option('datadir') / 'pntos-cobra')
install_subdir('../configs', install_dir: get_option('datadir') / 'pntos-cobra')
pkg = import('pkgconfig')
pkg.generate(cobra_lib,
  name: 'pntos-cobra',
  description: 'C++ port of the Cobra pntOS navigation filter (Pinson15 GNSS/INS EKF, plugins, config files, C ABI)',
  url: 'https://github.com/rreper/Cobra',
  subdirs: ['pntos/third_party'],
  requires: ['eigen3'])

cobra_dep = declare_dependency(
  link_with: cobra_lib,
  include_directories: third_party_inc,
  dependencies: [pntos_api_dep])
