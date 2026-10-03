test_sources = {
  'smoke': files('test_smoke.cpp'),
  'api': files('test_api.cpp'),
  'ekf_fusion_strategy': files('test_ekf_fusion_strategy.cpp'),
  'simple_state_blocks': files('test_simple_state_blocks.cpp'),
  'registry': files('test_registry.cpp'),
  'state_modeling': files('test_state_modeling.cpp'),
  'virtual_state_blocks': files('test_virtual_state_blocks.cpp'),
  'vsb_manager': files('test_vsb_manager.cpp'),
  'fusion_engine': files('test_fusion_engine.cpp'),
  'message_stream_config': files('test_message_stream_config.cpp'),
  'controller': files('test_controller.cpp'),
  'orchestration': files('test_orchestration.cpp'),
  'inertial': files('test_inertial.cpp'),
  'initialization': files('test_initialization.cpp'),
  'preprocessors': files('test_preprocessors.cpp'),
}

foreach name, srcs : test_sources
  exe = executable('test_' + name, srcs,
    dependencies: [cobra_dep, gtest_main_dep, gmock_dep])
  test(name, exe, protocol: 'gtest', timeout: 120)
endforeach
