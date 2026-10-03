test_sources = {
  'smoke': files('test_smoke.cpp'),
  'api': files('test_api.cpp'),
  'ekf_fusion_strategy': files('test_ekf_fusion_strategy.cpp'),
  'simple_state_blocks': files('test_simple_state_blocks.cpp'),
  'registry': files('test_registry.cpp'),
}

foreach name, srcs : test_sources
  exe = executable('test_' + name, srcs,
    dependencies: [cobra_dep, gtest_main_dep, gmock_dep])
  test(name, exe, protocol: 'gtest', timeout: 120)
endforeach
