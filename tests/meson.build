test_sources = {
  'smoke': files('test_smoke.cpp'),
}

foreach name, srcs : test_sources
  exe = executable('test_' + name, srcs,
    dependencies: [cobra_dep, gtest_main_dep, gmock_dep])
  test(name, exe, protocol: 'gtest', timeout: 120)
endforeach
