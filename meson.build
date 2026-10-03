project('pntos-cpp', 'cpp',
  version: '0.1.0',
  license: 'Apache-2.0',
  meson_version: '>=1.3',
  default_options: [
    'cpp_std=c++20',
    'warning_level=3',
    'buildtype=debugoptimized',
    'default_library=static',
    'b_ndebug=if-release',
  ])

cpp = meson.get_compiler('cpp')
add_project_arguments(cpp.get_supported_arguments('-Wno-unused-parameter'), language: 'cpp')

# ---------------------------------------------------------------------------
# Dependencies (all available as meson wraps under subprojects/)
# ---------------------------------------------------------------------------

# Eigen (header-only)
eigen_dep = dependency('eigen3', version: '>=3.4', include_type: 'system',
  fallback: ['eigen', 'eigen_dep'])

# ASPN-23 messages, Eigen flavour (aspn-generated: aspn-c + aspn-cpp/eigen)
aspn_generated = subproject('aspn-generated', default_options: [
  'aspn-c=enabled',
  'aspn-cpp=enabled',
  'aspn-cpp-eigen=enabled',
  'aspn-cpp-stl=disabled',
  'aspn-cpp-xtensor=disabled',
  'aspn-cpp-xtensor-py=disabled',
  'dds-generated=disabled',
  'lcm-generated=disabled',
  'warning_level=0',
  'werror=false',
])
aspn_c_dep = dependency('aspn23')
aspn_eigen_dep = dependency('aspn23-eigen', include_type: 'system')

# Threads
thread_dep = dependency('threads')

# ---------------------------------------------------------------------------
# Library layout
# ---------------------------------------------------------------------------
pntos_inc = include_directories('include')

subdir('src')

if get_option('tests')
  gtest_proj = subproject('gtest')
  gtest_dep = gtest_proj.get_variable('gtest_dep')
  gtest_main_dep = gtest_proj.get_variable('gtest_main_dep')
  gmock_dep = gtest_proj.get_variable('gmock_dep')
  subdir('tests')
endif

if get_option('apps')
  subdir('apps')
  subdir('tools')
endif
