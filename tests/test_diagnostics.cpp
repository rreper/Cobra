// Port of pntos-cobra/tests/test_diagnostic_log_plugin.py + test_hdf5utils.py (writing half).
// The files written here are also validated with h5py by tools/check_hdf5.py.
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/diagnostics/DiagnosticLogPlugin.hpp>
#include <pntos/cobra/utils/hdf5.hpp>

#include "test_support.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::RegistryValue;

namespace {
std::string out_path(const char* name) {
  const char* dir = std::getenv("PNTOS_TEST_OUT");
  return (std::filesystem::path(dir ? dir : std::filesystem::temp_directory_path().string()) / name).string();
}
std::vector<std::uint8_t> read_all(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
}  // namespace

TEST(Hdf5Writer, FileLayout) {
  cobra::utils::Hdf5Writer w;
  w.add_int64("time", {1, 2, 3});
  w.add_float64("estimate", {3, 2, 1}, {1.5, 2.5, 3.5, 4.5, 5.5, 6.5});
  w.add_strings("state_labels", {1, 2}, {"a_state0", "bb"});
  w.add_bools("flag", {true, false, true});
  w.add_float64("sigma", {2}, {0.25, 0.5});
  const auto bytes = w.serialise();
  ASSERT_GT(bytes.size(), 96u);
  const std::uint8_t sig[8] = {0x89, 'H', 'D', 'F', '\r', '\n', 0x1a, '\n'};
  EXPECT_TRUE(std::equal(sig, sig + 8, bytes.begin()));
  // end-of-file address (offset 40) equals the file size
  std::uint64_t eof = 0;
  for (int i = 7; i >= 0; --i) eof = (eof << 8) | bytes[40 + static_cast<std::size_t>(i)];
  EXPECT_EQ(eof, bytes.size());
  EXPECT_EQ(bytes.size() % 8, 0u);
  const std::string p = out_path("hdf5_writer_layout.hdf5");
  w.write(p);
  EXPECT_EQ(read_all(p), bytes);
}

TEST(Hdf5Writer, RejectsBadNames) {
  cobra::utils::Hdf5Writer w;
  EXPECT_THROW(w.add_int64("", {1}), std::invalid_argument);
  EXPECT_THROW(w.add_int64("a/b", {1}), std::invalid_argument);
  w.add_int64("a", {1});
  EXPECT_THROW(w.add_int64("a", {2}), std::invalid_argument);
}

TEST(Hdf5Writer, SaveStore) {
  std::map<std::string, std::vector<RegistryValue>> store;
  store["time"] = {std::int64_t{7}, std::int64_t{8}};
  Matrix m(2, 1);
  m << 1, 2;
  store["estimate"] = {m, Matrix(m * 2)};  // column vectors -> (2, 2) like numpy 1-D arrays
  Matrix sq(2, 2);
  sq << 1, 2, 3, 4;
  store["cov"] = {sq, sq};  // true matrices keep (2, 2, 2)
  store["state_labels"] = {api::StringArray{"x", "yy"}};
  store["ratio"] = {0.5, 0.25};
  store["ok"] = {true, false};
  store["name"] = {std::string("abc"), std::string("de")};
  std::vector<std::string> logs;
  const std::string p = out_path("hdf5_store.hdf5");
  ASSERT_TRUE(cobra::utils::save_to_hdf5_file(p, store, [&](api::LoggingLevel, const std::string& s) { logs.push_back(s); }));
  EXPECT_TRUE(logs.empty());
  EXPECT_TRUE(std::filesystem::exists(p));
  // mixed types abandon the file, like the Python
  store["mixed"] = {std::int64_t{1}, 2.0};
  EXPECT_FALSE(cobra::utils::save_to_hdf5_file(out_path("hdf5_mixed.hdf5"), store,
                                               [&](api::LoggingLevel, const std::string& s) { logs.push_back(s); }));
  EXPECT_EQ(logs.size(), 1u);
}

TEST(DiagnosticLogPlugin, RecordsEveryNotification) {
  TestMediator med;
  auto registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{});
  registry->init_plugin(std::nullopt, &med);
  med.set_registry(registry->new_registry());
  const std::string p = out_path("diagnostic_plugin.hdf5");
  cobra::DiagnosticLogPlugin plugin("diag", p);
  plugin.init_plugin(std::nullopt, &med);
  for (int i = 0; i < 3; ++i) {
    auto kv = med.registry().batch(cobra::DiagnosticLogPlugin::kGroupToWatch);
    if (i == 0) kv->set("state_labels", api::StringArray{"a", "b"});
    kv->set("time", std::int64_t{100 + i});
    Matrix est(2, 1);
    est << i, -i;
    kv->set("estimate", est);
  }
  // every set notifies, even with an unchanged value (Python StandardKeyValueStore semantics)
  {
    auto kv = med.registry().batch(cobra::DiagnosticLogPlugin::kGroupToWatch);
    kv->set("time", std::int64_t{102});
  }
  const auto& store = plugin.store();
  ASSERT_EQ(store.count("time"), 1u);
  EXPECT_EQ(store.at("time").size(), 4u);
  EXPECT_EQ(store.at("estimate").size(), 3u);
  EXPECT_EQ(store.at("state_labels").size(), 1u);
  EXPECT_EQ(std::get<std::int64_t>(store.at("time")[3]), 102);
  plugin.shutdown_plugin();
  EXPECT_TRUE(std::filesystem::exists(p));
  EXPECT_EQ(med.count(api::LoggingLevel::INFO), 1u);
  EXPECT_FALSE(med.has_error());
}

TEST(DiagnosticLogPlugin, NoMediatorIsAnError) {
  cobra::DiagnosticLogPlugin plugin("diag", out_path("never.hdf5"));
  plugin.init_plugin(std::nullopt, nullptr);
  plugin.shutdown_plugin();  // nothing recorded, nothing written
  EXPECT_FALSE(std::filesystem::exists(out_path("never.hdf5")));
}
