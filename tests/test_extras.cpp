// Tests for the extras zero-velocity preprocessor (no Python unit test exists; the Python coverage is
// the pos_ins_zerovel2d integration app).
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/extras/AdvancedPreprocessorPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <cmath>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Message;

namespace {
class ExtrasTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto zv = std::make_shared<cobra::ZeroVelocity2dGeneratorConfig>();
    zv->group_ = "config/pseudovel_generator";
    zv->channels = std::vector<std::string>{"/pos"};
    zv->trigger_dt_sec = 30.0;
    zv->lateral_vel_sigma = 0.5;
    zv->vertical_vel_sigma = 1.0;
    zv->output_channel = "/generated/zero/velocity2d";
    registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{zv});
    registry->init_plugin(std::nullopt, &med);
    med.set_registry(registry->new_registry());
    plugin = std::make_unique<cobra::AdvancedPreprocessorPlugin>("extras");
    plugin->init_plugin(std::nullopt, &med);
  }
  Message pos(std::int64_t t, const char* ch = "/pos") { return Message(make_position(t, 1, 2, 3, Matrix::Identity(3, 3)), ch); }
  TestMediator med;
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::unique_ptr<cobra::AdvancedPreprocessorPlugin> plugin;
};
}  // namespace

TEST_F(ExtrasTest, ConfigRoundTrip) {
  auto c = cobra::ZeroVelocity2dGeneratorConfig::from_registry(med, "config/pseudovel_generator");
  ASSERT_TRUE(c);
  EXPECT_EQ(c->identifier, "zero_velocity2d_generator");
  EXPECT_EQ(c->trigger_dt_sec, 30.0);
  EXPECT_EQ(c->lateral_vel_sigma, 0.5);
  EXPECT_EQ(c->vertical_vel_sigma, 1.0);
  EXPECT_EQ(c->output_channel, "/generated/zero/velocity2d");
  ASSERT_TRUE(c->channels);
  EXPECT_EQ(c->channels->at(0), "/pos");
  // the orchestration reads the base type back
  auto base = cobra::PreprocessorConfig::from_registry(med, "config/pseudovel_generator");
  ASSERT_TRUE(base);
  EXPECT_EQ(base->identifier, cobra::ZeroVelocity2dGeneratorConfig::kIdentifier);
}

TEST_F(ExtrasTest, PluginErrors) {
  EXPECT_EQ(plugin->preprocessor_identifiers(), std::vector<std::string>{"zero_velocity2d_generator"});
  EXPECT_EQ(plugin->new_preprocessor(1, std::string("config/pseudovel_generator")), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(0, std::nullopt), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(0, std::string("config/missing")), nullptr);
  EXPECT_GE(med.count(api::LoggingLevel::ERROR), 3u);  // the missing group also logs from the config reader
}

TEST_F(ExtrasTest, GeneratesTwoDZeroVelocity) {
  auto pp = plugin->new_preprocessor(0, std::string("config/pseudovel_generator"));
  ASSERT_TRUE(pp);
  const std::int64_t s = 1'000'000'000;
  // first trigger fires immediately
  auto out = pp->process_pntos_message(pos(10 * s));
  ASSERT_TRUE(out);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_EQ((*out)[0].source_identifier, "/pos");
  EXPECT_EQ((*out)[1].source_identifier, "/generated/zero/velocity2d");
  auto vel = (*out)[1].as<aspn23_eigen::MeasurementVelocity>();
  ASSERT_TRUE(vel);
  EXPECT_EQ(vel->get_time_of_validity().get_elapsed_nsec(), 10 * s);
  EXPECT_EQ(vel->get_reference_frame(), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR);
  EXPECT_TRUE(std::isnan(vel->get_x()));  // forward axis not measured
  EXPECT_EQ(vel->get_y(), 0.0);
  EXPECT_EQ(vel->get_z(), 0.0);
  Matrix R = vel->get_covariance();
  ASSERT_EQ(R.rows(), 2);
  EXPECT_DOUBLE_EQ(R(0, 0), 0.25);
  EXPECT_DOUBLE_EQ(R(1, 1), 1.0);
  // within trigger_dt: pass-through only
  out = pp->process_pntos_message(pos(20 * s));
  ASSERT_TRUE(out);
  EXPECT_EQ(out->size(), 1u);
  // other channels never trigger
  out = pp->process_pntos_message(pos(50 * s, "/other"));
  ASSERT_TRUE(out);
  EXPECT_EQ(out->size(), 1u);
  // after trigger_dt: fires again at the new time, and the earlier message is untouched
  out = pp->process_pntos_message(pos(40 * s));
  ASSERT_TRUE(out);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_EQ((*out)[1].as<aspn23_eigen::MeasurementVelocity>()->get_time_of_validity().get_elapsed_nsec(), 40 * s);
  EXPECT_EQ(vel->get_time_of_validity().get_elapsed_nsec(), 10 * s);
}
