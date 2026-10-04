// StandardFusionEngine tests (no direct Python equivalent; Cobra covers the engine through the
// orchestration/integration tests). These check bookkeeping, VSB routing, peek_ahead isolation and
// the diagnostics save against the EKF strategy and the simple state blocks.
#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/fusion/StandardFusionPlugin.hpp>
#include <pntos/cobra/state_modeling/SimpleStateBlocks.hpp>
#include <pntos/cobra/state_modeling/VirtualStateBlocks.hpp>

#include "test_support.hpp"

using namespace pntos;
using namespace pntos::test;
using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::Matrix;
using api::Message;
using api::Timestamp;
using api::Vector;

namespace {
constexpr std::int64_t kSec = 1'000'000'000;

/// Measures the states of its (single real or virtual) block directly: z = x + v.
class DirectProcessor final : public api::StandardMeasurementProcessor {
 public:
  DirectProcessor(std::string label, std::vector<std::string> blocks, Vector z, double r)
      : label_(std::move(label)), blocks_(std::move(blocks)), z_(std::move(z)), r_(r) {}
  const std::string& label() const override { return label_; }
  const std::vector<std::string>& state_block_labels() const override { return blocks_; }
  void receive_aux_data(const api::AuxData& aux) override { aux_count_ += aux.size(); }
  std::optional<api::StandardMeasurementModel> generate_model(const Message&, const api::GenXandP& gen) override {
    auto xp = gen(blocks_);
    if (!xp) return std::nullopt;
    seen_n_ = xp->estimate.size();
    const Eigen::Index n = z_.size();
    api::StandardMeasurementModel m;
    m.z = z_;
    m.h = [](const Vector& x) { return x; };
    m.H = Matrix::Identity(n, n);
    m.R = Matrix::Identity(n, n) * r_;
    return m;
  }
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<DirectProcessor>(*this);
  }
  std::size_t aux_count_ = 0;
  Eigen::Index seen_n_ = 0;

 private:
  std::string label_;
  std::vector<std::string> blocks_;
  Vector z_;
  double r_;
};

EstimateWithCovariance ewc(std::initializer_list<double> x, double p) {
  Vector v = vec(x);
  return {EstimateWithCovarianceType::EWC_GENERIC, v, Matrix::Identity(v.size(), v.size()) * p};
}

class FusionEngineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    registry_plugin = std::make_unique<cobra::StandardRegistryPlugin>(
        "registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{});
    registry_plugin->init_plugin(std::nullopt, &med);
    med.set_registry(registry_plugin->new_registry());
    engine = std::make_unique<cobra::StandardFusionEngine>(&med);
    auto strat = std::make_unique<cobra::EkfFusionStrategy>(&med);
    strat->set_joseph_form(false);
    engine->set_strategy(std::move(strat));
  }
  std::unique_ptr<api::StandardStateBlock> fogm(const std::string& label, int n) {
    return std::make_unique<cobra::FogmBlock>(label, &med, Vector::Constant(n, 1.0), Vector::Constant(n, 100.0));
  }
  std::unique_ptr<api::StandardStateBlock> constant(const std::string& label, int n) {
    return std::make_unique<cobra::ConstantStateBlock>(label, &med, static_cast<std::size_t>(n), std::nullopt);
  }
  TestMediator med;
  std::unique_ptr<cobra::StandardRegistryPlugin> registry_plugin;
  std::unique_ptr<cobra::StandardFusionEngine> engine;
};

TEST_F(FusionEngineTest, AddGetSetRemoveBlocks) {
  EXPECT_FALSE(engine->state_block_labels().has_value());
  engine->add_state_block(fogm("a", 3), ewc({1, 2, 3}, 1.0));
  engine->add_state_block(constant("b", 2), ewc({4, 5}, 2.0));
  api::CrossCovariances cc{{"a", "missing"}, {Matrix::Constant(3, 1, 0.5), Matrix::Zero(1, 1)}};
  engine->add_state_block(constant("c", 1), ewc({6}, 3.0), cc);
  EXPECT_EQ(engine->num_states(), 6u);
  EXPECT_EQ(*engine->state_block_labels(), (std::vector<std::string>{"a", "b", "c"}));
  EXPECT_EQ(med.count(api::LoggingLevel::WARN), 2u);  // "No state blocks" + missing cross-cov label

  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("b"), vec({4, 5}));
  EXPECT_ALLCLOSE(*engine->get_state_block_covariance("b"), Matrix(Matrix::Identity(2, 2) * 2.0));
  EXPECT_ALLCLOSE(*engine->get_state_block_cross_covariance("a", "c"), Matrix::Constant(3, 1, 0.5));
  EXPECT_ALLCLOSE(*engine->get_state_block_cross_covariance("c", "a"), Matrix::Constant(1, 3, 0.5));
  EXPECT_FALSE(med.has_error());
  EXPECT_FALSE(engine->get_state_block_estimate("nope").has_value());
  EXPECT_TRUE(med.has_error());  // unknown label is an error, as in Python

  engine->set_state_block_estimate("b", vec({40, 50}));
  engine->set_state_block_covariance("b", Matrix::Identity(2, 2) * 7.0);
  engine->set_state_block_cross_covariance("a", "b", Matrix::Constant(3, 2, 0.25));
  engine->set_state_block_estimate("b", vec({1, 2, 3}));  // size mismatch -> warn
  EXPECT_THROW(engine->set_state_block_covariance("b", Matrix::Identity(3, 3)), std::invalid_argument);
  auto xp = engine->generate_x_and_p({"b", "a"});
  ASSERT_TRUE(xp);
  EXPECT_ALLCLOSE(xp->estimate, vec({40, 50, 1, 2, 3}));
  Matrix exp = Matrix::Zero(5, 5);
  exp.block<2, 2>(0, 0) = Matrix::Identity(2, 2) * 7.0;
  exp.block<3, 3>(2, 2) = Matrix::Identity(3, 3);
  exp.block<2, 3>(0, 2) = Matrix::Constant(2, 3, 0.25);
  exp.block<3, 2>(2, 0) = Matrix::Constant(3, 2, 0.25);
  EXPECT_ALLCLOSE(xp->covariance, exp);

  engine->remove_state_block("a");
  EXPECT_EQ(engine->num_states(), 3u);
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("b"), vec({40, 50}));
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("c"), vec({6}));
  EXPECT_ALLCLOSE(*engine->strategy()->estimate(), vec({40, 50, 6}));
  engine->remove_state_block("a");  // warn, no throw
  EXPECT_FALSE(engine->generate_x_and_p({}).has_value());
  EXPECT_FALSE(engine->generate_x_and_p({"zzz"}).has_value());
}

TEST_F(FusionEngineTest, RejectsNonGenericEwc) {
  EstimateWithCovariance e = ewc({1, 2}, 1.0);
  e.type = EstimateWithCovarianceType::EWC_ATTITUDE_QUAT;
  engine->add_state_block(constant("a", 2), e);
  EXPECT_EQ(engine->num_states(), 0u);
  EXPECT_TRUE(med.has_error());
}

TEST_F(FusionEngineTest, PropagateAssemblesBlockDiagonalDynamics) {
  engine->add_state_block(fogm("f", 2), ewc({1, 1}, 1.0));
  engine->add_state_block(constant("c", 1), ewc({3}, 1.0));
  engine->propagate(Timestamp(-1));  // backwards -> warn, no change
  engine->propagate(Timestamp(0));   // same time -> no-op
  EXPECT_ALLCLOSE(*engine->strategy()->estimate(), vec({1, 1, 3}));
  engine->propagate(Timestamp(kSec));
  EXPECT_EQ(engine->time().elapsed_nsec, kSec);
  const double phi = std::exp(-1.0 / 100.0);
  EXPECT_ALLCLOSE(*engine->strategy()->estimate(), vec({phi, phi, 3}));
  Matrix P = *engine->strategy()->covariance();
  EXPECT_NEAR(P(2, 2), 1.0, 1e-12);                      // constant block untouched
  const double q = 2.0 / 100.0;  // 2 sigma^2 / tau
  const double qd = 0.5 * (phi * phi * q + q) * 1.0;  // FogmBlock's trapezoidal Qd
  EXPECT_NEAR(P(0, 0), phi * phi + qd, 1e-12);
  EXPECT_NEAR(P(0, 2), 0.0, 1e-12);
}

TEST_F(FusionEngineTest, UpdateThroughRealAndVirtualBlocks) {
  engine->add_state_block(constant("c", 3), ewc({0, 0, 0}, 4.0));
  engine->add_virtual_state_block(
      std::make_unique<cobra::StateExtractor>(&med, "c", "first_two", 3, std::vector<int>{0, 1}));
  EXPECT_TRUE(engine->has_virtual_state_block("first_two"));
  EXPECT_EQ(*engine->virtual_state_block_target_labels(), std::vector<std::string>{"first_two"});
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("first_two"), vec({0, 0}));
  EXPECT_ALLCLOSE(*engine->get_state_block_covariance("first_two"), Matrix(Matrix::Identity(2, 2) * 4.0));
  EXPECT_ALLCLOSE(*engine->get_state_block_cross_covariance("first_two", "c"),
                  Matrix(Matrix::Identity(3, 3).topRows<2>() * 4.0));

  engine->add_measurement_processor(std::make_unique<DirectProcessor>("virt", std::vector<std::string>{"first_two"},
                                                                      vec({2, 2}), 4.0));
  engine->add_measurement_processor(
      std::make_unique<DirectProcessor>("real", std::vector<std::string>{"c"}, vec({1, 1, 1}), 4.0));
  engine->add_measurement_processor(
      std::make_unique<DirectProcessor>("real", std::vector<std::string>{"c"}, vec({9, 9, 9}), 1.0));  // dup
  EXPECT_EQ(engine->measurement_processor_labels()->size(), 2u);

  Message m(make_position(kSec, 0, 0, 0, Matrix::Identity(3, 3)), "src");
  engine->update("virt", m);
  EXPECT_EQ(engine->time().elapsed_nsec, kSec);
  // Equal prior (4) and R (4): posterior mean halfway to z for the two measured states; third untouched.
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("c"), vec({1, 1, 0}));
  Matrix P = *engine->strategy()->covariance();
  EXPECT_NEAR(P(0, 0), 2.0, 1e-12);
  EXPECT_NEAR(P(2, 2), 4.0, 1e-12);

  engine->update("real", m);
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("c"), vec({1, 1, 0.5}));
  engine->update("missing", m);
  EXPECT_TRUE(med.has_error());

  engine->remove_measurement_processor("virt");
  EXPECT_EQ(engine->measurement_processor_labels()->size(), 1u);
  engine->remove_virtual_state_block("first_two");
  EXPECT_FALSE(engine->has_virtual_state_block("first_two"));
  EXPECT_FALSE(engine->virtual_state_block_target_labels().has_value());
}

TEST_F(FusionEngineTest, InnovationStatisticIsSideEffectFree) {
  engine->add_state_block(constant("c", 3), ewc({0, 0, 0}, 4.0));
  engine->add_measurement_processor(
      std::make_unique<DirectProcessor>("real", std::vector<std::string>{"c"}, vec({2, 2, 2}), 4.0));
  Message m(make_position(kSec, 0, 0, 0, Matrix::Identity(3, 3)), "src");
  auto st = engine->innovation_statistic("real", m);
  ASSERT_TRUE(st);
  EXPECT_EQ(st->dof, 3);
  EXPECT_NEAR(st->chi2, 3 * (2.0 * 2.0) / (4.0 + 4.0), 1e-12);  // nu = 2 per axis, S = P + R = 8
  EXPECT_ALLCLOSE(st->innovation, vec({2, 2, 2}));
  // nothing moved: time, estimate and covariance unchanged, no registry gating group
  EXPECT_EQ(engine->time().elapsed_nsec, 0);
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("c"), vec({0, 0, 0}));
  EXPECT_NEAR((*engine->strategy()->covariance())(0, 0), 4.0, 1e-12);
  EXPECT_FALSE(med.registry().has_group("fusion/gating"));
  EXPECT_FALSE(engine->innovation_statistic("missing", m));
  // a quiet clone neither reports gating to the registry nor loses the gate itself
  engine->set_innovation_gate("real", 0.5);
  auto quiet = engine->clone();
  auto* q = dynamic_cast<cobra::StandardFusionEngine*>(quiet.get());
  ASSERT_NE(q, nullptr);
  q->set_registry_reporting(false);
  q->update("real", m);
  EXPECT_FALSE(med.registry().has_group("fusion/gating"));
  ASSERT_TRUE(q->gate_stats("real"));
  EXPECT_EQ(q->gate_stats("real")->accepted + q->gate_stats("real")->rejected, 1u);
  EXPECT_EQ(engine->time().elapsed_nsec, 0);
}

TEST_F(FusionEngineTest, AuxDataRouting) {
  engine->add_state_block(constant("c", 1), ewc({0}, 1.0));
  auto proc = std::make_unique<DirectProcessor>("p", std::vector<std::string>{"c"}, vec({1}), 1.0);
  auto* raw = proc.get();
  engine->add_measurement_processor(std::move(proc));
  engine->give_measurement_processor_aux_data("p", {std::nullopt, std::nullopt});
  EXPECT_EQ(raw->aux_count_, 2u);
  engine->give_measurement_processor_aux_data("zzz", {});
  engine->give_state_block_aux_data("zzz", {});
  engine->give_state_block_aux_data("c", {});
  EXPECT_EQ(med.count(api::LoggingLevel::WARN), 2u);
}

TEST_F(FusionEngineTest, PeekAheadDoesNotDisturbTheEngine) {
  engine->add_state_block(fogm("f", 1), ewc({1}, 1.0));
  engine->add_state_block(constant("c", 1), ewc({3}, 1.0));
  auto proc = std::make_unique<DirectProcessor>("p", std::vector<std::string>{"c"}, vec({1}), 1.0);
  engine->add_measurement_processor(std::move(proc));

  EXPECT_FALSE(engine->peek_ahead(Timestamp(-5), {"f"}).has_value());
  EXPECT_FALSE(engine->peek_ahead(Timestamp(kSec), {}).has_value());
  EXPECT_FALSE(engine->peek_ahead(Timestamp(kSec), {"nope"}).has_value());
  auto now = engine->peek_ahead(Timestamp(0), {"f", "c"});
  ASSERT_TRUE(now);
  EXPECT_ALLCLOSE(now->estimate, vec({1, 3}));

  auto future = engine->peek_ahead(Timestamp(10 * kSec), {"f", "c"});
  ASSERT_TRUE(future);
  EXPECT_NEAR(future->estimate(0), std::exp(-10.0 / 100.0), 1e-12);
  EXPECT_DOUBLE_EQ(future->estimate(1), 3.0);
  // engine unchanged
  EXPECT_EQ(engine->time().elapsed_nsec, 0);
  EXPECT_ALLCLOSE(*engine->strategy()->estimate(), vec({1, 3}));
  EXPECT_EQ(engine->measurement_processor_labels()->size(), 1u);

  // clone is a deep copy
  auto copy = engine->clone();
  copy->propagate(Timestamp(kSec));
  copy->set_state_block_estimate("c", vec({99}));
  copy->remove_measurement_processor("p");
  EXPECT_EQ(engine->time().elapsed_nsec, 0);
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("c"), vec({3}));
  EXPECT_TRUE(engine->measurement_processor_labels().has_value());
}

TEST_F(FusionEngineTest, SavesDiagnosticsToRegistry) {
  auto saving = std::make_unique<cobra::StandardFusionEngine>(&med, true, true);
  saving->set_strategy(std::make_unique<cobra::EkfFusionStrategy>(&med));
  saving->add_state_block(fogm("f", 2), ewc({1, 1}, 4.0));
  saving->add_state_block(constant("c", 1), ewc({3}, 9.0));
  saving->propagate(Timestamp(kSec));
  auto kv = med.registry().batch("diagnostics");
  auto labels = kv->get_value<api::StringArray>("state_labels");
  ASSERT_TRUE(labels);
  EXPECT_EQ(*labels, (api::StringArray{"f_state0", "f_state1", "c_state0"}));
  EXPECT_EQ(*kv->get_value<std::int64_t>("time"), kSec);
  auto sigma = kv->get_value<Matrix>("sigma");
  ASSERT_TRUE(sigma);
  EXPECT_NEAR((*sigma)(2, 0), 3.0, 1e-12);
  EXPECT_EQ(kv->get_value<Matrix>("estimate")->rows(), 3);
}

TEST_F(FusionEngineTest, PluginReadsConfig) {
  cobra::StandardFusionPlugin plugin("fusion");
  plugin.init_plugin(std::nullopt, &med);
  EXPECT_TRUE(plugin.is_fusion_type_supported(api::FusionType::STANDARD));
  EXPECT_FALSE(plugin.is_fusion_type_supported(api::FusionType::SAMPLED));
  EXPECT_EQ(plugin.new_fusion_engine(api::FusionType::SAMPLED), nullptr);
  auto e = plugin.new_fusion_engine(api::FusionType::STANDARD);  // no config group -> defaults
  ASSERT_TRUE(e);
  cobra::FusionEngineConfig cfg;
  cfg.save_x_and_p_after_update = true;
  cfg.to_registry(med);
  auto back = cobra::FusionEngineConfig::from_registry(med);
  ASSERT_TRUE(back);
  EXPECT_FALSE(back->save_x_and_p_after_prop);
  EXPECT_TRUE(back->save_x_and_p_after_update);
  EXPECT_TRUE(plugin.new_fusion_engine(api::FusionType::STANDARD));
}

}  // namespace

TEST_F(FusionEngineTest, InnovationGateRejectsOutliers) {
  // chi-square quantiles (Wilson-Hilferty): exact 10.83 (p=0.999, dof 1), 7.81 (0.95, 3), 16.27 (0.999, 3)
  EXPECT_NEAR(cobra::StandardFusionEngine::chi2_quantile(0.999, 1), 10.83, 0.5);
  EXPECT_NEAR(cobra::StandardFusionEngine::chi2_quantile(0.95, 3), 7.81, 0.3);
  EXPECT_NEAR(cobra::StandardFusionEngine::chi2_quantile(0.999, 3), 16.27, 0.5);

  engine->add_state_block(std::make_unique<cobra::ConstantStateBlock>("c", &med, std::size_t{3}, std::nullopt), ewc({0, 0, 0}, 4.0), std::nullopt);
  engine->add_measurement_processor(std::make_unique<DirectProcessor>("far", std::vector<std::string>{"c"}, vec({100, 100, 100}), 1.0));
  engine->add_measurement_processor(std::make_unique<DirectProcessor>("near", std::vector<std::string>{"c"}, vec({1, 1, 1}), 1.0));
  engine->set_innovation_gate("far", 0.999);
  engine->set_innovation_gate("near", 0.999);
  Message m(make_position(kSec, 0, 0, 0, Matrix::Identity(3, 3)), "src");
  engine->update("far", m);  // chi2 = 3 * 100^2 / 5 = 6000 >> 16.3: rejected, state untouched
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("c"), vec({0, 0, 0}));
  auto st = engine->gate_stats("far");
  ASSERT_TRUE(st);
  EXPECT_EQ(st->rejected, 1u);
  EXPECT_EQ(st->accepted, 0u);
  EXPECT_NEAR(st->last_chi2, 6000.0, 1e-6);
  EXPECT_EQ(med.count(api::LoggingLevel::WARN), 1u);
  engine->update("near", m);  // chi2 = 3 / 5 = 0.6: accepted, posterior = 4/5 of z
  EXPECT_ALLCLOSE(*engine->get_state_block_estimate("c"), vec({0.8, 0.8, 0.8}));
  EXPECT_EQ(engine->gate_stats("near")->accepted, 1u);
  // counters in the registry
  auto kv = med.registry().batch("fusion/gating");
  EXPECT_EQ(kv->get_value<std::int64_t>("far_rejected"), 1);
  EXPECT_EQ(kv->get_value<std::int64_t>("near_accepted"), 1);
  // removing the gate lets the outlier through (posterior P = 0.8 after the near update: K = 0.8 / 1.8)
  engine->set_innovation_gate("far", 0.0);
  engine->update("far", m);
  EXPECT_NEAR((*engine->get_state_block_estimate("c"))(0), 0.8 + 0.8 / 1.8 * (100 - 0.8), 1e-9);
  // default gate from the config
  cobra::StandardFusionEngine gated(&med, false, false, 0.99);
  gated.set_strategy(std::make_unique<cobra::EkfFusionStrategy>(&med));
  gated.add_state_block(std::make_unique<cobra::ConstantStateBlock>("c", &med, std::size_t{3}, std::nullopt), ewc({0, 0, 0}, 4.0), std::nullopt);
  gated.add_measurement_processor(std::make_unique<DirectProcessor>("far", std::vector<std::string>{"c"}, vec({100, 100, 100}), 1.0));
  gated.update("far", m);
  EXPECT_ALLCLOSE(*gated.get_state_block_estimate("c"), vec({0, 0, 0}));
}
