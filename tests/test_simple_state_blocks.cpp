// Ports of test_fogm_block.py, test_constant_block.py, test_clock_bias_block.py
#include <pntos/cobra/state_modeling/SimpleStateBlocks.hpp>

#include "test_support.hpp"

#include <cmath>
#include <numbers>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Timestamp;
using api::Vector;

namespace {

const api::GenXandP kNoGen = [](const std::vector<std::string>&) -> std::optional<api::EstimateWithCovariance> {
  return std::nullopt;
};
constexpr std::int64_t kSec = 1'000'000'000;

// ---------------------------------------------------------------------------- FogmBlock

TEST(FogmBlock, EmptyInit) {
  TestMediator med;
  EXPECT_THROW(cobra::FogmBlock("bk", &med, Vector(0), Vector(0)), std::invalid_argument);
}

TEST(FogmBlock, SingleInit) {
  TestMediator med;
  cobra::FogmBlock blk("bk", &med, vec({1.0}), vec({2.0}));
  auto dyn = blk.generate_dynamics(kNoGen, Timestamp(0), Timestamp(kSec));
  ASSERT_TRUE(dyn);
  EXPECT_EQ(dyn->Phi.rows(), 1);
  EXPECT_EQ(dyn->Qd.rows(), 1);
  EXPECT_EQ(blk.num_states(), 1u);
}

TEST(FogmBlock, DoubleInit) {
  TestMediator med;
  cobra::FogmBlock blk("bk", &med, vec({1.0, 2.0}), vec({3.0, 4.0}));
  auto dyn = blk.generate_dynamics(kNoGen, Timestamp(0), Timestamp(kSec));
  ASSERT_TRUE(dyn);
  EXPECT_EQ(dyn->Phi.rows(), 2);
  EXPECT_EQ(dyn->Qd.cols(), 2);
}

TEST(FogmBlock, BadSizes) {
  TestMediator med;
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0}), vec({3.0, 4.0})), std::invalid_argument);
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0, 2.0, 3.0}), vec({3.0, 4.0})), std::invalid_argument);
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0, 2.0}), vec({3.0})), std::invalid_argument);
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0, 2.0}), vec({3.0, 4.0, 5.0})), std::invalid_argument);
}

TEST(FogmBlock, NonPositiveTau) {
  TestMediator med;
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0, 2.0}), vec({-3.0, 4.0})), std::invalid_argument);
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0, 2.0}), vec({-3.0, -4.0})), std::invalid_argument);
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0, 2.0}), vec({0.0, 4.0})), std::invalid_argument);
  EXPECT_THROW(cobra::FogmBlock("bk", &med, vec({1.0, 2.0}), vec({0.0, 0.0})), std::invalid_argument);
}

TEST(FogmBlock, NegativeSigmaAllowed) {
  TestMediator med;
  EXPECT_NO_THROW(cobra::FogmBlock("bk", &med, vec({-1.0, 2.0}), vec({3.0, 4.0})));
  EXPECT_NO_THROW(cobra::FogmBlock("bk", &med, vec({-1.0, -2.0}), vec({3.0, 4.0})));
}

TEST(FogmBlock, AuxIsIgnored) {
  TestMediator med;
  cobra::FogmBlock blk("bk", &med, vec({1.0}), vec({3.0}));
  blk.receive_aux_data({});
  blk.receive_aux_data({api::Message(make_position(kSec, 0.68, -1.47, 1005, Matrix::Identity(3, 3)), "garbage")});
  EXPECT_EQ(med.count(api::LoggingLevel::WARN), 2u);
}

TEST(FogmBlock, GenDyn) {
  TestMediator med;
  cobra::FogmBlock blk("bk", &med, vec({3.0}), vec({1.0}));
  auto dyn = blk.generate_dynamics(kNoGen, Timestamp(0), Timestamp(kSec));
  ASSERT_TRUE(dyn);
  Vector res = dyn->g(vec({1.0}));
  EXPECT_EQ(res.size(), 1);
  EXPECT_NEAR(res(0), 0.36787944, 1e-6);
}

TEST(FogmBlock, GenDynZeroDt) {
  TestMediator med;
  cobra::FogmBlock blk("bk", &med, vec({3.0}), vec({1.0}));
  auto dyn = blk.generate_dynamics(kNoGen, Timestamp(kSec), Timestamp(kSec));
  ASSERT_TRUE(dyn);
  Vector res = dyn->g(vec({3.0}));
  EXPECT_NEAR(res(0), 3.0, 1e-12);
  EXPECT_NEAR(dyn->Qd(0, 0), 0.0, 1e-12);
}

TEST(FogmBlock, GenDynNegativeDtNoError) {
  TestMediator med;
  cobra::FogmBlock blk("bk", &med, vec({1.0}), vec({3.0}));
  auto dyn = blk.generate_dynamics(kNoGen, Timestamp(kSec), Timestamp(0));
  ASSERT_TRUE(dyn);
  dyn->g(vec({0.0}));
}

TEST(FogmBlock, GenDynMulti) {
  TestMediator med;
  Vector sigmas = vec({1, 1, 1}), taus = vec({1, 1, 1});
  cobra::FogmBlock blk("bk", &med, sigmas, taus);
  Vector x = vec({0.5, 1.5, -2.5});
  auto dyn = blk.generate_dynamics(kNoGen, Timestamp(0), Timestamp(kSec));
  ASSERT_TRUE(dyn);
  Vector res = dyn->g(x);
  Vector expected = x.array() * (-1.0 / taus.array()).exp();
  EXPECT_ALLCLOSE(res, expected);
  EXPECT_ALLCLOSE(Vector(dyn->Phi * x), res);
  Matrix q_exp = Matrix::Zero(3, 3);
  for (int i = 0; i < 3; ++i) q_exp(i, i) = 2.0 * sigmas(i) * sigmas(i) / taus(i);
  Matrix alt = (q_exp + dyn->Phi * q_exp * dyn->Phi.transpose()) * 0.5;
  EXPECT_ALLCLOSE(dyn->Qd, alt);
}

// ---------------------------------------------------------------------- ConstantStateBlock

TEST(ConstantStateBlock, GenerateDynamics) {
  TestMediator med;
  cobra::ConstantStateBlock block("constant_block", &med, 3);
  Timestamp t1(0), t2(kSec), t3(101 * kSec);
  auto dyn = block.generate_dynamics(kNoGen, t1, t2);
  ASSERT_TRUE(dyn);
  Vector x = Vector::Random(3);
  EXPECT_EQ(dyn->g(x), x);
  EXPECT_EQ(dyn->Phi, Matrix::Identity(3, 3));
  EXPECT_EQ(dyn->Qd, Matrix::Zero(3, 3));
  auto dyn2 = block.generate_dynamics(kNoGen, t2, t3);
  EXPECT_EQ(dyn2->Phi, dyn->Phi);
  EXPECT_EQ(dyn2->Qd, dyn->Qd);
}

TEST(ConstantStateBlock, GenerateDynamicsWithNoise) {
  TestMediator med;
  cobra::ConstantStateBlock block("constant_block_with_noise", &med, 3, Matrix::Identity(3, 3));
  Timestamp t1(0), t2(kSec), t3(101 * kSec);
  auto dyn = block.generate_dynamics(kNoGen, t1, t2);
  EXPECT_EQ(dyn->Phi, Matrix::Identity(3, 3));
  EXPECT_EQ(dyn->Qd, Matrix::Identity(3, 3));
  auto dyn2 = block.generate_dynamics(kNoGen, t2, t3);
  EXPECT_EQ(dyn2->Qd, Matrix(Matrix::Identity(3, 3) * 100.0));
}

TEST(ConstantStateBlock, VariousSizes) {
  TestMediator med;
  for (std::size_t n = 1; n <= 91; n += 10) {
    cobra::ConstantStateBlock block("block", &med, n);
    auto dyn = block.generate_dynamics(kNoGen, Timestamp(0), Timestamp(kSec));
    Vector x = Vector::Random(static_cast<Eigen::Index>(n));
    EXPECT_EQ(dyn->g(x), x);
    EXPECT_EQ(block.num_states(), n);
  }
}

// ---------------------------------------------------------------------- ClockBiasStateBlock

TEST(ClockBiasStateBlock, TwoState) {
  TestMediator med;
  const double h_0 = 2e-20, h_neg2 = 4e-29;
  cobra::ClockBiasStateBlock block("clock_bias", &med, h_0, h_neg2);
  EXPECT_EQ(block.num_states(), 2u);
  Timestamp t_start(376'000'000), t_stop(1'242'300'000);
  const double dt = (t_stop.elapsed_nsec - t_start.elapsed_nsec) * 1e-9, dt2 = dt * dt;
  const double pi_sq = std::numbers::pi * std::numbers::pi;
  const double q00 = 0.5 * h_0 * dt + (2.0 / 3) * pi_sq * h_neg2 * dt * dt2;
  const double q01 = pi_sq * h_neg2 * dt2;
  const double q11 = 2 * pi_sq * h_neg2 * dt;
  auto dyn = block.generate_dynamics(kNoGen, t_start, t_stop);
  ASSERT_TRUE(dyn);
  Vector x = vec({0.0, 1.0});
  EXPECT_ALLCLOSE(dyn->g(x), vec({dt, 1.0}));
  EXPECT_TRUE(allclose(dyn->Phi, mat({{1, dt}, {0, 1}}), 1e-25, 0.0));
  EXPECT_TRUE(allclose(dyn->Qd, mat({{q00, q01}, {q01, q11}}), 1e-25, 0.0));
}

TEST(ClockBiasStateBlock, ThreeState) {
  TestMediator med;
  const double h_0 = 2e-20, h_neg2 = 4e-29, q3 = 0.0;
  cobra::ClockBiasStateBlock block("clock_bias", &med, h_0, h_neg2, q3);
  EXPECT_EQ(block.num_states(), 3u);
  Timestamp t_start(376'000'000), t_stop(1'242'300'000);
  const double dt = (t_stop.elapsed_nsec - t_start.elapsed_nsec) * 1e-9, dt2 = dt * dt;
  const double pi_sq = std::numbers::pi * std::numbers::pi;
  const double ct = pi_sq * h_neg2 * dt2;
  Matrix expected_Qd = mat({{0.5 * h_0 * dt + (2.0 / 3) * pi_sq * h_neg2 * dt * dt2, ct, 0},
                            {ct, 2 * pi_sq * h_neg2 * dt, 0},
                            {0, 0, 0}});
  auto dyn = block.generate_dynamics(kNoGen, t_start, t_stop);
  Vector x = vec({0.0, 1.0, 0.1});
  EXPECT_ALLCLOSE(dyn->g(x), vec({dt + 0.1 * 0.5 * dt2, 1.0 + 0.1 * dt, 0.1}));
  EXPECT_TRUE(allclose(dyn->Phi, mat({{1, dt, 0.5 * dt2}, {0, 1, dt}, {0, 0, 1}}), 1e-25, 0.0));
  EXPECT_TRUE(allclose(dyn->Qd, expected_Qd, 1e-25, 0.0));
}

TEST(ClockBiasStateBlock, ThreeStateWithQ3) {
  TestMediator med;
  const double h_0 = 2e-20, h_neg2 = 4e-29, q3 = 1e-30;
  cobra::ClockBiasStateBlock block("clock_bias", &med, h_0, h_neg2, q3);
  const double dt = 0.8663, dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
  auto dyn = block.generate_dynamics(kNoGen, Timestamp(0), Timestamp::from_seconds(dt));
  Matrix q_three = mat({{q3 * dt5 / 20, q3 * dt4 / 8, q3 * dt3 / 6},
                        {q3 * dt4 / 8, q3 * dt3 / 3, q3 * dt2 / 2},
                        {q3 * dt3 / 6, q3 * dt2 / 2, q3 * dt}});
  Matrix expected = cobra::ClockBiasStateBlock::hwang_brown_q(h_0, h_neg2, dt) + q_three;
  EXPECT_TRUE(allclose(dyn->Qd, expected, 1e-9, 0.0));
}

}  // namespace
