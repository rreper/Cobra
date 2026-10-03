// Port of pntos-cobra/tests/test_standard_ekf_fusion_strategy.py
#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>

#include "test_support.hpp"

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Vector;
using cobra::EkfFusionStrategy;

namespace {

class EkfTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plugin = std::make_unique<cobra::EkfFusionStrategyPlugin>("plg");
    plugin->init_plugin(std::nullopt, &mediator);
    auto s = plugin->new_fusion_strategy(api::FusionType::STANDARD);
    strat.reset(dynamic_cast<EkfFusionStrategy*>(s.release()));
    ASSERT_TRUE(strat);
    strat->set_joseph_form(false);  // the Python tests assert against the simple form
  }

  /// pop_strat(st, ns, v1): x = arange(v1, v1+ns), P = diag(x^2)
  void pop(int ns, double v1) {
    Vector x(ns);
    for (int i = 0; i < ns; ++i) x(i) = v1 + i;
    Matrix p = Matrix::Zero(ns, ns);
    for (int i = 0; i < ns; ++i) p(i, i) = x(i) * x(i);
    strat->add_states(x, p);
  }

  TestMediator mediator;
  std::unique_ptr<cobra::EkfFusionStrategyPlugin> plugin;
  std::unique_ptr<EkfFusionStrategy> strat;
};

TEST_F(EkfTest, GetStrategyFail) {
  EXPECT_EQ(plugin->new_fusion_strategy(api::FusionType::SAMPLED), nullptr);
  EXPECT_TRUE(mediator.has_error());
}

TEST_F(EkfTest, Shutdown) { plugin->shutdown_plugin(); }

TEST_F(EkfTest, GetNoStates) {
  EXPECT_EQ(strat->num_states(), 0u);
  EXPECT_FALSE(strat->estimate().has_value());
  EXPECT_FALSE(strat->covariance().has_value());
}

TEST_F(EkfTest, AddOneState) {
  pop(1, 0);
  EXPECT_EQ(strat->num_states(), 1u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({0}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{0}}));
}

TEST_F(EkfTest, AddTwoState) {
  pop(2, 0);
  EXPECT_EQ(strat->num_states(), 2u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({0, 1}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{0, 0}, {0, 1}}));
}

TEST_F(EkfTest, AddStatesTwice) {
  pop(2, 0);
  pop(1, 5);
  EXPECT_EQ(strat->num_states(), 3u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({0, 1, 5}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{0, 0, 0}, {0, 1, 0}, {0, 0, 25}}));
}

TEST_F(EkfTest, AddThenRemoveOnce) {
  pop(1, 0);
  EXPECT_EQ(strat->num_states(), 1u);
  strat->remove_states(0, 1);
  EXPECT_EQ(strat->num_states(), 0u);
  EXPECT_FALSE(strat->estimate().has_value());
  EXPECT_FALSE(strat->covariance().has_value());
}

TEST_F(EkfTest, AddThenRemoveMulti) {
  pop(2, 1);
  pop(2, 4);
  EXPECT_EQ(strat->num_states(), 4u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 2, 4, 5}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 0, 0, 0}, {0, 4, 0, 0}, {0, 0, 16, 0}, {0, 0, 0, 25}}));
  strat->remove_states(1, 2);
  EXPECT_EQ(strat->num_states(), 2u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 5}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 0}, {0, 25}}));
  Vector x = vec({9, 8, 7});
  Matrix p = mat({{1.1, 1.2, 1.3}, {1.2, 3.3, 9.7}, {1.3, 9.7, 2.2}});
  strat->add_states(x, p);
  EXPECT_EQ(strat->num_states(), 5u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 5, 9, 8, 7}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 0, 0, 0, 0},
                                             {0, 25, 0, 0, 0},
                                             {0, 0, 1.1, 1.2, 1.3},
                                             {0, 0, 1.2, 3.3, 9.7},
                                             {0, 0, 1.3, 9.7, 2.2}}));
  strat->remove_states(1, 3);
  EXPECT_EQ(strat->num_states(), 2u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 7}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 0}, {0, 2.2}}));
}

TEST_F(EkfTest, AddStatesMismatchSize) {
  EXPECT_THROW(strat->add_states(Vector::Zero(1), Matrix::Zero(2, 2)), std::invalid_argument);
  EXPECT_THROW(strat->add_states(Vector::Zero(1), Matrix::Zero(2, 1)), std::invalid_argument);
  EXPECT_THROW(strat->add_states(Vector::Zero(1), Matrix::Zero(1, 2)), std::invalid_argument);
}

TEST_F(EkfTest, AddStatesWithInitialCross) {
  // No existing states: nothing to form a cross covariance with
  EXPECT_THROW(strat->add_states(Vector::Zero(1), Matrix::Zero(1, 1), Matrix::Zero(1, 1)), std::invalid_argument);
}

TEST_F(EkfTest, AddStatesWithCross) {
  pop(2, 1);
  strat->add_states(vec({3.0}), mat({{9.0}}), mat({{2.2}, {3.3}}));
  EXPECT_EQ(strat->num_states(), 3u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 2, 3}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 0, 2.2}, {0, 4, 3.3}, {2.2, 3.3, 9}}));
}

TEST_F(EkfTest, AddStatesWithCrossTransposed) {
  pop(2, 1);
  EXPECT_THROW(strat->add_states(vec({3.0}), mat({{9.0}}), mat({{2.2, 3.3}})), std::invalid_argument);
}

TEST_F(EkfTest, AddStatesWithCrossBadShape) {
  pop(2, 1);
  EXPECT_THROW(strat->add_states(vec({3.0}), mat({{9.0}}), mat({{2.2}, {3.3}, {1.0}})), std::invalid_argument);
  EXPECT_THROW(strat->add_states(vec({3.0}), mat({{9.0}}), mat({{1.0}})), std::invalid_argument);
}

TEST_F(EkfTest, RemoveNoStates) {
  strat->remove_states(0, 0);
  strat->remove_states(0, 1);
  pop(2, 1);
  strat->remove_states(0, 0);
  EXPECT_EQ(strat->num_states(), 2u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 2}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 0}, {0, 4}}));
}

TEST_F(EkfTest, RemoveBadArgs) {
  pop(2, 1);
  strat->remove_states(2, 1);
  EXPECT_EQ(strat->num_states(), 2u);
  strat->remove_states(0, 3);
  EXPECT_EQ(strat->num_states(), 2u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 2}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 0}, {0, 4}}));
  EXPECT_TRUE(mediator.has_error());
}

TEST_F(EkfTest, AddRemoveStatesWithCross) {
  pop(2, 1);
  strat->add_states(vec({3.0}), mat({{9.0}}), mat({{2.2}, {3.3}}));
  strat->remove_states(1, 1);
  EXPECT_EQ(strat->num_states(), 2u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 3}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1, 2.2}, {2.2, 9}}));
  strat->remove_states(0, 1);
  EXPECT_EQ(strat->num_states(), 1u);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({3}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{9}}));
}

TEST_F(EkfTest, SetEstimateSliceFull) {
  pop(5, 0);
  Vector a = vec({9, 3, 4, 1, -2});
  strat->set_estimate_slice(a, 0);
  EXPECT_ALLCLOSE(*strat->estimate(), a);
}

TEST_F(EkfTest, SetEstimateSlicePart) {
  pop(5, 0);
  Vector a = vec({9, 3, 4, 1, -2});
  strat->set_estimate_slice(a.tail(4), 1);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({0, 3, 4, 1, -2}));
  strat->set_estimate_slice(a.tail(2), 0);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, -2, 4, 1, -2}));
  strat->set_estimate_slice(a.segment(3, 1), 2);
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, -2, 1, 1, -2}));
}

TEST_F(EkfTest, SetEstimateSliceBad) {
  pop(5, 0);
  EXPECT_THROW(strat->set_estimate_slice(Vector::Zero(17), 1), std::invalid_argument);
  EXPECT_THROW(strat->set_estimate_slice(vec({9, 3, 4, 1, -2}), 5), std::invalid_argument);  // oob
  EXPECT_THROW(strat->set_estimate_slice(vec({9, 3, 4, 1, -2}), 3), std::invalid_argument);  // wrap
}

TEST_F(EkfTest, SetCovSliceFull) {
  pop(3, 0);
  Matrix p = mat({{1.1, 1.2, 1.3}, {1.2, 3.3, 9.7}, {1.3, 9.7, 2.2}});
  strat->set_covariance_slice(p, 0, 0);
  EXPECT_ALLCLOSE(*strat->covariance(), p);
}

TEST_F(EkfTest, SetCovSlicePartDiag) {
  pop(3, 0);
  Matrix p = mat({{1.1, 1.2}, {1.2, 3.3}});
  strat->set_covariance_slice(p, 1, 1);
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{0, 0, 0}, {0, 1.1, 1.2}, {0, 1.2, 3.3}}));
  strat->set_covariance_slice(p, 0, 0);
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1.1, 1.2, 0}, {1.2, 3.3, 1.2}, {0, 1.2, 3.3}}));
}

TEST_F(EkfTest, SetCovSlicePartOffDiag) {
  pop(3, 0);
  Matrix p = mat({{1.1, 1.2}, {1.2, 3.3}});
  strat->set_covariance_slice(p, 0, 1);
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{0, 1.1, 1.2}, {0, 1.2, 3.3}, {0, 0, 4}}));
  strat->set_covariance_slice(p, 1, 0);
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{0, 1.1, 1.2}, {1.1, 1.2, 3.3}, {1.2, 3.3, 4}}));
}

TEST_F(EkfTest, SetCovSliceBadSize) {
  pop(2, 0);
  Matrix p = mat({{1.1, 1.2}, {1.2, 3.3}});
  EXPECT_THROW(strat->set_covariance_slice(p, 0, 1), std::invalid_argument);
  EXPECT_THROW(strat->set_covariance_slice(p, 1, 0), std::invalid_argument);
  EXPECT_THROW(strat->set_covariance_slice(p, 1, 1), std::invalid_argument);
  Matrix q = mat({{1.1, 1.2, 2.2}, {1.2, 3.3, 1.1}});
  EXPECT_THROW(strat->set_covariance_slice(q, 0, 0), std::invalid_argument);
  EXPECT_THROW(strat->set_covariance_slice(q.transpose(), 0, 0), std::invalid_argument);
}

TEST_F(EkfTest, PropBadModelG) {
  pop(2, 0);
  api::StandardDynamicsModel m{[](const Vector& x) { return Vector(x.tail(1)); }, Matrix::Identity(2, 2),
                               Matrix::Zero(2, 2)};
  EXPECT_THROW(strat->propagate(m), std::invalid_argument);
}

TEST_F(EkfTest, PropBadModelPhiQd) {
  pop(2, 0);
  auto g = [](const Vector& x) { return x; };
  for (auto [r, c] : std::vector<std::pair<int, int>>{{1, 1}, {3, 3}, {2, 1}, {1, 2}, {2, 3}, {3, 2}}) {
    EXPECT_THROW(strat->propagate({g, Matrix::Zero(r, c), Matrix::Zero(2, 2)}), std::invalid_argument);
    EXPECT_THROW(strat->propagate({g, Matrix::Identity(2, 2), Matrix::Zero(r, c)}), std::invalid_argument);
  }
}

TEST_F(EkfTest, UpdateBadModelH) {
  pop(3, 0);
  auto h_bad = [](const Vector&) { return vec({0, 0}); };
  EXPECT_THROW(strat->update({Vector::Zero(1), h_bad, Matrix::Zero(1, 3), Matrix::Identity(1, 1)}),
               std::invalid_argument);
  EXPECT_THROW(strat->update({Vector::Zero(3), h_bad, Matrix::Zero(3, 3), Matrix::Identity(3, 3)}),
               std::invalid_argument);
  EXPECT_THROW(strat->update({Vector::Zero(2), h_bad, Matrix::Zero(2, 2), Matrix::Identity(2, 2)}),
               std::invalid_argument);
}

TEST_F(EkfTest, Propagate) {
  pop(2, 0);
  auto g = [](const Vector& x) {
    Vector out = x;
    out(0) += x(1);
    out(1) *= 2;
    return out;
  };
  strat->propagate({g, mat({{1, 1}, {0, 2}}), mat({{0.1, 0.2}, {0.2, 3.0}})});
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1, 2}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1.1, 2.2}, {2.2, 7}}));
}

TEST_F(EkfTest, Symmetrize) {
  Vector x = vec({2.0, 4.4});
  Matrix p = mat({{1.0, 2.3}, {1.7, 4.0}});
  strat->add_states(x, p);
  strat->propagate({[](const Vector& v) { return v; }, Matrix::Identity(2, 2), Matrix::Zero(2, 2)});
  EXPECT_ALLCLOSE(*strat->estimate(), x);
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{1.0, 2.0}, {2.0, 4.0}}));
}

TEST_F(EkfTest, Update) {
  pop(1, 1);
  strat->update({vec({2}), [](const Vector& x) { return x; }, mat({{1}}), mat({{3}})});
  EXPECT_ALLCLOSE(*strat->estimate(), vec({1.25}));
  EXPECT_ALLCLOSE(*strat->covariance(), mat({{0.75}}));
}

TEST_F(EkfTest, UpdateJosephMatchesSimpleForm) {
  pop(3, 1);
  strat->set_covariance_slice(mat({{4, 1, 0}, {1, 9, 2}, {0, 2, 16}}), 0);
  auto simple = std::unique_ptr<EkfFusionStrategy>(dynamic_cast<EkfFusionStrategy*>(strat->clone().release()));
  strat->set_joseph_form(true);
  api::StandardMeasurementModel m{vec({2.5, -1.0}), [](const Vector& x) { return Vector(x.head(2)); },
                                  mat({{1, 0, 0}, {0, 1, 0}}), mat({{3, 0.5}, {0.5, 2}})};
  strat->update(m);
  simple->update(m);
  EXPECT_ALLCLOSE(*strat->estimate(), *simple->estimate());
  EXPECT_TRUE(allclose(*strat->covariance(), *simple->covariance(), 1e-9, 1e-9));
}

TEST_F(EkfTest, CloneIsIndependent) {
  pop(2, 1);
  auto c = strat->clone();
  strat->set_estimate_slice(vec({7, 8}), 0);
  EXPECT_ALLCLOSE(*c->estimate(), vec({1, 2}));
}

}  // namespace
