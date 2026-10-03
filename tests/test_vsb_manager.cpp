// Port of the VirtualStateBlockManager tests in pntos-cobra/tests/test_virtual_state_blocks.py
#include <pntos/cobra/fusion/VirtualStateBlockManager.hpp>
#include <pntos/cobra/state_modeling/VirtualStateBlocks.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <random>

using namespace pntos;
using namespace pntos::test;
using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::Matrix;
using api::Message;
using api::Timestamp;
using api::Vector;

namespace {
const std::string kSource = "test_source";

std::unique_ptr<cobra::StateExtractor> se(TestMediator& m, const std::string& s, const std::string& t) {
  return std::make_unique<cobra::StateExtractor>(&m, s, t, 3, std::vector<int>{0, 1, 2});
}

TEST(VsbManager, ValidOps) {
  TestMediator med;
  cobra::VirtualStateBlockManager vsbm(&med);
  Vector est = vec({1, 2, 3});
  EstimateWithCovariance ewc{EstimateWithCovarianceType::EWC_GENERIC, est, Matrix::Identity(3, 3) * 2.0};

  const int max_val = 25;
  std::vector<std::unique_ptr<api::VirtualStateBlock>> vsbs;
  std::vector<std::string> targets;
  std::string prev = kSource;
  for (int i = 0; i < max_val; ++i) {
    std::string t = "t" + std::to_string(i);
    vsbs.push_back(se(med, prev, t));
    targets.push_back(t);
    prev = t;
  }
  std::shuffle(vsbs.begin(), vsbs.end(), std::mt19937{42});
  for (auto& v : vsbs) vsbm.add_virtual_state_block(std::move(v));

  const std::string last = targets.back();
  auto ewc_out = vsbm.convert(ewc, targets[0], last, Timestamp(0));
  ASSERT_TRUE(ewc_out);
  EXPECT_ALLCLOSE(ewc_out->estimate, ewc.estimate);
  EXPECT_ALLCLOSE(ewc_out->covariance, ewc.covariance);
  auto est_out = vsbm.convert_estimate(est, targets[0], last, Timestamp(0));
  ASSERT_TRUE(est_out);
  EXPECT_ALLCLOSE(*est_out, est);
  auto jac = vsbm.jacobian(est, targets[0], last, Timestamp(0));
  ASSERT_TRUE(jac);
  EXPECT_ALLCLOSE(*jac, Matrix::Identity(3, 3));

  auto start = vsbm.get_start_block_label(last);
  ASSERT_TRUE(start);
  EXPECT_EQ(*start, kSource);
  start = vsbm.get_start_block_label(last);  // cached path
  ASSERT_TRUE(start);
  EXPECT_EQ(*start, kSource);

  vsbm.add_virtual_state_block(se(med, "new_source", "t25"));
  targets.push_back("t25");
  auto labels = vsbm.get_virtual_state_block_labels();
  ASSERT_TRUE(labels);
  for (const auto& l : *labels) EXPECT_NE(std::find(targets.begin(), targets.end(), l), targets.end());
  EXPECT_EQ(labels->size(), targets.size());
  EXPECT_FALSE(med.has_error());

  // Removing the tail of the chain leaves the rest intact; removing a middle node prunes its descendants.
  vsbm.remove_virtual_state_block(last);
  EXPECT_FALSE(vsbm.has_node(last));
  EXPECT_TRUE(vsbm.has_node(targets[max_val - 2]));
  vsbm.remove_virtual_state_block("t10");
  for (int i = 10; i < max_val - 1; ++i) EXPECT_FALSE(vsbm.has_node("t" + std::to_string(i)));
  EXPECT_TRUE(vsbm.has_node("t9"));
  EXPECT_TRUE(vsbm.convert_estimate(est, kSource, "t9", Timestamp(0)).has_value());
  EXPECT_FALSE(vsbm.convert_estimate(est, kSource, "t11", Timestamp(0)).has_value());
}

TEST(VsbManager, GiveAuxData) {
  TestMediator med;
  cobra::VirtualStateBlockManager vsbm(&med);
  vsbm.add_virtual_state_block(std::make_unique<cobra::PinsonErrorToStandard>(&med, kSource, "give_data"));
  Message pva(make_pva(0, 0.6, -1.5, 300, 1, 2, 3, vec({1, 0, 0, 0})), "test");
  vsbm.give_virtual_state_block_aux_data("give_data", {pva});
  auto* blk = dynamic_cast<const cobra::PinsonErrorToStandard*>(vsbm.block("give_data"));
  ASSERT_TRUE(blk);
  ASSERT_TRUE(blk->pva());
  EXPECT_DOUBLE_EQ(blk->pva()->get_p1(), 0.6);
}

TEST(VsbManager, InvalidOps) {
  TestMediator med;
  cobra::VirtualStateBlockManager vsbm(&med);
  Vector est = vec({1, 2, 3});
  EstimateWithCovariance ewc{EstimateWithCovarianceType::EWC_GENERIC, est, Matrix::Identity(3, 3)};
  EXPECT_FALSE(vsbm.get_virtual_state_block_labels().has_value());
  vsbm.add_virtual_state_block(se(med, kSource, kSource));
  EXPECT_FALSE(vsbm.has_node(kSource));
  EXPECT_TRUE(med.has_error());

  vsbm.add_virtual_state_block(se(med, kSource, "duplicate"));
  vsbm.add_virtual_state_block(se(med, kSource, "duplicate"));
  EXPECT_EQ(vsbm.num_nodes(), 2u);

  Timestamp t(0);
  EXPECT_FALSE(vsbm.convert(ewc, kSource, "bad_targ", t).has_value());
  EXPECT_FALSE(vsbm.convert_estimate(est, kSource, "bad_targ", t).has_value());
  EXPECT_FALSE(vsbm.jacobian(est, kSource, "bad_targ", t).has_value());
  EXPECT_FALSE(vsbm.get_start_block_label("bad_targ").has_value());
  vsbm.give_virtual_state_block_aux_data("bad_targ", {});
  vsbm.give_virtual_state_block_aux_data(kSource, {});  // root without block
  vsbm.remove_virtual_state_block("bad_targ");
  EXPECT_EQ(vsbm.num_nodes(), 2u);
  EXPECT_FALSE(vsbm.convert(ewc, "bad_source", "duplicate", t).has_value());
}

TEST(VsbManager, CopyIsDeep) {
  TestMediator med;
  cobra::VirtualStateBlockManager a(&med);
  a.add_virtual_state_block(std::make_unique<cobra::PinsonErrorToStandard>(&med, kSource, "pes"));
  cobra::VirtualStateBlockManager b(a);
  Message pva(make_pva(0, 0.6, -1.5, 300, 1, 2, 3, vec({1, 0, 0, 0})), "test");
  b.give_virtual_state_block_aux_data("pes", {pva});
  EXPECT_EQ(dynamic_cast<const cobra::PinsonErrorToStandard*>(a.block("pes"))->pva(), nullptr);
  EXPECT_NE(dynamic_cast<const cobra::PinsonErrorToStandard*>(b.block("pes"))->pva(), nullptr);
  EXPECT_NE(a.block("pes"), b.block("pes"));
}

}  // namespace
