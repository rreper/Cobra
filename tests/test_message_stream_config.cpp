// Port of pntos-cobra/tests/test_message_stream_config.py
#include <pntos/cobra/controller/StandardMessageStreamConfig.hpp>

#include <gtest/gtest.h>

using pntos::cobra::StandardMessageStreamConfig;

namespace {
const std::vector<Aspn23MessageType> kTypes = {ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, ASPN_MEASUREMENT_ALTITUDE,
                                               ASPN_MEASUREMENT_ANGULAR_VELOCITY, ASPN_MEASUREMENT_HEADING,
                                               ASPN_MEASUREMENT_SATNAV};
constexpr std::size_t n = 3;

void expect_split(const StandardMessageStreamConfig& c, bool first_n_sequenced) {
  for (std::size_t i = 0; i < kTypes.size(); ++i)
    EXPECT_EQ(c.is_sequenced(kTypes[i]), i < n ? first_n_sequenced : !first_n_sequenced) << i;
}

TEST(MessageStreamConfig, ImmediateStreamAll) {
  StandardMessageStreamConfig c;
  c.immediate_stream_all(true);
  for (auto t : kTypes) EXPECT_FALSE(c.is_sequenced(t));
}

TEST(MessageStreamConfig, SequencedStreamAll) {
  StandardMessageStreamConfig c;
  c.sequenced_stream_all(true);
  for (auto t : kTypes) EXPECT_TRUE(c.is_sequenced(t));
}

TEST(MessageStreamConfig, ImmediateStreamAdd) {
  StandardMessageStreamConfig c;
  c.sequenced_stream_all(true);
  for (std::size_t i = 0; i < n; ++i) c.immediate_stream_add(kTypes[i]);
  expect_split(c, false);
}

TEST(MessageStreamConfig, SequencedStreamAdd) {
  StandardMessageStreamConfig c;
  c.immediate_stream_all(true);
  for (std::size_t i = 0; i < n; ++i) c.sequenced_stream_add(kTypes[i]);
  expect_split(c, true);
}

TEST(MessageStreamConfig, ImmediateStreamRemove) {
  StandardMessageStreamConfig c;
  c.sequenced_stream_all(true);
  for (std::size_t i = 0; i < n; ++i) c.immediate_stream_add(kTypes[i]);
  expect_split(c, false);
  for (std::size_t i = 0; i < n; ++i) c.immediate_stream_remove(kTypes[i]);
  for (auto t : kTypes) EXPECT_TRUE(c.is_sequenced(t));
}

TEST(MessageStreamConfig, SequencedStreamRemove) {
  StandardMessageStreamConfig c;
  c.immediate_stream_all(true);
  for (std::size_t i = 0; i < n; ++i) c.sequenced_stream_add(kTypes[i]);
  expect_split(c, true);
  for (std::size_t i = 0; i < n; ++i) c.sequenced_stream_remove(kTypes[i]);
  for (auto t : kTypes) EXPECT_FALSE(c.is_sequenced(t));
}

TEST(MessageStreamConfig, ImmediateAddAfterSequencedAdd) {
  StandardMessageStreamConfig c;
  c.immediate_stream_all(true);
  for (std::size_t i = 0; i < n; ++i) c.sequenced_stream_add(kTypes[i]);
  expect_split(c, true);
  for (std::size_t i = 0; i < n; ++i) c.immediate_stream_add(kTypes[i]);
  for (auto t : kTypes) EXPECT_FALSE(c.is_sequenced(t));
}

TEST(MessageStreamConfig, SequencedAddAfterImmediateAdd) {
  StandardMessageStreamConfig c;
  c.sequenced_stream_all(true);
  for (std::size_t i = 0; i < n; ++i) c.immediate_stream_add(kTypes[i]);
  expect_split(c, false);
  for (std::size_t i = 0; i < n; ++i) c.sequenced_stream_add(kTypes[i]);
  for (auto t : kTypes) EXPECT_TRUE(c.is_sequenced(t));
}

TEST(MessageStreamConfig, SourceSpecificOverrides) {
  StandardMessageStreamConfig c;
  c.sequenced_stream_all(true);
  c.immediate_stream_add(ASPN_MEASUREMENT_IMU, "imu_a");
  EXPECT_FALSE(c.is_sequenced(ASPN_MEASUREMENT_IMU, "imu_a"));
  EXPECT_TRUE(c.is_sequenced(ASPN_MEASUREMENT_IMU, "imu_b"));
  EXPECT_TRUE(c.is_sequenced(ASPN_MEASUREMENT_IMU));
  c.immediate_stream_add(ASPN_MEASUREMENT_IMU);
  c.sequenced_stream_add(ASPN_MEASUREMENT_IMU, "imu_b");
  EXPECT_FALSE(c.is_sequenced(ASPN_MEASUREMENT_IMU, "imu_c"));  // type-wide override
  EXPECT_TRUE(c.is_sequenced(ASPN_MEASUREMENT_IMU, "imu_b"));   // exact match wins
}

}  // namespace
