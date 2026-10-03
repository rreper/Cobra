// Shared test helpers: a recording Mediator and small ASPN message builders.
#pragma once

#include <pntos/api.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementPositionVelocityAttitude.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace pntos::test {

using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using DynVector = Eigen::Matrix<double, Eigen::Dynamic, 1>;

struct LogEntry {
  api::LoggingLevel level;
  std::string message;
};

/// A Mediator that records log messages and (optionally) exposes a registry.
/// Orchestration-style calls (request_solutions etc.) are no-ops.
class TestMediator final : public api::Mediator {
 public:
  explicit TestMediator(std::shared_ptr<api::Registry> registry = nullptr) : registry_(std::move(registry)) {}

  std::vector<std::string> filter_description_list() const override { return {}; }
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>&, const std::optional<std::string>&) override {
    return std::nullopt;
  }
  void process_pntos_message(const api::Message& m) override { processed.push_back(m); }
  void broadcast_aspn_message(const api::Message& m, const std::optional<std::string>&,
                              const std::optional<std::string>&) override {
    broadcast.push_back(m);
  }
  void log_message(api::LoggingLevel level, const std::string& message) override {
    logs.push_back({level, message});
  }
  api::Registry& registry() override {
    if (!registry_) throw std::runtime_error("TestMediator has no registry");
    return *registry_;
  }
  void set_registry(std::shared_ptr<api::Registry> r) { registry_ = std::move(r); }

  std::size_t count(api::LoggingLevel level) const {
    std::size_t n = 0;
    for (const auto& l : logs)
      if (l.level == level) ++n;
    return n;
  }
  bool has_error() const { return count(api::LoggingLevel::ERROR) > 0; }
  std::string last_message() const { return logs.empty() ? std::string() : logs.back().message; }

  std::vector<LogEntry> logs;
  std::vector<api::Message> processed;
  std::vector<api::Message> broadcast;

 private:
  std::shared_ptr<api::Registry> registry_;
};

inline aspn23_eigen::TypeHeader header(Aspn23MessageType type) { return aspn23_eigen::TypeHeader(type, 0, 0, 0, 0); }

inline DynVector dyn(std::initializer_list<double> v) {
  DynVector out(static_cast<Eigen::Index>(v.size()));
  Eigen::Index i = 0;
  for (double d : v) out(i++) = d;
  return out;
}

inline RowMajorMatrix rm(const api::Matrix& m) { return RowMajorMatrix(m); }

inline std::shared_ptr<aspn23_eigen::MeasurementPositionVelocityAttitude> make_pva(
    std::int64_t tov_nsec, double lat, double lon, double alt, double vn, double ve, double vd,
    const api::Vector& quat, const api::Matrix& cov = api::Matrix::Zero(9, 9)) {
  return std::make_shared<aspn23_eigen::MeasurementPositionVelocityAttitude>(
      header(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE), aspn23_eigen::TypeTimestamp(tov_nsec),
      ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC, lat, lon, alt, vn, ve, vd,
      DynVector(quat), rm(cov), ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE, DynVector(0),
      std::vector<aspn23_eigen::TypeIntegrity>{});
}

inline std::shared_ptr<aspn23_eigen::MeasurementPosition> make_position(std::int64_t tov_nsec, double lat, double lon,
                                                                        double alt, const api::Matrix& cov,
                                                                        Aspn23MeasurementPositionReferenceFrame frame =
                                                                            ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC) {
  return std::make_shared<aspn23_eigen::MeasurementPosition>(
      header(ASPN_MEASUREMENT_POSITION), aspn23_eigen::TypeTimestamp(tov_nsec), frame, lat, lon, alt, rm(cov),
      ASPN23_MEASUREMENT_POSITION_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
}

inline std::shared_ptr<aspn23_eigen::MeasurementImu> make_imu(std::int64_t tov_nsec, const api::Vector3& accel,
                                                              const api::Vector3& gyro) {
  return std::make_shared<aspn23_eigen::MeasurementImu>(header(ASPN_MEASUREMENT_IMU),
                                                        aspn23_eigen::TypeTimestamp(tov_nsec),
                                                        ASPN23_MEASUREMENT_IMU_IMU_TYPE_SAMPLED, DynVector(accel),
                                                        DynVector(gyro), std::vector<aspn23_eigen::TypeIntegrity>{});
}

/// numpy.allclose semantics.
inline bool allclose(const api::Matrix& a, const api::Matrix& b, double rtol = 1e-5, double atol = 1e-8) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  for (Eigen::Index i = 0; i < a.rows(); ++i)
    for (Eigen::Index j = 0; j < a.cols(); ++j)
      if (std::abs(a(i, j) - b(i, j)) > atol + rtol * std::abs(b(i, j))) return false;
  return true;
}

inline api::Matrix mat(std::initializer_list<std::initializer_list<double>> rows) {
  const auto r = static_cast<Eigen::Index>(rows.size());
  const auto c = static_cast<Eigen::Index>(rows.begin()->size());
  api::Matrix m(r, c);
  Eigen::Index i = 0;
  for (const auto& row : rows) {
    Eigen::Index j = 0;
    for (double v : row) m(i, j++) = v;
    ++i;
  }
  return m;
}

inline api::Vector vec(std::initializer_list<double> v) { return api::Vector(dyn(v)); }

}  // namespace pntos::test

#define EXPECT_ALLCLOSE(a, b) EXPECT_TRUE(::pntos::test::allclose((a), (b))) << "\n" << (a) << "\n!=\n" << (b)
