// Small helpers over aspn23_eigen messages.
#pragma once

#include <pntos/api/types.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementPositionVelocityAttitude.hpp>

#include <cmath>
#include <optional>

namespace pntos::cobra::utils {

using PVA = aspn23_eigen::MeasurementPositionVelocityAttitude;

/// Time of validity for the ASPN measurement types Cobra handles; nullopt for metadata types.
std::optional<api::Timestamp> time_of_validity(const api::AspnBase& msg);
inline bool has_tov(const api::AspnBase& msg) { return time_of_validity(msg).has_value(); }

/// A field that ASPN marks optional is "absent" when NaN.
inline bool present(double v) { return !std::isnan(v); }

/// Quaternion of a PVA as a fixed 4-vector, nullopt if absent (empty or NaN).
std::optional<nav::Vector4> quaternion(const PVA& pva);
inline api::Vector3 position(const PVA& pva) { return api::Vector3(pva.get_p1(), pva.get_p2(), pva.get_p3()); }
inline api::Vector3 velocity(const PVA& pva) { return api::Vector3(pva.get_v1(), pva.get_v2(), pva.get_v3()); }
inline bool has_position(const PVA& pva) {
  return present(pva.get_p1()) && present(pva.get_p2()) && present(pva.get_p3());
}
inline bool has_velocity(const PVA& pva) {
  return present(pva.get_v1()) && present(pva.get_v2()) && present(pva.get_v3());
}

/// Build a geodetic PVA message (NONE error model, no integrity).
std::shared_ptr<PVA> make_pva(const aspn23_eigen::TypeHeader& header, api::Timestamp tov, const api::Vector3& llh,
                              const api::Vector3& vel_ned, const nav::Vector4& quat, const api::Matrix& covariance);

/// Deep copy of a PVA (the aspn classes own a C struct; copy ctor is deep).
inline std::shared_ptr<PVA> copy_pva(const PVA& p) { return std::make_shared<PVA>(p); }

/// Shorthand for the "Nx1 estimate" convention: view a vector as a column matrix and back.
inline api::Matrix as_column(const api::Vector& v) { return api::Matrix(v); }

}  // namespace pntos::cobra::utils
