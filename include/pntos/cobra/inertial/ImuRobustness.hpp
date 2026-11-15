// Opt-in handling of imperfect live IMU data in BufferedImu / StandardInertialPlugin. Every default
// reproduces NavToolkit exactly (tools/inertial_parity_check.py); nothing here changes the mechanization
// equations, only which samples are accepted and what interval their increments are taken to cover.
#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace pntos::cobra::inertial {

/// What to do with an IMU message that is not INTEGRATED (i.e. SAMPLED: m/s² and rad/s).
enum class SampledImuPolicy {
  THROW,      ///< std::invalid_argument, as NavToolkit does in ErrorMode::DIE (default)
  DROP,       ///< reject the sample, as NavToolkit does in ErrorMode::OFF (what the Python apps run with)
  INTEGRATE,  ///< mechanize it as rates held over the time since the previous sample
};

/// How a sample is mechanized when it arrives late, i.e. after one or more samples were lost.
enum class ImuGapPolicy {
  /// NavToolkit: increments are used as they are and dt comes from the time stamps. The increments of a
  /// lost sample are simply missing while gravity is still integrated over the whole gap, so every lost
  /// 10 ms sample of a static IMU adds about 0.1 m/s of downward velocity (default).
  NAVTK,
  /// The late sample stands for the lost ones as well: its rate is held across the gap (see gap_factor).
  /// This is what reset() already assumes for the partial interval after a reset and what the averaged
  /// forces assume when they weight a record by its dt. SAMPLED data, being rates, is always held.
  HOLD,
};

/// Attitude interpolation between two buffered solutions.
enum class AttitudeInterpolation {
  RPY,         ///< NavToolkit: quaternion -> roll/pitch/yaw -> slerp -> roll/pitch/yaw -> quaternion (default)
  QUATERNION,  ///< slerp on the quaternions themselves; no singularity at pitch = +-90 degrees
};

struct ImuRobustness {
  SampledImuPolicy sampled_imu = SampledImuPolicy::THROW;
  /// Reject IMU samples, resets and error corrections holding NaN or infinity (and solutions without
  /// position, velocity or attitude) instead of letting them poison every later solution.
  bool reject_non_finite = false;
  ImuGapPolicy gap_policy = ImuGapPolicy::NAVTK;
  /// A sample is "late" when its dt exceeds gap_factor x estimated_dt(); it is then mechanized
  /// round(dt / estimated_dt()) times (at least twice) in steps of one nominal interval, i.e. as if every
  /// sample lost before it had carried the same increments. A single lost sample gives 2.0; the recorded
  /// VN-100 example log has time-stamp jitter up to 1.6 with no sample lost. Must be greater than 1.
  double gap_factor = 1.75;
  /// Seconds, 0 = unlimited. A sample further than this ahead of the latest solution is rejected (a lone
  /// corrupt time stamp must not move the inertial into the future, where every later sample is "old").
  /// If the samples that follow agree with it (kGapConfirmations in a row, each within max_gap of the one
  /// before) the time base really moved, and the last of them is accepted and bridged per gap_policy.
  double max_gap = 0.0;
  /// 0: estimated_dt() is the mean of every interval since construction (NavToolkit), so one long gap
  /// biases the reported forces and rates for the rest of the run. N >= kMinDtWindow: median of the last
  /// N intervals. gap_policy = HOLD needs a nominal interval that gaps cannot move and uses
  /// N = kDefaultDtWindow when this is left at 0.
  std::size_t dt_window = 0;
  AttitudeInterpolation attitude_interpolation = AttitudeInterpolation::RPY;

  static constexpr std::size_t kDefaultDtWindow = 100;
  static constexpr std::size_t kMinDtWindow = 3;
  static constexpr int kGapConfirmations = 3;

  /// Everything switched on with the values recommended for a live sensor: sampled data integrated,
  /// non-finite input rejected, gaps held, at most 1 s bridged without confirmation, windowed dt estimate,
  /// quaternion interpolation.
  static ImuRobustness live_sensor() {
    ImuRobustness r;
    r.sampled_imu = SampledImuPolicy::INTEGRATE;
    r.reject_non_finite = true;
    r.gap_policy = ImuGapPolicy::HOLD;
    r.max_gap = 1.0;
    r.dt_window = kDefaultDtWindow;
    r.attitude_interpolation = AttitudeInterpolation::QUATERNION;
    return r;
  }

  bool is_default() const { return *this == ImuRobustness{}; }
  bool operator==(const ImuRobustness&) const = default;
};

/// Names used in config files and in the registry ("throw" / "drop" / "integrate", "navtk" / "hold",
/// "rpy" / "quaternion"); the parsers return nullopt for anything else.
const char* to_string(SampledImuPolicy p);
const char* to_string(ImuGapPolicy p);
const char* to_string(AttitudeInterpolation p);
std::optional<SampledImuPolicy> sampled_imu_policy_from_string(const std::string& s);
std::optional<ImuGapPolicy> imu_gap_policy_from_string(const std::string& s);
std::optional<AttitudeInterpolation> attitude_interpolation_from_string(const std::string& s);

}  // namespace pntos::cobra::inertial
