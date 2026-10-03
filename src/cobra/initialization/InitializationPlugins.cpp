#include <pntos/cobra/inertial/BufferedImu.hpp>
#include <pntos/cobra/initialization/InitializationPlugins.hpp>
#include <pntos/cobra/utils/logging.hpp>

namespace pntos::cobra {

using api::InitializationStatus;
using api::LoggingLevel;
using api::Matrix;
using api::Message;
using api::Timestamp;
using api::Vector3;

namespace {
Vector3 v3(const Vec3& a) { return Vector3(a[0], a[1], a[2]); }
}  // namespace

// ----------------------------------------------------------------------------- ManualInitialization

ManualInitialization::ManualInitialization(const std::string& group, api::Mediator* mediator) {
  std::optional<ManualAlignmentConfig> c = mediator ? ManualAlignmentConfig::from_registry(*mediator, group) : std::nullopt;
  if (!c) throw std::invalid_argument("ManualInitialization: missing ManualAlignmentConfig at " + group);
  api::Vector d(9);
  d << v3(c->initial_pos_var), v3(c->initial_vel_var), v3(c->initial_tilt_var);
  auto pva = utils::make_pva(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0),
                             Timestamp{static_cast<std::int64_t>(c->initial_time * 1e9)}, v3(c->initial_pos),
                             v3(c->initial_vel), nav::rpy_to_quat(v3(c->initial_rpy)), Matrix(d.asDiagonal()));
  solution_.solution = Message(pva, kSourceIdentifier);
  api::StandardInertialErrors e;
  e.accel_biases = v3(c->initial_accel_bias);
  e.gyro_biases = v3(c->initial_gyro_bias);
  e.accel_scale_factors = v3(c->initial_accel_scale_factor);
  e.gyro_scale_factors = v3(c->initial_gyro_scale_factor);
  solution_.inertial_errors = e;
  api::Vector b(6);
  b << v3(c->initial_accel_bias_var), v3(c->initial_gyro_bias_var);
  solution_.inertial_error_covariance = Matrix(b.asDiagonal());
  solution_.status = InitializationStatus::INITIALIZED_GOOD;
}

// ----------------------------------------------------------------------------- AlignmentStrategy

AlignmentStrategy::AlignmentStrategy(std::unique_ptr<inertial::AlignBase> aligner, api::Mediator* mediator)
    : aligner_(std::move(aligner)), mediator_(mediator) {}

api::InitializationMotionNeeded AlignmentStrategy::request_motion_needed() const {
  switch (aligner_->motion_needed()) {
    case inertial::MotionNeeded::NO_MOTION: return api::InitializationMotionNeeded::NO_MOTION;
    case inertial::MotionNeeded::MOTION_NEEDED: return api::InitializationMotionNeeded::MOTION_NEEDED;
    default: return api::InitializationMotionNeeded::ANY_MOTION;
  }
}

InitializationStatus AlignmentStrategy::request_current_status() const {
  switch (aligner_->check_alignment_status()) {
    case inertial::AlignmentStatus::ALIGNING_COARSE: return InitializationStatus::INITIALIZING_COARSE;
    case inertial::AlignmentStatus::ALIGNING_FINE: return InitializationStatus::INITIALIZING_FINE;
    case inertial::AlignmentStatus::ALIGNED_GOOD: return InitializationStatus::INITIALIZED_GOOD;
  }
  return InitializationStatus::INITIALIZATION_FAILED;
}

void AlignmentStrategy::process_pntos_message(const Message& message) {
  if (!message.wrapped_message) return;
  const auto type = message.message_type();
  if (type != ASPN_MEASUREMENT_IMU && type != ASPN_MEASUREMENT_POSITION &&
      type != ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, "Could not convert message");
    return;
  }
  try {
    aligner_->process(*message.wrapped_message);
  } catch (const std::exception& e) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, std::string("Alignment failed: ") + e.what());
  }
}

api::InitialInertialSolution AlignmentStrategy::request_solution() {
  api::InitialInertialSolution out;
  out.status = request_current_status();
  auto [have_sol, sol] = aligner_->get_computed_alignment();
  auto [have_cov, cov] = aligner_->get_computed_covariance();
  auto [have_err, err] = aligner_->get_imu_errors();
  if (have_sol) {
    // NavToolkit stores C_nav_to_sensor; the ASPN quaternion is C_sensor_to_nav.
    auto pva = utils::make_pva(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0),
                               sol.time, sol.pos, sol.vel, nav::dcm_to_quat(sol.rot_mat.transpose()),
                               have_cov ? Matrix(cov.topLeftCorner(9, 9)) : Matrix::Zero(9, 9));
    out.solution = Message(pva, kSourceIdentifier);
  }
  if (have_cov) out.inertial_error_covariance = Matrix(cov.bottomRightCorner(cov.rows() - 9, cov.cols() - 9));
  if (have_err)
    out.inertial_errors = api::StandardInertialErrors{err.accel_biases, err.gyro_biases, err.accel_scale_factors,
                                                      err.gyro_scale_factors};
  return out;
}

// ----------------------------------------------------------------------------- PvaMessageInitialization

PvaMessageInitialization::PvaMessageInitialization(api::Mediator* mediator, const PvaMessageInitializationConfig& c)
    : mediator_(mediator), channel_(c.initial_pva_channel) {
  if (c.start_time) start_time_ns_ = static_cast<std::int64_t>(*c.start_time * 1e9);
  if (c.initial_pva_sigma) {
    api::Vector s(9);
    for (int i = 0; i < 9; ++i) s(i) = (*c.initial_pva_sigma)[static_cast<std::size_t>(i)];
    pva_cov_ = Matrix(s.cwiseAbs2().asDiagonal());
  }
  api::Vector b(6);
  b << v3(c.initial_accel_bias_sigma), v3(c.initial_gyro_bias_sigma);
  imu_error_cov_ = Matrix(b.cwiseAbs2().asDiagonal());
}

void PvaMessageInitialization::process_pntos_message(const Message& message) {
  if (message.source_identifier != channel_) return;
  auto pva = message.as<utils::PVA>();
  if (!pva) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::WARN,
                             "PvaMessageInitialization expected a MeasurementPositionVelocityAttitude on channel " +
                                 channel_ + ". Cannot process message.");
    return;
  }
  const std::int64_t t = pva->get_time_of_validity().get_elapsed_nsec();
  if (start_time_ns_ && t < *start_time_ns_) return;
  auto initial = utils::copy_pva(*pva);
  if (pva_cov_) initial->set_covariance(*pva_cov_);
  solution_.solution = Message(initial, "Initial PVA");
  solution_.inertial_errors = api::StandardInertialErrors{};
  solution_.inertial_error_covariance = imu_error_cov_;
  solution_.status = InitializationStatus::INITIALIZED_GOOD;
  status_ = InitializationStatus::INITIALIZED_GOOD;
}

// ----------------------------------------------------------------------------- plugins

bool InitializationPluginBase::config_group_ok(const std::optional<std::string>& group) const {
  if (group) return true;
  if (mediator_)
    mediator_->log_message(LoggingLevel::ERROR, "config_group is a required parameter for this plugin and cannot be None");
  else
    utils::print_message(LoggingLevel::ERROR, identifier_, "config_group is required");
  return false;
}

std::unique_ptr<api::CommonInitializationStrategy> InitializationPluginBase::new_initialization_strategy(
    api::InitializationType type, const std::optional<std::string>& group) {
  if (!config_group_ok(group)) return nullptr;
  if (!is_initialization_type_supported(type)) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, "Unsupported type requested.");
    return nullptr;
  }
  try {
    return make(*group);
  } catch (const std::exception& e) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, e.what());
    return nullptr;
  }
}

std::unique_ptr<api::CommonInitializationStrategy> TutorialInitializationPlugin::make(const std::string& group) {
  return std::make_unique<ManualInitialization>(group, mediator_);
}

std::unique_ptr<api::CommonInitializationStrategy> StaticAlignInitializationPlugin::make(const std::string& group) {
  auto c = mediator_ ? StaticAlignmentConfig::from_registry(*mediator_, group) : std::nullopt;
  if (!c)
    throw std::invalid_argument("Failed to populate config from registry to config type StaticAlignmentConfig and group " +
                                group + ".");
  return std::make_unique<AlignmentStrategy>(
      std::make_unique<inertial::StaticAlignment>(inertial::ImuModel::from_config(c->imu_model), c->static_time), mediator_);
}

std::unique_ptr<api::CommonInitializationStrategy> ManualHeadingAlignInitializationPlugin::make(const std::string& group) {
  auto c = mediator_ ? ManualHeadingAlignmentConfig::from_registry(*mediator_, group) : std::nullopt;
  if (!c)
    throw std::invalid_argument(
        "Failed to populate config from registry to config type ManualHeadingAlignmentConfig and group " + group + ".");
  return std::make_unique<AlignmentStrategy>(
      std::make_unique<inertial::ManualHeadingAlignment>(c->heading, c->heading_sigma,
                                                         inertial::ImuModel::from_config(c->imu_model), c->static_time),
      mediator_);
}

std::unique_ptr<api::CommonInitializationStrategy> PvaMessageInitializationPlugin::make(const std::string& group) {
  auto c = mediator_ ? PvaMessageInitializationConfig::from_registry(*mediator_, group) : std::nullopt;
  if (!c)
    throw std::invalid_argument(
        "new_initialization_strategy() could not extract PvaMessageInitializationConfig to create PvaMessageInitialization");
  return std::make_unique<PvaMessageInitialization>(mediator_, *c);
}

}  // namespace pntos::cobra
