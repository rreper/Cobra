#include <pntos/cobra/state_modeling/Pinson15NedBlock.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <cmath>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Matrix;
using api::Matrix3;
using api::Vector;
using api::Vector3;

Pinson15NedBlock::Pinson15NedBlock(std::string label, api::Mediator* mediator, const ImuConfig& imu_model)
    : label_(std::move(label)), mediator_(mediator), imu_model_(imu_model) {
  Vector diag(15);
  diag.setZero();
  for (int i = 0; i < 3; ++i) {
    diag(3 + i) = imu_model.accel_random_walk_sigma[i];
    diag(6 + i) = imu_model.gyro_random_walk_sigma[i];
    diag(9 + i) = imu_model.accel_bias_sigma[i] * std::sqrt(2.0 / imu_model.accel_bias_tau[i]);
    diag(12 + i) = imu_model.gyro_bias_sigma[i] * std::sqrt(2.0 / imu_model.gyro_bias_tau[i]);
  }
  pre_Q_ = diag.array().square().matrix().asDiagonal();
}

void Pinson15NedBlock::receive_aux_data(const api::AuxData& aux) {
  for (const auto& message : aux) {
    if (!message) continue;
    if (auto pva = message->as<utils::PVA>()) {
      if (!utils::quaternion(*pva)) {
        if (mediator_)
          mediator_->log_message(LoggingLevel::WARN,
                                 "Pinson15NedBlock received PVA aux data with no quaternion at time " +
                                     std::to_string(pva->get_time_of_validity().get_elapsed_nsec() * 1e-9) +
                                     "s. Ignoring.");
        continue;
      }
      old_pva_aux_ = new_pva_aux_;
      new_pva_aux_ = pva;
    } else if (auto imu = message->as<aspn23_eigen::MeasurementImu>()) {
      force_and_rate_aux_ = imu;
    } else if (mediator_) {
      mediator_->log_message(LoggingLevel::ERROR,
                             "Pinson15NedBlock expected aux data of type MeasurementPositionVelocityAttitude or "
                             "MeasurementImu, but got message of type " +
                                 std::to_string(static_cast<int>(message->message_type())) + ".");
    }
  }
}

std::optional<api::StandardDynamicsModel> Pinson15NedBlock::generate_dynamics(const api::GenXandP&,
                                                                              api::Timestamp from,
                                                                              api::Timestamp to) {
  if (!new_pva_aux_ || !force_and_rate_aux_) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::ERROR, "Pinson15NedBlock cannot propagate from time " +
                                                      std::to_string(from.seconds()) + "s to " +
                                                      std::to_string(to.seconds()) +
                                                      "s as it has not received PVA and force aux data.");
    return std::nullopt;
  }
  const double dt = static_cast<double>(to.elapsed_nsec - from.elapsed_nsec) * 1e-9;
  const Matrix F = generate_f_pinson15();
  const Matrix Q = generate_q_pinson15();

  // Second-order discretisation
  Matrix Phi = 0.5 * dt * dt * (F * F) + (F * dt + Matrix::Identity(15, 15));
  const Matrix Q_prop = Phi * Q * Phi.transpose();
  Matrix Qd = (Q_prop + Q) * (0.5 * dt);

  scale_phi(Phi);

  api::StandardDynamicsModel model;
  model.Phi = Phi;
  model.Qd = Qd;
  model.g = [Phi](const Vector& x) { return Vector(Phi * x); };
  return model;
}

void Pinson15NedBlock::scale_phi(Matrix& Phi) const {
  if (old_pva_aux_ && new_pva_aux_) {
    const Vector3 pos = utils::position(*old_pva_aux_);
    const double lat_factor0 = nav::delta_lat_to_north(1, pos(0), pos(2));
    const double lon_factor0 = nav::delta_lon_to_east(1, pos(0), pos(2));
    const Vector3 new_pos = utils::position(*new_pva_aux_);
    const double lat_factor1 = nav::delta_lat_to_north(1, new_pos(0), new_pos(2));
    const double lon_factor1 = nav::delta_lon_to_east(1, new_pos(0), new_pos(2));
    Phi.col(0) *= lat_factor1 / lat_factor0;
    Phi.col(1) *= lon_factor1 / lon_factor0;
  }
}

Matrix Pinson15NedBlock::generate_f_pinson15() const {
  const Vector3 pos = utils::position(*new_pva_aux_);
  const Vector3 vel = utils::velocity(*new_pva_aux_);
  const Vector3 force = Vector3(force_and_rate_aux_->get_meas_accel());
  const Matrix3 C_sensor_to_ned = nav::quat_to_dcm(*utils::quaternion(*new_pva_aux_));

  const nav::EarthModel earth(pos, vel);
  const double omega = nav::ROTATION_RATE;
  const double sinl = earth.sin_l, cosl = earth.cos_l, tanl = earth.tan_l;
  const double vn = vel(0), ve = vel(1), vd = vel(2);
  const double re = earth.r_e, rn = earth.r_n;
  Matrix3 scalem2r;
  scalem2r << 1 / earth.lat_factor, 0, 0, 0, 1 / earth.lon_factor, 0, 0, 0, -1;

  Matrix F = Matrix::Zero(15, 15);

  Matrix3 block1;  // dtilt = block1 * dtilt
  block1 << 0, -(omega * sinl + ve / re * tanl), vn / rn,  //
      (omega * sinl + ve / re * tanl), 0, omega * cosl + ve / re,  //
      -vn / rn, -omega * cosl - ve / re, 0;

  Matrix3 block2;  // dtilt = block2 * dvel
  block2 << 0, 1 / re, 0, -1 / rn, 0, 0, 0, -tanl / re, 0;

  Matrix3 block3;  // dtilt = block3 * dpos
  block3 << -omega * sinl, 0, -ve / (re * re),  //
      0, 0, vn / (rn * rn),                     //
      -omega * cosl - ve / (re * cosl * cosl), 0, ve * tanl / (re * re);
  block3 = block3 * scalem2r;

  const Matrix3 block4 = nav::skew(force);  // dvel = block4 * dtilt

  Matrix3 block5;  // dvel = block5 * dvel
  block5 << vd / rn, -2 * (omega * sinl + ve / re * tanl), vn / rn,  //
      2 * omega * sinl + ve / re * tanl, 1 / re * (vn * tanl + vd), 2 * omega * cosl + ve / re,  //
      -2 * vn / rn, -2 * (omega * cosl + ve / re), 0;

  // Gravity gradient from the Schwartz model
  const double a1 = 9.7803267715, a2 = 0.0052790414, a3 = 0.0000232718;
  const double a4 = -3.0876910891e-6, a5 = 4.3977311e-9, a6 = 7.211e-13;
  const double lat = pos(0), alt = pos(2);
  const double dgdlat = 2 * a1 * a2 * std::cos(2 * lat) +
                        a1 * a3 * (12 * (1 - std::cos(4 * lat)) / 8 - std::pow(std::sin(lat), 4)) +
                        2 * a5 * (std::cos(2 * lat) - cosl * sinl) * alt;
  const double dgdalt = (a4 + a5 * sinl * sinl) + a6 * 2 * alt;

  Matrix3 block6;  // dvel = block6 * dpos
  block6 << -ve * (2 * omega * cosl + ve / (re * cosl * cosl)), 0, ve * ve * tanl / (re * re) - vn * vd / (rn * rn),  //
      (2 * omega * (vn * cosl - vd * sinl) + vn * ve / (re * cosl * cosl)), 0, -ve / (re * re) * (vn * tanl + vd),   //
      2 * omega * ve * sinl + dgdlat, 0, vn * vn / (rn * rn) + ve * ve / (re * re) + dgdalt;
  block6 = block6 * scalem2r;

  const Matrix3 block8 = Matrix3::Identity();  // dpos = block8 * dvel

  F.block<3, 3>(6, 6) = block1;
  F.block<3, 3>(6, 3) = block2;
  F.block<3, 3>(6, 0) = block3;
  F.block<3, 3>(3, 6) = block4;
  F.block<3, 3>(3, 3) = block5;
  F.block<3, 3>(3, 0) = block6;
  F.block<3, 3>(0, 3) = block8;

  F.block<3, 3>(3, 9) = C_sensor_to_ned;    // accel bias -> vdot
  F.block<3, 3>(6, 12) = -C_sensor_to_ned;  // gyro bias -> tiltdot

  for (int i = 0; i < 3; ++i) {
    F(9 + i, 9 + i) = -1.0 / imu_model_.accel_bias_tau[i];
    F(12 + i, 12 + i) = -1.0 / imu_model_.gyro_bias_tau[i];
  }
  return F;
}

Matrix Pinson15NedBlock::generate_q_pinson15() const {
  Matrix Q = pre_Q_;  // copy (the Python original rotated its stored matrix in place)
  const Matrix3 C = nav::quat_to_dcm(*utils::quaternion(*new_pva_aux_));
  Q.block<3, 3>(3, 3) = C * Q.block<3, 3>(3, 3) * C.transpose();
  Q.block<3, 3>(6, 6) = C * Q.block<3, 3>(6, 6) * C.transpose();
  return Q;
}

}  // namespace pntos::cobra
