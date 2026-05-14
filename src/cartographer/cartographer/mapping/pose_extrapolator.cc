/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <fstream>
#include <unordered_map>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"
#include <cstdlib>
#include <string>





namespace cartographer {
namespace mapping {

namespace {

std::string Trim(const std::string& value) {
  size_t begin = 0;
  while (begin < value.size() &&
         std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }

  size_t end = value.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }

  return value.substr(begin, end - begin);
}

std::unordered_map<std::string, double> LoadFlatDoubleConfig(
    const std::string& config_path) {
  std::unordered_map<std::string, double> values;
  std::ifstream file(config_path);
  if (!file.is_open()) {
    LOG(WARNING) << "Could not open pose extrapolator config: "
                 << config_path;
    return values;
  }

  std::string line;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    const size_t comment_position = line.find('#');
    if (comment_position != std::string::npos) {
      line = line.substr(0, comment_position);
    }

    line = Trim(line);
    if (line.empty()) {
      continue;
    }

    const size_t colon_position = line.find(':');
    if (colon_position == std::string::npos) {
      LOG(WARNING) << "Ignoring pose extrapolator config line without ':' at "
                   << config_path << ":" << line_number;
      continue;
    }

    const std::string key = Trim(line.substr(0, colon_position));
    std::string raw_value = Trim(line.substr(colon_position + 1));
    if (key.empty() || raw_value.empty()) {
      continue;
    }

    if (raw_value.size() >= 2 &&
        ((raw_value.front() == '"' && raw_value.back() == '"') ||
         (raw_value.front() == '\'' && raw_value.back() == '\''))) {
      raw_value = raw_value.substr(1, raw_value.size() - 2);
    }

    try {
      size_t parsed_chars = 0;
      const double parsed_value = std::stod(raw_value, &parsed_chars);
      if (!Trim(raw_value.substr(parsed_chars)).empty()) {
        LOG(WARNING) << "Ignoring non-numeric pose extrapolator config value "
                     << "at " << config_path << ":" << line_number << " for "
                     << key << ": " << raw_value;
        continue;
      }
      values[key] = parsed_value;
    } catch (const std::exception& exception) {
      LOG(WARNING) << "Ignoring invalid pose extrapolator config value at "
                   << config_path << ":" << line_number << " for " << key
                   << ": " << raw_value << " (" << exception.what() << ")";
    }
  }

  return values;
}

double ConfigValue(const std::unordered_map<std::string, double>& config,
                   const std::string& key, const double fallback) {
  const auto it = config.find(key);
  return it == config.end() ? fallback : it->second;
}

void SetIsotropicNoise(Eigen::Matrix2d* matrix, const double value) {
  *matrix = Eigen::Matrix2d::Identity() * value;
}

}  // namespace

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {
  // Respect environment variable set by launch file if present.
  const char* env_val = std::getenv("FUSION_EXTRPOLATOR");
  if (env_val != nullptr) {
    std::string v(env_val);
    for (auto& c : v) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (v == "false" || v == "0" || v == "off") {
      fusion_extrpolator = false;
    } else if (v == "true" || v == "1" || v == "on") {
      fusion_extrpolator = true;
    }
  }

  LoadFusionConfigFromYaml();

  LOG(INFO) << "fusion is " << (fusion_extrpolator ? "active" : "inactive");
}

void PoseExtrapolator::LoadFusionConfigFromYaml() {
  const char* config_path_env = std::getenv("POSE_EXTRAPOLATOR_CONFIG");
  if (config_path_env == nullptr || std::string(config_path_env).empty()) {
    config_path_env = std::getenv("WHEEL_ODOM_CONFIG");
  }
  if (config_path_env == nullptr || std::string(config_path_env).empty()) {
    LOG(INFO) << "POSE_EXTRAPOLATOR_CONFIG/WHEEL_ODOM_CONFIG is not set. "
              << "Using built-in pose extrapolator fusion defaults.";
    return;
  }

  const std::string config_path(config_path_env);
  const auto config = LoadFlatDoubleConfig(config_path);
  if (config.empty()) {
    LOG(WARNING) << "Pose extrapolator config has no numeric values. "
                 << "Using built-in/default values. path=" << config_path;
    return;
  }

  const double process_noise_value =
      ConfigValue(config, "process_noise", process_noise(0, 0));
  measurement_noise_scan_default =
      ConfigValue(config, "measurement_noise_scan_default",
                  measurement_noise_scan_default);
  measurement_noise_odom_default =
      ConfigValue(config, "measurement_noise_odom_default",
                  measurement_noise_odom_default);
  measurement_noise_scan_low_score_or_straight =
      ConfigValue(config, "measurement_noise_scan_low_score_or_straight",
                  measurement_noise_scan_low_score_or_straight);
  measurement_noise_odom_low_score_or_straight =
      ConfigValue(config, "measurement_noise_odom_low_score_or_straight",
                  measurement_noise_odom_low_score_or_straight);
  measurement_noise_scan_high_score_or_curve =
      ConfigValue(config, "measurement_noise_scan_high_score_or_curve",
                  measurement_noise_scan_high_score_or_curve);
  measurement_noise_odom_high_score_or_curve =
      ConfigValue(config, "measurement_noise_odom_high_score_or_curve",
                  measurement_noise_odom_high_score_or_curve);
  scan_match_low_score_threshold =
      ConfigValue(config, "scan_match_low_score_threshold",
                  scan_match_low_score_threshold);
  scan_match_high_score_threshold =
      ConfigValue(config, "scan_match_high_score_threshold",
                  scan_match_high_score_threshold);
  straight_yaw_speed_threshold =
      ConfigValue(config, "straight_yaw_speed_threshold",
                  straight_yaw_speed_threshold);
  curve_yaw_speed_threshold =
      ConfigValue(config, "curve_yaw_speed_threshold",
                  curve_yaw_speed_threshold);

  // config.yaml에서 실제로 자주 튜닝할 local fusion 값들이다.
  imu_weight = ConfigValue(config, "imu_weight", imu_weight);
  imu_delta_min = ConfigValue(config, "imu_delta_min", imu_delta_min);
  wheelodom_weight = ConfigValue(config, "wheelodom_weight", wheelodom_weight);

  SetIsotropicNoise(&process_noise, process_noise_value);
  SetIsotropicNoise(&measurement_noise_scan, measurement_noise_scan_default);
  SetIsotropicNoise(&measurement_noise_odom, measurement_noise_odom_default);

  LOG(INFO) << "Loaded pose extrapolator config from " << config_path
            << " imu_weight=" << imu_weight
            << " imu_delta_min=" << imu_delta_min
            << " wheelodom_weight=" << wheelodom_weight
            << " process_noise=" << process_noise_value
            << " measurement_noise_scan_default="
            << measurement_noise_scan_default
            << " measurement_noise_odom_default="
            << measurement_noise_odom_default;
}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();

  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  if (fusion_extrpolator) {
    // Fusion-enabled behavior: integrate IMU and compute imu_delta_velocity.

    if (imu_velocity_initalized && imu_data.time <= last_imu_time) {
      LOG(WARNING) << "Received IMU data with non-increasing timestamp. Ignoring." << imu_data.time << " <= " << last_imu_time;
      return;
    }

    if (!timed_pose_queue_.empty() && imu_data.time < timed_pose_queue_.back().time) {
      LOG(WARNING) << "Received IMU data with timestamp earlier than last pose. Ignoring." << imu_data.time << " < " << timed_pose_queue_.back().time;
      return;
    }

    imu_data_.push_back(imu_data);

    // If IMU tracker not yet created, nothing more to do here.
    if (imu_tracker_ == nullptr) {
      TrimImuData();
      return;
    }

    if (!imu_velocity_initalized) {
      const Eigen::Quaterniond current_orientation = imu_tracker_->orientation();
      const Eigen::Vector3d world_frame_linear_acceleration = current_orientation * imu_data.linear_acceleration;
      const Eigen::Vector3d world_frame_considered_gravity_linear_acceleration = world_frame_linear_acceleration - Eigen::Vector3d(0.0, 0.0, 9.806);
      prev_linear_acceleration = world_frame_considered_gravity_linear_acceleration;
      imu_delta_velocity.setZero();
      last_imu_time = imu_data.time;
      imu_velocity_initalized = true;
      TrimImuData();
      return;
    }

    const double delta_time = common::ToSeconds(imu_data.time - last_imu_time);
    if (delta_time <= 0.0) {
      TrimImuData();
      return;
    }

    const Eigen::Quaterniond current_orientation = imu_tracker_->orientation();
    const Eigen::Vector3d world_frame_linear_acceleration = current_orientation * imu_data.linear_acceleration;
    const Eigen::Vector3d world_frame_considered_gravity_linear_acceleration = world_frame_linear_acceleration - Eigen::Vector3d(0.0, 0.0, 9.806);
    const Eigen::Vector3d current_linear_acceleration = world_frame_considered_gravity_linear_acceleration;
    imu_delta_velocity = (prev_linear_acceleration + current_linear_acceleration) * 0.5 * delta_time;
    imu_delta_velocity.z() = 0.0;

    if (imu_delta_velocity.norm() > imu_delta_min) {
      imu_delta_velocity.setZero();
    }

    prev_linear_acceleration = current_linear_acceleration;
    last_imu_time = imu_data.time;
    TrimImuData();
    return;
  }

  // ORIGINAL behavior: simple enqueue and trim.
  CHECK(timed_pose_queue_.empty() || imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_newest.time - odometry_data_oldest.time);
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_oldest.pose.inverse() * odometry_data_newest.pose;
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;

  //translation_fusion(odometry_data.time, nullptr,
    //                 &linear_velocity_from_odometry_);
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // fake gravity and an angular velocity fallback for 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    if (fusion_extrpolator) {
      // Fusion-enabled behavior: prefer pose-derived angular velocity.
      imu_tracker->AddImuAngularVelocityObservation(angular_velocity_from_poses_);
    } else {
      // Original behavior: use odometry when available, otherwise pose-derived.
      imu_tracker->AddImuAngularVelocityObservation(
          odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                    : angular_velocity_from_odometry_);
    }
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}
///////////////////////////Reliabilty of sensor and control noise ////////////////
void PoseExtrapolator::ScanMatchScore(double score){
  scan_match_score = score;
  if(score<scan_match_low_score_threshold){
    measurement_noise_scan =
        Eigen::Matrix2d::Identity() * measurement_noise_scan_low_score_or_straight;
    measurement_noise_odom =
        Eigen::Matrix2d::Identity() * measurement_noise_odom_low_score_or_straight;
    return;
  }
  if(score>scan_match_high_score_threshold){
    measurement_noise_scan =
        Eigen::Matrix2d::Identity() * measurement_noise_scan_high_score_or_curve;
    measurement_noise_odom =
        Eigen::Matrix2d::Identity() * measurement_noise_odom_high_score_or_curve;
    return;
  }

  measurement_noise_scan =
      Eigen::Matrix2d::Identity() * measurement_noise_scan_default;
  measurement_noise_odom =
      Eigen::Matrix2d::Identity() * measurement_noise_odom_default;
  }

// Q= 1e-3이면:

// - q = 2e-5
// - 5% 반영 -> R_odom ≈ 2.0e-4
// - 10% 반영 -> R_odom ≈ 1.2e-4
// - 1% 반영 -> R_odom ≈ 1.0e-3
// - 정상 score 구간: scan **95% R_scan = 1.5e-6**
// - 낮은 score 구간: scan **80~85% R_scan = 5.5 e-6**
// - 높은 score 구간: scan **97~99% 5.0e-7**


void PoseExtrapolator::Reliability_sensor(){
  yaw_speed = std::abs(angular_velocity_from_poses_.z());
  
  if (scan_match_score>scan_match_low_score_threshold &&
      scan_match_score<scan_match_high_score_threshold){

    if(yaw_speed<straight_yaw_speed_threshold){
      measurement_noise_scan =
          Eigen::Matrix2d::Identity() * measurement_noise_scan_low_score_or_straight;
      measurement_noise_odom =
          Eigen::Matrix2d::Identity() * measurement_noise_odom_low_score_or_straight;
      return;
    }
  
    if(yaw_speed>curve_yaw_speed_threshold){
      measurement_noise_scan =
          Eigen::Matrix2d::Identity() * measurement_noise_scan_high_score_or_curve;
      measurement_noise_odom =
          Eigen::Matrix2d::Identity() * measurement_noise_odom_high_score_or_curve;
      return;
    }
  
  
    measurement_noise_scan =
        Eigen::Matrix2d::Identity() * measurement_noise_scan_default;
    measurement_noise_odom =
        Eigen::Matrix2d::Identity() * measurement_noise_odom_default;
    return;
  }
    return;


  // - q = 2e-5
// - 5% 반영 -> R_odom ≈ 2.0e-4
// - 10% 반영 -> R_odom ≈ 1.2e-4
// - 1% 반영 -> R_odom ≈ 1.0e-3
// - 정상 구간: scan **95% R_scan = 1.5e-6**
// - 직선 구간: scan **80~85% R_scan = 5.5 e-6**
// - 커브 구간: scan **97~99% 5.0e-7**
}


////////////////////////////////////////////////////////


//////////////////////////fusion /////////////////////////////
Eigen::Vector3d PoseExtrapolator::translation_fusion(
    common::Time time, const Eigen::Vector3d* linear_velocity_scan,
    const Eigen::Vector3d* linear_velocity_odom) {
  if (linear_velocity_scan == nullptr && linear_velocity_odom == nullptr) {
    return Eigen::Vector3d(fusion_linear_velocity.x(), fusion_linear_velocity.y(),
                           0.0);
  }

  auto update = [&](const Eigen::Vector2d& measurement,
                    const Eigen::Matrix2d& measurement_noise) {
    const Eigen::Matrix2d prediction_covariance =
        velocity_covariance + measurement_noise;
    const Eigen::Matrix2d kalman_gain =
        velocity_covariance * prediction_covariance.inverse();
    const Eigen::Vector2d difference = measurement - fusion_linear_velocity;
    fusion_linear_velocity += kalman_gain * difference;
    velocity_covariance =
        (Eigen::Matrix2d::Identity() - kalman_gain) * velocity_covariance;
  };

  if (!velocity_filter_initalized) {
    if (linear_velocity_scan != nullptr) {
      fusion_linear_velocity = linear_velocity_scan->head<2>();
    } else {
      fusion_linear_velocity = linear_velocity_odom->head<2>();
    }
    velocity_covariance = Eigen::Matrix2d::Identity() * 1e-2;
    last_velocity_time = time;
    velocity_filter_initalized = true;

    if (linear_velocity_odom != nullptr) {
      update(linear_velocity_odom->head<2>(), measurement_noise_odom);
    }
    return Eigen::Vector3d(fusion_linear_velocity.x(), fusion_linear_velocity.y(),
                           0.0);
  }

  const double time_diff = common::ToSeconds(time - last_velocity_time);
  if (time_diff > 0.0) {
    velocity_covariance += process_noise * time_diff;
    last_velocity_time = time;
  }

  if (linear_velocity_scan != nullptr) {
    Reliability_sensor();
    update(linear_velocity_scan->head<2>(), measurement_noise_scan);
  }
  if (linear_velocity_odom != nullptr) {
    Reliability_sensor();
    update(linear_velocity_odom->head<2>(), measurement_noise_odom);
  }

  return Eigen::Vector3d(fusion_linear_velocity.x(), fusion_linear_velocity.y(),
                         0.0);
}


///////////////////////////////////////Wheel ODOM IMU fix //////////////////

Eigen::Vector3d PoseExtrapolator::translation_imu_wheel(const Eigen::Vector3d* linear_velocity_scan, const Eigen::Vector3d* linear_velocity_odom){

  if (linear_velocity_scan == nullptr){
    return Eigen::Vector3d::Zero();
  }
  if (linear_velocity_odom == nullptr){
    return Eigen::Vector3d(linear_velocity_scan->x(), linear_velocity_scan->y(), 0.0);
  }
  
  // wheel odom이 들어오는 속도와 imu가 보내는 속도와 scan이 보내는 속도가 달라서 맞춰줄 필요가 있다

  const Eigen::Vector2d velocity_from_scan_imu = linear_velocity_scan->head<2>();
  const Eigen::Vector2d velocity_from_odom = linear_velocity_odom->head<2>();

  if(velocity_from_scan_imu.norm() > 1e-6 && velocity_from_odom.norm() > 1e-6){
    const Eigen::Vector2d direction = velocity_from_scan_imu.normalized();
    //현재 속도에 대한 단위 벡터를 구해 현재 속도에 대한 방향만 구한다
    const Eigen::Vector2d velocity_diff_from_odom = velocity_from_odom - velocity_from_scan_imu;
    // wheel odom과 imu로 보정한 속도 사이의 차이를 구한다
    //imu로 보정한 scan 속도에 wheel odom으로 측정한 속도가 맞지 않을 경우 해당 차이에 대하여 보정한다
    const Eigen::Vector2d velocity_diff_consider = velocity_diff_from_odom.dot(direction) * direction;
    const Eigen::Vector2d velocity_from_wheel_fusion = velocity_diff_consider * wheelodom_weight;
    const Eigen::Vector2d linear_velocity_from_fusion = velocity_from_scan_imu + velocity_from_wheel_fusion;
    return Eigen::Vector3d(linear_velocity_from_fusion.x(), linear_velocity_from_fusion.y(),
                         0.0);
  }

  return Eigen::Vector3d(velocity_from_scan_imu.x(), velocity_from_scan_imu.y(), 0.0);

 
  }

////////////////////////////////////////////////////////////////////////




Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  if (fusion_extrpolator) {
    // Fusion-enabled behavior: fuse scan-based velocity with IMU delta and
    // optional wheel odometry correction.
    ///////////////fusion velocity wheel odom /////////////
    // if (velocity_filter_initalized) {
    //   return Eigen::Vector3d(extrapolation_delta * fusion_linear_velocity.x(),
    //                          extrapolation_delta * fusion_linear_velocity.y(),
    //                          0.0);
    // }
    /////////////////////////////////////////////

    Eigen::Vector2d velocity_from_scan = linear_velocity_from_poses_.head<2>();
    Eigen::Vector2d delta_velocity_from_imu = imu_delta_velocity.head<2>();
    const Eigen::Vector2d scan_direction = velocity_from_scan.normalized();
    const double imu_delta_scalar = delta_velocity_from_imu.dot(scan_direction);
    const Eigen::Vector2d imu_delta_velocity = imu_delta_scalar * scan_direction;
    const Eigen::Vector2d scan_velocity_with_imu = velocity_from_scan + imu_weight * imu_delta_velocity;
    const Eigen::Vector3d velocity_scan_based = {scan_velocity_with_imu.x(), scan_velocity_with_imu.y(), 0.0};

    if (odometry_data_.size() < 2) {
      return extrapolation_delta * velocity_scan_based;
    }
    return extrapolation_delta * translation_imu_wheel(&velocity_scan_based, &linear_velocity_from_odometry_);
  }

  // Original behavior: simple extrapolation using pose- or odometry-derived
  // linear velocity.
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }
  if (fusion_extrpolator) {
    Eigen::Vector2d velocity_from_scan = linear_velocity_from_poses_.head<2>();
    Eigen::Vector2d delta_velocity_from_imu = imu_delta_velocity.head<2>();
    const Eigen::Vector2d scan_direction = velocity_from_scan.normalized();
    const double imu_delta_scalar = delta_velocity_from_imu.dot(scan_direction);
    const Eigen::Vector2d imu_delta_velocity = imu_delta_scalar * scan_direction;
    const Eigen::Vector2d scan_velocity_with_imu = velocity_from_scan + imu_weight * imu_delta_velocity;
    const Eigen::Vector3d velocity_scan_based = {scan_velocity_with_imu.x(), scan_velocity_with_imu.y(), 0.0};

    const Eigen::Vector3d current_velocity =
        (odometry_data_.size() < 2 ? velocity_scan_based
                                   : translation_imu_wheel(&velocity_scan_based, &linear_velocity_from_odometry_));
    return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                               current_velocity,
                               EstimateGravityOrientation(times.back())};
  }

  // ORIGINAL behavior
  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
