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
#include <cmath>
#include <cstdlib>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

double EnvDouble(const char* name, const double default_value) {
  const char* value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return default_value;
  }
  char* end = nullptr;
  const double result = std::strtod(value, &end);
  return end == value ? default_value : result;
}

bool EnvBool(const char* name, const bool default_value) {
  const char* value = std::getenv(name);
  if (value == nullptr) {
    return default_value;
  }
  const std::string text(value);
  if (text == "1" || text == "true" || text == "TRUE" || text == "on") {
    return true;
  }
  if (text == "0" || text == "false" || text == "FALSE" || text == "off") {
    return false;
  }
  return default_value;
}

transform::Rigid3d InterpolateOdometry(
    const boost::circular_buffer<sensor::OdometryData>& odometry_data,
    const common::Time time,
    PoseExtrapolator::OdometrySourceInfo* const source_info = nullptr) {
  if (source_info != nullptr) {
    *source_info = PoseExtrapolator::OdometrySourceInfo{};
    source_info->has_data = !odometry_data.empty();
    source_info->requested_time = time;
    if (!odometry_data.empty()) {
      source_info->latest_time = odometry_data.back().time;
    }
  }
  transform::Rigid3d odom;
  if (time <= odometry_data.front().time) {
    if (source_info != nullptr) {
      source_info->clamped_to_earliest = true;
      source_info->before_time = odometry_data.front().time;
      source_info->after_time = odometry_data.front().time;
    }
    LOG(WARNING) << "No odometry data for time: " << time
                 << " (earliest: " << odometry_data.front().time << ")";
    return odometry_data.front().pose;
  }
  if (time >= odometry_data.back().time) {
    if (time == odometry_data.back().time) {
      if (source_info != nullptr) {
        source_info->before_time = odometry_data.back().time;
        source_info->after_time = odometry_data.back().time;
      }
      return odometry_data.back().pose;
    }
    auto prev_it = odometry_data.end() - 1;
    if (source_info != nullptr) {
      source_info->extrapolated_from_latest = true;
      source_info->before_time = prev_it->time;
    }
    const double t_diff = common::ToSeconds(time - prev_it->time);
    const Eigen::Quaterniond rot =
        Eigen::AngleAxisd(t_diff * prev_it->angular_velocity.x(),
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(t_diff * prev_it->angular_velocity.y(),
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(t_diff * prev_it->angular_velocity.z(),
                          Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d current_t =
        prev_it->pose.translation() + rot * (prev_it->linear_velocity * t_diff);
    const Eigen::Quaterniond current_r = rot * prev_it->pose.rotation();
    return transform::Rigid3d(current_t, current_r);
  }
  auto it = std::lower_bound(
      odometry_data.begin(), odometry_data.end(), time,
      [](const sensor::OdometryData& odometry_data,
         const common::Time target_time) {
        return odometry_data.time < target_time;
      });
  if (it == odometry_data.begin()) {
    if (source_info != nullptr) {
      source_info->clamped_to_earliest = true;
      source_info->before_time = it->time;
      source_info->after_time = it->time;
    }
    LOG(WARNING) << "No odometry data for time: " << time
                 << " (earliest: " << odometry_data.front().time << ")";
    odom = it->pose;
  } else if (it == odometry_data.end()) {
    auto prev_it = it - 1;
    if (source_info != nullptr) {
      source_info->extrapolated_from_latest = true;
      source_info->before_time = prev_it->time;
    }
    const double t_diff = common::ToSeconds(time - prev_it->time);
    const Eigen::Quaterniond rot =
        Eigen::AngleAxisd(t_diff * prev_it->angular_velocity.x(),
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(t_diff * prev_it->angular_velocity.y(),
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(t_diff * prev_it->angular_velocity.z(),
                          Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d current_t =
        prev_it->pose.translation() + rot * (prev_it->linear_velocity * t_diff);
    const Eigen::Quaterniond current_r = rot * prev_it->pose.rotation();
    odom = transform::Rigid3d(current_t, current_r);
  } else {
    auto prev_it = it - 1;
    if (source_info != nullptr) {
      source_info->before_time = prev_it->time;
      source_info->after_time = it->time;
    }
    odom = transform::Interpolate(
               transform::TimestampedTransform{prev_it->time, prev_it->pose},
               transform::TimestampedTransform{it->time, it->pose}, time)
               .transform;
  }
  return odom;
}

}  // namespace

PoseExtrapolator::PoseExtrapolator(
    const common::Duration pose_queue_duration,
    double /*imu_gravity_time_constant*/)
    : pose_queue_duration_(pose_queue_duration),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity(),
                                transform::Rigid3d::Identity()},
      adaptive_odometry_blend_(
          EnvBool("CARTOGRAPHER_ADAPTIVE_ODOMETRY_BLEND", true)),
      adaptive_odometry_longitudinal_only_(EnvBool(
          "CARTOGRAPHER_ADAPTIVE_ODOMETRY_LONGITUDINAL_ONLY", true)),
      adaptive_odometry_full_weight_yaw_rate_(EnvDouble(
          "CARTOGRAPHER_ADAPTIVE_ODOMETRY_FULL_WEIGHT_YAW_RATE", 0.05)),
      adaptive_odometry_zero_weight_yaw_rate_(EnvDouble(
          "CARTOGRAPHER_ADAPTIVE_ODOMETRY_ZERO_WEIGHT_YAW_RATE", 0.20)),
      adaptive_odometry_min_weight_(
          EnvDouble("CARTOGRAPHER_ADAPTIVE_ODOMETRY_MIN_WEIGHT", 0.)),
      adaptive_odometry_max_weight_(
          EnvDouble("CARTOGRAPHER_ADAPTIVE_ODOMETRY_MAX_WEIGHT", 1.)),
      adaptive_odometry_mismatch_override_(EnvBool(
          "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MISMATCH_OVERRIDE", true)),
      adaptive_odometry_mismatch_ratio_(EnvDouble(
          "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MISMATCH_RATIO", 0.35)),
      adaptive_odometry_min_forward_delta_(EnvDouble(
          "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MIN_FORWARD_DELTA", 0.005)),
      adaptive_odometry_mismatch_force_weight_(EnvDouble(
          "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MISMATCH_FORCE_WEIGHT", 1.)),
      imu_yaw_weight_(std::max(
          0., std::min(1., EnvDouble("CARTOGRAPHER_IMU_YAW_WEIGHT", 1.)))),
      odometry_data_(2000) {}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant,
    const sensor::ImuData& imu_data) {
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(Eigen::Quaterniond::Identity()));
  return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (!reference_pose_) {
    return common::Time::min();
  }
  return reference_pose_->time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  return cached_extrapolated_pose_.time;
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  reference_pose_ = absl::make_unique<TimedPose>(TimedPose{time, pose});
  last_pose_integrated_imu_yaw_ = integrated_imu_yaw_;
  pose_queue_.push_back(TimedPose{time, pose});
  while (pose_queue_.size() > 2 &&
         pose_queue_[1].time <= time - pose_queue_duration_) {
    pose_queue_.pop_front();
  }
  UpdateVelocityFromPoses();
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  if (last_imu_time_.has_value()) {
    const double dt = common::ToSeconds(imu_data.time - last_imu_time_.value());
    if (dt > 0. && dt < 1.) {
      integrated_imu_yaw_ += imu_data.angular_velocity.z() * dt;
    }
  }
  last_imu_time_ = imu_data.time;
  latest_imu_angular_velocity_z_ = imu_data.angular_velocity.z();
  has_imu_data_ = true;
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  odometry_data_.push_back(odometry_data);
}

transform::Rigid3d PoseExtrapolator::Odom(const common::Time time) const {
  if (odometry_data_.empty()) {
    return transform::Rigid3d::Identity();
  }
  return InterpolateOdometry(odometry_data_, time);
}

void PoseExtrapolator::UpdateVelocityFromPoses() {
  if (pose_queue_.size() < 2) {
    linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
    angular_velocity_from_poses_ = Eigen::Vector3d::Zero();
    return;
  }
  const TimedPose& oldest_pose = pose_queue_.front();
  const TimedPose& newest_pose = pose_queue_.back();
  const double queue_delta =
      common::ToSeconds(newest_pose.time - oldest_pose.time);
  if (queue_delta <= 0.) {
    linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
    angular_velocity_from_poses_ = Eigen::Vector3d::Zero();
    return;
  }
  linear_velocity_from_poses_ =
      (newest_pose.pose.translation() - oldest_pose.pose.translation()) /
      queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.pose.rotation().inverse() * newest_pose.pose.rotation()) /
      queue_delta;
}

double PoseExtrapolator::GetAdaptiveOdometryWeight() const {
  return last_adaptive_odometry_weight_;
}

double PoseExtrapolator::ComputeAdaptiveOdometryWeight() const {
  if (!adaptive_odometry_blend_) {
    return 1.;
  }
  const double yaw_rate =
      has_imu_data_
          ? std::abs(latest_imu_angular_velocity_z_)
          : (odometry_data_.empty()
                 ? 0.
                 : std::abs(odometry_data_.back().angular_velocity.z()));
  if (adaptive_odometry_zero_weight_yaw_rate_ <=
      adaptive_odometry_full_weight_yaw_rate_) {
    return adaptive_odometry_max_weight_;
  }
  const double yaw_confidence =
      1. - (yaw_rate - adaptive_odometry_full_weight_yaw_rate_) /
               (adaptive_odometry_zero_weight_yaw_rate_ -
                adaptive_odometry_full_weight_yaw_rate_);
  const double clamped_confidence =
      std::max(0., std::min(1., yaw_confidence));
  return adaptive_odometry_min_weight_ +
         clamped_confidence *
             (adaptive_odometry_max_weight_ - adaptive_odometry_min_weight_);
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  CHECK(reference_pose_);
  if (extrapolation_debug_enabled_) {
    last_extrapolation_debug_info_ = ExtrapolationDebugInfo{};
    last_extrapolation_debug_info_.valid = true;
    last_extrapolation_debug_info_.target_time = time;
    last_extrapolation_debug_info_.reference_pose_time = reference_pose_->time;
    last_extrapolation_debug_info_.imu_integration_start_time =
        reference_pose_->time;
    last_extrapolation_debug_info_.has_imu_data = has_imu_data_;
    if (last_imu_time_.has_value()) {
      last_extrapolation_debug_info_.latest_imu_time = last_imu_time_.value();
    }
    last_extrapolation_debug_info_.adaptive_odometry_weight =
        last_adaptive_odometry_weight_;
  }
  if (odometry_data_.empty()) {
    cached_extrapolated_pose_ = Extrapolation{
        time, reference_pose_->pose, transform::Rigid3d::Identity()};
    return cached_extrapolated_pose_.pose;
  }
  if (cached_extrapolated_pose_.time != time) {
    const TimedPose& newest_timed_pose = *reference_pose_;
    CHECK_GE(time, newest_timed_pose.time);
    CHECK(!odometry_data_.empty());
    OdometrySourceInfo* const reference_odom_debug =
        extrapolation_debug_enabled_
            ? &last_extrapolation_debug_info_.reference_odom
            : nullptr;
    OdometrySourceInfo* const current_odom_debug =
        extrapolation_debug_enabled_
            ? &last_extrapolation_debug_info_.current_odom
            : nullptr;
    const transform::Rigid3d reference_odom =
        InterpolateOdometry(odometry_data_, newest_timed_pose.time,
                            reference_odom_debug);
    const transform::Rigid3d current_odom =
        InterpolateOdometry(odometry_data_, time, current_odom_debug);
    const transform::Rigid3d odom_diff =
        reference_odom.inverse() * current_odom;
    const Eigen::Vector3d odom_translation_delta =
        newest_timed_pose.pose.rotation() * odom_diff.translation();
    Eigen::Quaterniond predicted_rotation = newest_timed_pose.pose.rotation();
    if (has_imu_data_) {
      const double imu_delta_yaw =
          integrated_imu_yaw_ - last_pose_integrated_imu_yaw_;
      predicted_rotation =
          newest_timed_pose.pose.rotation() *
          Eigen::AngleAxisd(imu_yaw_weight_ * imu_delta_yaw,
                            Eigen::Vector3d::UnitZ());
    } else if (pose_queue_.size() >= 2) {
      const double extrapolation_delta =
          common::ToSeconds(time - newest_timed_pose.time);
      const Eigen::Vector3d rotation_vector =
          extrapolation_delta * angular_velocity_from_poses_;
      predicted_rotation =
          newest_timed_pose.pose.rotation() *
          transform::AngleAxisVectorToRotationQuaternion(rotation_vector);
    }
    transform::Rigid3d extrapolated(
        newest_timed_pose.pose.translation() + odom_translation_delta,
        predicted_rotation);
    if (adaptive_odometry_blend_ && pose_queue_.size() >= 2) {
      double odom_weight = ComputeAdaptiveOdometryWeight();
      const double extrapolation_delta =
          common::ToSeconds(time - newest_timed_pose.time);
      const Eigen::Vector3d scan_delta =
          extrapolation_delta * linear_velocity_from_poses_;
      const Eigen::Vector3d odom_delta =
          extrapolated.translation() - newest_timed_pose.pose.translation();
      Eigen::Vector3d blended_delta;
      if (adaptive_odometry_longitudinal_only_) {
        const Eigen::Vector3d heading =
            newest_timed_pose.pose.rotation() * Eigen::Vector3d::UnitX();
        const double scan_forward = scan_delta.dot(heading);
        const double odom_forward = odom_delta.dot(heading);
        if (adaptive_odometry_mismatch_override_ &&
            odom_forward > adaptive_odometry_min_forward_delta_ &&
            (scan_forward <= 0. ||
             scan_forward <
                 adaptive_odometry_mismatch_ratio_ * odom_forward)) {
          odom_weight =
              std::max(odom_weight, adaptive_odometry_mismatch_force_weight_);
        }
        odom_weight = std::max(
            adaptive_odometry_min_weight_,
            std::min(adaptive_odometry_max_weight_, odom_weight));
        last_adaptive_odometry_weight_ = odom_weight;
        const Eigen::Vector3d scan_lateral =
            scan_delta - scan_forward * heading;
        const double blended_forward =
            (1. - odom_weight) * scan_forward + odom_weight * odom_forward;
        blended_delta = scan_lateral + blended_forward * heading;
      } else {
        last_adaptive_odometry_weight_ = odom_weight;
        blended_delta =
            (1. - odom_weight) * scan_delta + odom_weight * odom_delta;
      }
      extrapolated = transform::Rigid3d(
          newest_timed_pose.pose.translation() + blended_delta,
          extrapolated.rotation());
    }
    cached_extrapolated_pose_ = Extrapolation{time, extrapolated, odom_diff};
  }
  if (extrapolation_debug_enabled_) {
    last_extrapolation_debug_info_.adaptive_odometry_weight =
        last_adaptive_odometry_weight_;
  }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time /*time*/) {
  return Eigen::Quaterniond::Identity();
}

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }
  return ExtrapolationResult{
      poses, ExtrapolatePose(times.back()), GetOdometryLinearVelocity(),
      EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
