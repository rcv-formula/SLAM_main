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

#include "absl/memory/memory.h"
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

transform::Rigid3d InterpolateOdometry(
    const boost::circular_buffer<sensor::OdometryData>& odometry_data,
    const common::Time time) {
  transform::Rigid3d odom;
  auto it = odometry_data.begin();
  while (it != odometry_data.end() && it->time < time) {
    ++it;
  }
  if (it == odometry_data.begin()) {
    LOG(WARNING) << "No odometry data for time: " << time
                 << " (earliest: " << odometry_data.front().time << ")";
    odom = it->pose;
  } else if (it == odometry_data.end()) {
    auto prev_it = it - 1;
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
    odom = transform::Interpolate(
               transform::TimestampedTransform{prev_it->time, prev_it->pose},
               transform::TimestampedTransform{it->time, it->pose}, time)
               .transform;
  }
  return odom;
}

}  // namespace

PoseExtrapolator::PoseExtrapolator(
    const common::Duration /*pose_queue_duration*/,
    double /*imu_gravity_time_constant*/)
    : cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity(),
                                transform::Rigid3d::Identity()},
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
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& /*imu_data*/) {}

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

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  CHECK(reference_pose_);
  if (odometry_data_.empty()) {
    cached_extrapolated_pose_ = Extrapolation{
        time, reference_pose_->pose, transform::Rigid3d::Identity()};
    return cached_extrapolated_pose_.pose;
  }
  if (cached_extrapolated_pose_.time != time) {
    const TimedPose& newest_timed_pose = *reference_pose_;
    CHECK_GE(time, newest_timed_pose.time);
    CHECK(!odometry_data_.empty());
    const transform::Rigid3d reference_odom =
        InterpolateOdometry(odometry_data_, newest_timed_pose.time);
    const transform::Rigid3d current_odom =
        InterpolateOdometry(odometry_data_, time);
    const transform::Rigid3d odom_diff =
        reference_odom.inverse() * current_odom;
    const transform::Rigid3d extrapolated = newest_timed_pose.pose * odom_diff;
    cached_extrapolated_pose_ = Extrapolation{time, extrapolated, odom_diff};
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
