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

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"





namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

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
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  
  //아직 imu tracker가 구성되지 않았다면 함수를 벗어난다. imu tracker가 orientaion을 계산한다
  if (imu_tracker_ == nullptr) {
      TrimImuData();
    return;
  }
  
  // 처음 imu 데이터가 들어왔으며 imu tracker가 구성되어 있다면 초기 값을 저장하고 속도 변화량은 계산하지 않는다
  //두개의 acceleration이 존재해야 변화량을 추정할 수 있다.
  if (imu_velocity_initalized == false){
    //body frame의 orienation을 현재의 worldframe orienation으로 바꾼다
    AdvanceImuTracker(imu_data.time, imu_tracker_.get()); //Advacned imu tracker를 update하여 현재 시점에서 orienation을 계산한다
    
    //현재 시점에서의 world frame 기준 orientaion을 저장한다
    const Eigen::Quaterniond current_orientation = imu_tracker_->orientation();

    //imu를 통해 얻은 tranlsation 가속도를 world frame 기준으로 가져온다. orientation을 통해 body frame에서 world frame으로 바꾼다.
    //imu 데이터를 world frame으로 변환
    const Eigen::Vector3d world_frame_linear_acceleration = current_orientation * imu_data.linear_acceleration;
    //imu 데이터는 time linear accleration, angular velocity가 저장된다.

    //중력을 제거한다.
    const Eigen::Vector3d world_frame_considered_gravity_linear_acceleration = world_frame_linear_acceleration - Eigen::Vector3d(0.0,0.0,9.806);
    //현재 시점의 linear velocity를 최근 속도로 저장
    prev_linear_acceleration = world_frame_considered_gravity_linear_acceleration;
    //imu로 보정한 속도 변화량에 대하여 초기화 한다.
    imu_delta_velocity.setZero();
    //현재 시간을 측정한 최근 시간으로 저장한다
    last_imu_time = imu_data.time;
    //imu 속도를 구하기 위한 절차가 되었다고 저장한다
    imu_velocity_initalized = true;

    TrimImuData();
    return;

  }

  // imu 속도가 초기화 되었다면 속도 변화량 계산을 한다

  //현재 시간과 가장 최근에 측정한 시간의 차이를 계산해서 적분을 위한 시간 변화량을 찾는다
  const double delta_time = common::ToSeconds(imu_data.time - last_imu_time);
  if (delta_time <= 0.0){
    TrimImuData();
    return;
  } // 시간이 거꾸로 흐르거나 시간 간격이 너무 짧으면 계산하지 않는다

  //현재에 IMU시간에 맞게 Imu tracker를 바꾼다
  AdvanceImuTracker(imu_data.time, imu_tracker_.get());
  // 현재 시점에서 orientaion을 계산한다
  const Eigen::Quaterniond current_orientation = imu_tracker_->orientation();
  //imu에서 얻은 accelreation을 world를 기준으로 frame을 변환
  const Eigen::Vector3d world_frame_linear_acceleration = current_orientation * imu_data.linear_acceleration;
  //중력 제거
  const Eigen::Vector3d world_frame_considered_gravity_linear_acceleration = world_frame_linear_acceleration - Eigen::Vector3d(0.0,0.0,9.806);
  //현재 시점에서의 속도를 저장한다
  const Eigen::Vector3d current_linear_acceleration = world_frame_considered_gravity_linear_acceleration;
  // 적분을 통해 최근에 저장한 시점과 현재 시점에서의 속도 변화량을 파악한다
  imu_delta_velocity = (prev_linear_acceleration + current_linear_acceleration) *0.5 * delta_time;
  //z는 평면에서 사용하지 않으므로 0으로 설정한다
  imu_delta_velocity.z() = 0.0;
  
  //너무 강한 속도 변화량이 가해지는 경우 최대 설정 속도 변화량으로 제한한다
  //x방향 y방향의 통합된 속도
  // const norm_xy = imu_delta_velocity.head<2>().norm();
  // //방향은 유지하면서 제한한 최대 속도 변화량으로 설정한다
  // if (norm_xy > imu_delta_clip){
  //   imu_delta_velocity.head<2>() = (imu_delta_velocity.head<2>() / norm_xy) * imu_delta_clip;
  // }
  // 현재 추정되어진 가속도를 다음 속도 변화량 파악에 사용하기 위헤 최근 가속도로 저장한다
  prev_linear_acceleration = current_linear_acceleration;
  // 현재 시간을 다음 파악에 사용하기 위해 최근 시간으로 저장한다
  last_imu_time = imu_data.time;
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
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
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
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(angular_velocity_from_poses_);
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
  if(score<0.6){
    measurement_noise_scan = Eigen::Matrix2d::Identity() * 5.5e-6;
    measurement_noise_odom = Eigen::Matrix2d::Identity() * 1.2e-4;
    return;
  }
  if(score>0.9){
    measurement_noise_scan = Eigen::Matrix2d::Identity() * 5.0e-7;
    measurement_noise_odom = Eigen::Matrix2d::Identity() * 1.0e-3;
    return;
  }

  measurement_noise_scan = Eigen::Matrix2d::Identity() * 1.5e-6;
  measurement_noise_odom = Eigen::Matrix2d::Identity() * 2.0e-4;
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
  yaw_speed = abs(angular_velocity_from_poses_.z());
  
  if (scan_match_score>0.6 && scan_match_score<0.9){

    if(yaw_speed<0.15){
      measurement_noise_scan = Eigen::Matrix2d::Identity() * 5.5e-6;
      measurement_noise_odom = Eigen::Matrix2d::Identity() * 1.2e-4;
      return;
    }
  
    if(yaw_speed>0.41){
      measurement_noise_scan = Eigen::Matrix2d::Identity() * 5.0e-7;
      measurement_noise_odom = Eigen::Matrix2d::Identity() * 1.0e-3;
      return;
    }
  
  
    measurement_noise_scan = Eigen::Matrix2d::Identity() * 1.5e-6;
    measurement_noise_odom = Eigen::Matrix2d::Identity() * 2.0e-4;
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


Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  ///////////////fusion velocity wheel odom /////////////
  // if (velocity_filter_initalized) {
  //   return Eigen::Vector3d(extrapolation_delta * fusion_linear_velocity.x(),
  //                          extrapolation_delta * fusion_linear_velocity.y(),
  //                          0.0);
  // }
  /////////////////////////////////////////////

  Eigen::Vector2d velocity_from_scan = linear_velocity_from_poses_.head<2>();
  //scan을 통해 얻은 translation velocity
  Eigen::Vector2d delta_velocity_from_imu = imu_delta_velocity.head<2>();
  //imu를 통해 계산한 delta velocity

  //현재 world frame으로 scan기반 속도와 imu 기반 속도 변화가 맞추어져 있지만 imu의 센서 노이즈 및 누적 오차로 인해
  //scan 기반 속도의 방향과 imu 기반 속도 변화의 방향이 맞이 않을 수 있다.
  //그래서 scan 기반 속도의 방향에 대한 단위 벡터를 추출하고 이를 imu 기반 속도 변화랑 scan에 대한 단위 벡터를 내적하여
  // scan 방향으로의 imu 기반 속도 변화량 크기를 구하고 이를 scan 방향과 곱해서 scan 방향으로의 
  //imu 기반 속도 변화량을 계산한다.
  const Eigen::Vector2d scan_direction = velocity_from_scan.normalized();
  // scan에 대한 속도의 단위 벡터를 구한다
  // 이는 scan의 방향이 된다.
  const double imu_delta_scalar = delta_velocity_from_imu.dot(scan_direction);
  // imu 속도를 scan 방향에 대하여 내적하여 scan 방향으로의 속도 변화량 크기에 대해 구한다
  const Eigen::Vector2d imu_delta_velocity = imu_delta_scalar * scan_direction;
  // imu 속도 변화량의 크기에 scan 방향을 곱하여 scan 방향의 imu 속도 변화량을 구한다.

  const Eigen::Vector2d scan_velocity_with_imu = velocity_from_scan + imu_weight * imu_delta_velocity;
  //scan을 통해 얻은 속도와 imu를 통해 얻은 속도 변화량을 합한다

  const Eigen::Vector3d velocity_scan_based = {scan_velocity_with_imu.x(), scan_velocity_with_imu.y(), 0.0};

  if (odometry_data_.size() <2){
    return extrapolation_delta * velocity_scan_based;
    //imu를 통해 얻은 속도와 시간을 곱해서 현재 tranlsation 변화량을 파악한다.
  }
  return extrapolation_delta * translation_fusion(time, &velocity_scan_based, &linear_velocity_from_odometry_);

//   if (odometry_data_.size() < 2) {
//     return extrapolation_delta * linear_velocity_from_poses_;
//   }
//   return extrapolation_delta * translation_fusion(time,&linear_velocity_from_poses_, &linear_velocity_from_odometry_);
  }

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  const Eigen::Vector3d current_velocity =
      velocity_filter_initalized
          ? Eigen::Vector3d(fusion_linear_velocity.x(),
                            fusion_linear_velocity.y(), 0.0)
          : (odometry_data_.size() < 2 ? linear_velocity_from_poses_
                                       : linear_velocity_from_odometry_);
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
