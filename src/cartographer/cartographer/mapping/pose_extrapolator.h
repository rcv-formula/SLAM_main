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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
class PoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;

  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;
  //////////////////////reliability of sensor data ///////////////////
  void ScanMatchScore(double score);
  double scan_match_score = 1.0;
  void Reliability_sensor();
  double yaw_speed = 0.0;
  ////////////////////////////////////////////////////

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  ///////////////////////////////////////////////////
  // Fusion state for scan- and odom-derived planar velocities.
  bool velocity_filter_initalized = false;
  common::Time last_velocity_time = common::Time::min();
  Eigen::Vector2d fusion_linear_velocity = Eigen::Vector2d::Zero();
  Eigen::Matrix2d velocity_covariance =
      Eigen::Matrix2d::Identity() * 1e-2;

//setting Q = 1e-3
//straight line: R_scan = 1.5e-6, R_odom = 1.0e-3
//curve line: R_scan = 5.0e-7, R_odom = 1.0e-3
  Eigen::Matrix2d process_noise = Eigen::Matrix2d::Identity() * 1e-3;
  Eigen::Matrix2d measurement_noise_scan =
      Eigen::Matrix2d::Identity() * 1.5e-6;
  Eigen::Matrix2d measurement_noise_odom =
      Eigen::Matrix2d::Identity() * 2.0e-4;

  Eigen::Vector3d translation_fusion(
      common::Time time, const Eigen::Vector3d* linear_velocity_scan,
      const Eigen::Vector3d* linear_velocity_odom);


// Q= 1e-3이면:

// - q = 2e-5
// - 5% 반영 -> R_odom ≈ 2.0e-4
// - 10% 반영 -> R_odom ≈ 1.2e-4
// - 1% 반영 -> R_odom ≈ 1.0e-3
// - 평소 구간: scan **95% R_scan = 1.5e-6**
// - 직선 구간: scan **80~85% R_scan = 5.5 e-6**
// - 곡선 구간: scan **97~99% 5.0e-7**

// Q = 2e-3이면:

// - q = 4e-5
// - 5% 반영 -> R_odom ≈ 4.0e-4
// - 10% 반영 -> R_odom ≈ 2.5e-4
// - 1% 반영 -> R_odom ≈ 2.0e-3
// - 평소 구간: scan **95% R_scan = 2.5e-6**
// - 직선 구간: scan **80~85% R_scan = 1.1 e-5**
// - 곡선 구간: scan **97~99%  1.0e-6**


//////////////////////////////////////////////////////////////////


///////////////////////IMU condiser translation velocity //////////////////
Eigen::Vector3d imu_delta_velocity = Eigen::Vector3d::Zero(); // imu 기반으로 계산한 속도 변화량을 저장
Eigen::Vector3d prev_linear_acceleration = Eigen::Vector3d::Zero(); // 이전 가속도 값을 저장
bool imu_velocity_initalized = false;
common::Time last_imu_time = common::Time::min(); // imu가 이전에 측정한 시간을 뜻한다

// 튜닝 완료
// double imu_weight = 0.2;//예측 속도에 미칠 imu의 영향
// // double imu_weight = 0.2;//예측 속도에 미칠 imu의 
// // double imu_delta_clip = 0.2; // 너무 강한 보정이 들어갈 경우 clip 한다
// double imu_delta_min = 0.3; // 너무 작은 보정이 들어갈 경우 제거할 임계값
//double wheelodom_weight = 0.2;

// 튜닝 중
double imu_weight = 0.2;//예측 속도에 미칠 imu의 영향
// double imu_weight = 0.2;//예측 속도에 미칠 imu의 
// double imu_delta_clip = 0.2; // 너무 강한 보정이 들어갈 경우 clip 한다
double imu_delta_min = 0.3; // 너무 작은 보정이 들어갈 경우 제거할 임계값
double wheelodom_weight = 0.01;
Eigen::Vector3d translation_imu_wheel(const Eigen::Vector3d* linear_velocity_scan, const Eigen::Vector3d* linear_velocity_odom);
////////////////////////////////////////////////////////////////////////



  const common::Duration pose_queue_duration_;
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
