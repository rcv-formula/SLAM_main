/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kFrontWeakMinDistance = 4.0;
constexpr double kFrontWeakHalfWidth = 1.5;
constexpr int kFrontWeakMaxPointCount = 15;
constexpr double kFrontWeakMaxPointFraction = 0.03;
constexpr double kMotionLossMaxScanVelocityRatio = 0.35;
constexpr int kMotionLossPriorHoldScans = 3;
constexpr int kStraightMismatchMinConsecutiveScans = 2;
constexpr double kStraightCommandSteeringMax = 0.08;
constexpr double kStraightLongitudinalWeightMultiplier = 4.0;

absl::Mutex* CommandDebugMutex() {
  static auto* const mutex = new absl::Mutex;
  return mutex;
}

absl::optional<LocalSlamCommandDebugData>* LatestCommandDebugData() {
  static auto* const command =
      new absl::optional<LocalSlamCommandDebugData>;
  return command;
}

}  // namespace

void SetLocalSlamCommandDebugData(const common::Time time, const double speed,
                                  const double steering_angle) {
  absl::MutexLock lock(CommandDebugMutex());
  *LatestCommandDebugData() =
      LocalSlamCommandDebugData{time, speed, steering_angle};
}

absl::optional<LocalSlamCommandDebugData> GetLocalSlamCommandDebugData() {
  absl::MutexLock lock(CommandDebugMutex());
  return *LatestCommandDebugData();
}

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {
  quality_metrics_csv_enabled_ = InitializeQualityMetricsCsvWriter();
}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z());
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    LocalSlamQualityMetrics* quality_metrics) {
  if (active_submaps_.submaps().empty()) {
    quality_metrics->translation_residual = 0.;
    quality_metrics->rotation_residual = 0.;
    quality_metrics->num_filtered_points =
        static_cast<int>(filtered_gravity_aligned_point_cloud.size());
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  quality_metrics->num_filtered_points =
      static_cast<int>(filtered_gravity_aligned_point_cloud.size());
  int front_point_count = 0;
  for (const sensor::RangefinderPoint& point :
       filtered_gravity_aligned_point_cloud) {
    if (point.position.x() > kFrontWeakMinDistance &&
        std::abs(point.position.y()) < kFrontWeakHalfWidth) {
      ++front_point_count;
    }
  }
  quality_metrics->front_point_count = front_point_count;
  quality_metrics->front_point_fraction =
      filtered_gravity_aligned_point_cloud.empty()
          ? 0.
          : static_cast<double>(front_point_count) /
                static_cast<double>(filtered_gravity_aligned_point_cloud.size());
  quality_metrics->front_weak =
      front_point_count <= kFrontWeakMaxPointCount ||
      quality_metrics->front_point_fraction <= kFrontWeakMaxPointFraction;
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
    quality_metrics->real_time_correlative_score = score;
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
    latest_scan_match_score_ = score;
    latest_scan_match_score_valid_ = true;
    if (extrapolator_) {
      extrapolator_->ScanMatchScore(score);
    }

  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  double longitudinal_translation_weight = 0.;
  Eigen::Vector2d longitudinal_target_heading(
      std::cos(pose_prediction.rotation().angle()),
      std::sin(pose_prediction.rotation().angle()));
  Eigen::Vector2d longitudinal_target_translation =
      pose_prediction.translation();
  const double dt_since_last_pose =
      last_pose_estimate_time_.has_value()
          ? common::ToSeconds(time - last_pose_estimate_time_.value())
          : 0.;
  quality_metrics->motion_loss_prior_hold_count =
      longitudinal_motion_loss_prior_hold_count_;
  double wheel_velocity = std::numeric_limits<double>::quiet_NaN();
  if (extrapolator_ && extrapolator_->HasOdometryData()) {
    wheel_velocity = extrapolator_->GetOdometryForwardVelocity();
    quality_metrics->raw_wheel_forward_velocity = wheel_velocity;
    quality_metrics->wheel_forward_velocity = wheel_velocity;
    if (dt_since_last_pose > 0.) {
      quality_metrics->wheel_twist_expected_delta =
          wheel_velocity * dt_since_last_pose;
    }
  }
  const double base_longitudinal_translation_weight =
      options_.ceres_scan_matcher_options().longitudinal_translation_weight();
  quality_metrics->longitudinal_prior_wheel_delta_scale =
      options_.ceres_scan_matcher_options()
          .longitudinal_prior_wheel_delta_scale();
  if (last_pose_estimate_time_.has_value() &&
      last_pose_estimate_2d_.has_value() &&
      std::isfinite(quality_metrics->wheel_twist_expected_delta) &&
      std::isfinite(wheel_velocity)) {
    quality_metrics->odom_prior_translation =
        std::abs(quality_metrics->wheel_twist_expected_delta);
    quality_metrics->odom_prior_yaw_rate =
        std::isfinite(quality_metrics->imu_delta_yaw_rate)
            ? std::abs(quality_metrics->imu_delta_yaw_rate)
            : std::abs(latest_imu_angular_velocity_z_);
  }
  quality_metrics->longitudinal_prior_active =
      longitudinal_translation_weight > 0.;
  quality_metrics->longitudinal_prior_weight =
      longitudinal_translation_weight;
  const double rotation_weight =
      options_.ceres_scan_matcher_options().rotation_weight();
  quality_metrics->ceres_rotation_weight = rotation_weight;
  ceres_scan_matcher_.Match(
      pose_prediction.translation(), longitudinal_target_heading,
      longitudinal_target_translation,
      longitudinal_translation_weight, rotation_weight, initial_ceres_pose,
      filtered_gravity_aligned_point_cloud, *matching_submap->grid(),
      pose_observation.get(), &summary);
  if (pose_observation) {
    bool straight_longitudinal_mismatch = false;
    if (last_pose_estimate_time_.has_value() &&
        last_pose_estimate_2d_.has_value() && dt_since_last_pose > 0. &&
        std::isfinite(quality_metrics->wheel_twist_expected_delta) &&
        std::isfinite(wheel_velocity)) {
      const transform::Rigid2d& last_pose = last_pose_estimate_2d_.value();
      const double imu_delta_yaw =
          std::isfinite(quality_metrics->imu_delta_yaw)
              ? quality_metrics->imu_delta_yaw
              : 0.;
      const double imu_mid_heading_angle =
          last_pose.rotation().angle() + 0.5 * imu_delta_yaw;
      const Eigen::Vector2d imu_mid_heading(
          std::cos(imu_mid_heading_angle), std::sin(imu_mid_heading_angle));
      const double preliminary_scan_forward_delta =
          (pose_observation->translation() - last_pose.translation())
              .dot(imu_mid_heading);
      quality_metrics->trigger_scan_forward_delta =
          preliminary_scan_forward_delta;
      const double wheel_delta = quality_metrics->wheel_twist_expected_delta;
      const double wheel_speed = std::abs(wheel_velocity);
      const double yaw_rate =
          std::isfinite(quality_metrics->imu_delta_yaw_rate)
              ? std::abs(quality_metrics->imu_delta_yaw_rate)
              : std::abs(latest_imu_angular_velocity_z_);
      const bool steering_is_straight =
          !std::isfinite(quality_metrics->command_steering_angle) ||
          std::abs(quality_metrics->command_steering_angle) <=
              kStraightCommandSteeringMax;
      const bool robot_moves_forward =
          wheel_delta > 0. &&
          wheel_speed >= options_.ceres_scan_matcher_options()
                             .longitudinal_translation_min_speed();
      const bool scan_lost_forward_motion =
          preliminary_scan_forward_delta <= 0. ||
          preliminary_scan_forward_delta <
              wheel_delta * kMotionLossMaxScanVelocityRatio;
      const bool yaw_is_straight =
          yaw_rate <= options_.ceres_scan_matcher_options()
                          .longitudinal_translation_max_yaw_rate();
      const bool strict_straight_longitudinal_mismatch =
          robot_moves_forward && scan_lost_forward_motion && yaw_is_straight &&
          steering_is_straight;
      if (strict_straight_longitudinal_mismatch) {
        ++straight_longitudinal_mismatch_streak_;
      } else if (!robot_moves_forward || !scan_lost_forward_motion) {
        straight_longitudinal_mismatch_streak_ = 0;
        longitudinal_motion_loss_prior_hold_count_ = 0;
      } else {
        straight_longitudinal_mismatch_streak_ = 0;
      }
      const bool confirmed_straight_longitudinal_mismatch =
          strict_straight_longitudinal_mismatch &&
          straight_longitudinal_mismatch_streak_ >=
              kStraightMismatchMinConsecutiveScans;
      if (confirmed_straight_longitudinal_mismatch) {
        longitudinal_motion_loss_prior_hold_count_ =
            kMotionLossPriorHoldScans;
      }
      const bool hold_allows_prior =
          robot_moves_forward && scan_lost_forward_motion &&
          longitudinal_motion_loss_prior_hold_count_ > 0;
      straight_longitudinal_mismatch =
          strict_straight_longitudinal_mismatch || hold_allows_prior;
      quality_metrics->straight_longitudinal_mismatch =
          straight_longitudinal_mismatch;
      quality_metrics->straight_longitudinal_mismatch_streak =
          straight_longitudinal_mismatch_streak_;
      quality_metrics->motion_loss_prior_hold_count =
          longitudinal_motion_loss_prior_hold_count_;
      const bool apply_straight_longitudinal_prior =
          base_longitudinal_translation_weight > 0. &&
          (confirmed_straight_longitudinal_mismatch || hold_allows_prior);
      if (apply_straight_longitudinal_prior) {
        longitudinal_target_heading = imu_mid_heading;
        const double target_delta =
            options_.ceres_scan_matcher_options()
                .longitudinal_prior_wheel_delta_scale() *
            wheel_delta;
        longitudinal_target_translation =
            last_pose.translation() + target_delta * longitudinal_target_heading;
        longitudinal_translation_weight =
            base_longitudinal_translation_weight *
            kStraightLongitudinalWeightMultiplier;
        quality_metrics->longitudinal_prior_target_delta = target_delta;
        quality_metrics->longitudinal_prior_active = true;
        quality_metrics->longitudinal_prior_weight =
            longitudinal_translation_weight;
        initial_ceres_pose = *pose_observation;
        ceres_scan_matcher_.Match(
            pose_prediction.translation(), longitudinal_target_heading,
            longitudinal_target_translation, longitudinal_translation_weight,
            rotation_weight, initial_ceres_pose,
            filtered_gravity_aligned_point_cloud, *matching_submap->grid(),
            pose_observation.get(), &summary);
      }
      if (!confirmed_straight_longitudinal_mismatch &&
          hold_allows_prior && longitudinal_motion_loss_prior_hold_count_ > 0) {
        --longitudinal_motion_loss_prior_hold_count_;
      }
      quality_metrics->motion_loss_prior_hold_count =
          longitudinal_motion_loss_prior_hold_count_;
    }
    quality_metrics->ceres_final_cost = summary.final_cost;
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation()).norm();
    quality_metrics->translation_residual = residual_distance;
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::atan2(std::sin(pose_observation->rotation().angle() -
                            pose_prediction.rotation().angle()),
                   std::cos(pose_observation->rotation().angle() -
                            pose_prediction.rotation().angle()));
    const Eigen::Vector2d heading(
        std::cos(pose_prediction.rotation().angle()),
        std::sin(pose_prediction.rotation().angle()));
    const Eigen::Vector2d lateral(-heading.y(), heading.x());
    const Eigen::Vector2d residual =
        pose_observation->translation() - pose_prediction.translation();
    quality_metrics->longitudinal_residual = residual.dot(heading);
    quality_metrics->lateral_residual = residual.dot(lateral);
    quality_metrics->yaw_residual = residual_angle;
    quality_metrics->rotation_residual = std::abs(residual_angle);
    kScanMatcherResidualAngleMetric->Observe(
        quality_metrics->rotation_residual);
  }
  return pose_observation;
}

bool LocalTrajectoryBuilder2D::IsLocalSlamOutlier(
    LocalSlamQualityMetrics* const quality_metrics) {
  if (!options_.skip_submap_insertion_for_outliers()) {
    quality_metrics->medium_outlier_streak = 0;
    return false;
  }

  int failure_count = 0;
  if (options_.outlier_min_correlative_score() > 0. &&
      !std::isnan(quality_metrics->real_time_correlative_score) &&
      quality_metrics->real_time_correlative_score <
          options_.outlier_min_correlative_score()) {
    ++failure_count;
  }
  if (!std::isnan(quality_metrics->translation_residual) &&
      quality_metrics->translation_residual >
          options_.outlier_max_translation_residual()) {
    ++failure_count;
  }
  if (!std::isnan(quality_metrics->rotation_residual) &&
      quality_metrics->rotation_residual >
          options_.outlier_max_rotation_residual()) {
    ++failure_count;
  }
  if (options_.outlier_min_num_filtered_points() > 0 &&
      quality_metrics->num_filtered_points <
          options_.outlier_min_num_filtered_points()) {
    ++failure_count;
  }

  const bool medium_translation_outlier =
      !std::isnan(quality_metrics->translation_residual) &&
      quality_metrics->translation_residual >
          options_.outlier_medium_translation_residual();
  const bool medium_rotation_outlier =
      !std::isnan(quality_metrics->rotation_residual) &&
      quality_metrics->rotation_residual >
          options_.outlier_medium_rotation_residual();
  const bool medium_outlier = medium_translation_outlier || medium_rotation_outlier;

  if (medium_outlier) {
    ++consecutive_medium_outlier_count_;
  } else {
    consecutive_medium_outlier_count_ = 0;
  }
  quality_metrics->medium_outlier_streak = consecutive_medium_outlier_count_;

  const bool hard_outlier =
      failure_count >= options_.outlier_required_failures();
  const bool sustained_medium_outlier =
      options_.outlier_medium_required_consecutive() > 0 &&
      consecutive_medium_outlier_count_ >=
          options_.outlier_medium_required_consecutive();
  return hard_outlier || sustained_medium_outlier;
}

absl::optional<transform::Rigid2d> LocalTrajectoryBuilder2D::InterpolateOdometry2D(
    const common::Time time) const {
  if (odometry_history_.size() < 2 || time < odometry_history_.front().time ||
      time > odometry_history_.back().time) {
    return absl::nullopt;
  }
  auto it = std::lower_bound(
      odometry_history_.begin(), odometry_history_.end(), time,
      [](const sensor::OdometryData& odometry_data,
         const common::Time time) { return odometry_data.time < time; });
  if (it == odometry_history_.begin()) {
    return transform::Project2D(it->pose);
  }
  if (it == odometry_history_.end()) {
    return transform::Project2D(odometry_history_.back().pose);
  }
  const sensor::OdometryData& after = *it;
  const sensor::OdometryData& before = *(it - 1);
  const double duration = common::ToSeconds(after.time - before.time);
  if (duration <= 0.) {
    return transform::Project2D(before.pose);
  }
  const double factor = common::ToSeconds(time - before.time) / duration;
  const transform::Rigid2d before_2d = transform::Project2D(before.pose);
  const transform::Rigid2d after_2d = transform::Project2D(after.pose);
  const Eigen::Vector2d translation =
      before_2d.translation() +
      factor * (after_2d.translation() - before_2d.translation());
  const double angle_delta = std::atan2(
      std::sin(after_2d.rotation().angle() - before_2d.rotation().angle()),
      std::cos(after_2d.rotation().angle() - before_2d.rotation().angle()));
  return transform::Rigid2d(
      translation, before_2d.rotation().angle() + factor * angle_delta);
}

absl::optional<transform::Rigid2d>
LocalTrajectoryBuilder2D::InterpolateOrLatestOdometry2D(
    const common::Time time) const {
  if (odometry_history_.size() < 2 || time < odometry_history_.front().time) {
    return absl::nullopt;
  }
  if (time > odometry_history_.back().time) {
    return transform::Project2D(odometry_history_.back().pose);
  }
  return InterpolateOdometry2D(time);
}

std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;
  for (const auto& range : synchronized_data.ranges) {
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;
    num_accumulated_ = 0;
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    accumulated_range_data_.origin = range_data_poses.back().translation();
    return AddAccumulatedRangeData(
        time,
        TransformToGravityAlignedFrameAndFilter(
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment, sensor_duration);
  }
  return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) {
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  LocalSlamQualityMetrics quality_metrics;
  quality_metrics.odom_history_size =
      static_cast<int>(odometry_history_.size());
  if (!odometry_history_.empty()) {
    quality_metrics.odom_latest_age =
        common::ToSeconds(time - odometry_history_.back().time);
  }
  if (last_pose_estimate_time_.has_value() &&
      last_pose_estimate_2d_.has_value()) {
    const double dt =
        common::ToSeconds(time - last_pose_estimate_time_.value());
    quality_metrics.prediction_delta_dt = dt;
    if (dt > 0.) {
      const transform::Rigid2d& last_pose =
          last_pose_estimate_2d_.value();
      const Eigen::Vector2d last_heading(
          std::cos(last_pose.rotation().angle()),
          std::sin(last_pose.rotation().angle()));
      const Eigen::Vector2d prediction_delta =
          pose_prediction.translation() - last_pose.translation();
      const Eigen::Vector2d last_lateral(-last_heading.y(), last_heading.x());
      const double prediction_yaw_delta = std::atan2(
          std::sin(pose_prediction.rotation().angle() -
                   last_pose.rotation().angle()),
          std::cos(pose_prediction.rotation().angle() -
                   last_pose.rotation().angle()));
      quality_metrics.prediction_delta_forward =
          prediction_delta.dot(last_heading);
      quality_metrics.prediction_delta_lateral =
          prediction_delta.dot(last_lateral);
      quality_metrics.prediction_delta_x = prediction_delta.x();
      quality_metrics.prediction_delta_y = prediction_delta.y();
      quality_metrics.prediction_delta_forward_velocity =
          quality_metrics.prediction_delta_forward / dt;
      quality_metrics.prediction_delta_translation =
          prediction_delta.norm();
      quality_metrics.prediction_delta_yaw = prediction_yaw_delta;
      quality_metrics.imu_delta_dt = dt;
      quality_metrics.imu_delta_yaw =
          integrated_imu_yaw_ - last_pose_integrated_imu_yaw_;
      quality_metrics.imu_delta_yaw_rate =
          quality_metrics.imu_delta_yaw / dt;
      const absl::optional<LocalSlamCommandDebugData> command =
          GetLocalSlamCommandDebugData();
      if (command.has_value()) {
        quality_metrics.command_latest_age =
            common::ToSeconds(time - command.value().time);
        quality_metrics.command_speed = command.value().speed;
        quality_metrics.command_steering_angle =
            command.value().steering_angle;
        quality_metrics.command_expected_delta =
            command.value().speed * dt;
      }

      const absl::optional<transform::Rigid2d> odom_prev =
          InterpolateOrLatestOdometry2D(last_pose_estimate_time_.value());
      const absl::optional<transform::Rigid2d> odom_now =
          InterpolateOrLatestOdometry2D(time);
      if (odom_prev.has_value() && odom_now.has_value()) {
        const transform::Rigid2d odom_delta =
            odom_prev.value().inverse() * odom_now.value();
        const Eigen::Vector2d odom_delta_in_local =
            last_pose.rotation() * odom_delta.translation();
        const Eigen::Vector2d last_lateral(-last_heading.y(), last_heading.x());
        const double odom_yaw_delta = std::atan2(
            std::sin(odom_delta.rotation().angle()),
            std::cos(odom_delta.rotation().angle()));
        quality_metrics.odom_pose_delta_dt = dt;
        quality_metrics.odom_pose_delta_forward =
            odom_delta_in_local.dot(last_heading);
        quality_metrics.odom_pose_delta_lateral =
            odom_delta_in_local.dot(last_lateral);
        quality_metrics.odom_pose_delta_local_x =
            odom_delta.translation().x();
        quality_metrics.odom_pose_delta_local_y =
            odom_delta.translation().y();
        quality_metrics.odom_pose_delta_forward_velocity =
            quality_metrics.odom_pose_delta_forward / dt;
        quality_metrics.odom_pose_delta_translation =
            odom_delta_in_local.norm();
        quality_metrics.odom_pose_delta_yaw = odom_yaw_delta;
      }
    }
  }
  // local map frame <- gravity-aligned frame
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud,
                &quality_metrics);
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  if (last_pose_estimate_time_.has_value() &&
      last_pose_estimate_2d_.has_value()) {
    const double dt =
        common::ToSeconds(time - last_pose_estimate_time_.value());
    if (dt > 0.) {
      const transform::Rigid2d& last_pose =
          last_pose_estimate_2d_.value();
      const double scan_match_yaw_delta = std::atan2(
          std::sin(pose_estimate_2d->rotation().angle() -
                   last_pose.rotation().angle()),
          std::cos(pose_estimate_2d->rotation().angle() -
                   last_pose.rotation().angle()));
      quality_metrics.scan_match_yaw_rate =
          std::abs(scan_match_yaw_delta / dt);
      const Eigen::Vector2d heading(std::cos(last_pose.rotation().angle()),
                                    std::sin(last_pose.rotation().angle()));
      const Eigen::Vector2d lateral(-heading.y(), heading.x());
      const Eigen::Vector2d scan_match_delta =
          pose_estimate_2d->translation() - last_pose.translation();
      quality_metrics.scan_match_delta_forward =
          scan_match_delta.dot(heading);
      quality_metrics.scan_match_delta_lateral =
          scan_match_delta.dot(lateral);
      quality_metrics.scan_match_delta_translation =
          scan_match_delta.norm();
      quality_metrics.scan_match_delta_yaw = scan_match_yaw_delta;
      quality_metrics.scan_forward_velocity =
          quality_metrics.scan_match_delta_forward / dt;
    }
  }
  const bool longitudinal_motion_mismatch =
      quality_metrics.straight_longitudinal_mismatch;
  quality_metrics.longitudinal_motion_loss = longitudinal_motion_mismatch;
  quality_metrics.motion_loss_prior_hold_count =
      longitudinal_motion_loss_prior_hold_count_;
  const transform::Rigid2d accepted_pose_2d = *pose_estimate_2d;
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(accepted_pose_2d) * gravity_alignment;
  extrapolator_->AddPose(time, pose_estimate);
  last_pose_estimate_time_ = time;
  last_pose_estimate_2d_ = accepted_pose_2d;
  last_pose_integrated_imu_yaw_ = integrated_imu_yaw_;
  quality_metrics.was_outlier = IsLocalSlamOutlier(&quality_metrics);

  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(accepted_pose_2d.cast<float>()));
  if (quality_metrics.was_outlier) {
    LOG_EVERY_N(WARNING, 20)
        << "Local SLAM outlier detected (inserting with reduced constraint weight). "
        << "score=" << quality_metrics.real_time_correlative_score
        << " translation_residual=" << quality_metrics.translation_residual
        << " rotation_residual=" << quality_metrics.rotation_residual
        << " num_filtered_points=" << quality_metrics.num_filtered_points
        << " medium_outlier_streak=" << quality_metrics.medium_outlier_streak;
  }
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation(), quality_metrics.was_outlier);
  MaybeWriteQualityMetricsCsv(
      time, pose_prediction, accepted_pose_2d, quality_metrics,
      insertion_result != nullptr,
      insertion_result != nullptr ? insertion_result->insertion_submaps.size()
                                  : 0);

  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     quality_metrics,
                     latest_scan_match_score_,
                     latest_scan_match_score_valid_,
                     std::move(insertion_result)});
}

std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment,
    const bool is_outlier) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate,
          is_outlier}),
      std::move(insertion_submaps)});
}

bool LocalTrajectoryBuilder2D::InitializeQualityMetricsCsvWriter() {
  if (!options_.log_local_quality_metrics_to_csv()) {
    return false;
  }
  if (options_.local_quality_metrics_csv_path().empty()) {
    LOG(WARNING) << "Local quality metrics CSV logging is enabled, but "
                    "'local_quality_metrics_csv_path' is empty.";
    return false;
  }

  const std::string& path = options_.local_quality_metrics_csv_path();
  bool write_header = true;
  {
    std::ifstream existing_file(path);
    write_header = !existing_file.good() ||
                   existing_file.peek() == std::ifstream::traits_type::eof();
  }

  quality_metrics_csv_.open(path, std::ios::out | std::ios::app);
  if (!quality_metrics_csv_.is_open()) {
    LOG(ERROR) << "Failed to open local quality metrics CSV file: " << path;
    return false;
  }
  quality_metrics_csv_ << std::setprecision(17);
  if (write_header) {
    quality_metrics_csv_
        << "stamp,rt_correlative_score,ceres_final_cost,"
           "translation_residual,rotation_residual,num_filtered_points,"
           "was_outlier,medium_outlier_streak,inserted_to_submap,"
           "num_insertion_submaps,pose_prediction_x,pose_prediction_y,"
           "pose_prediction_yaw,pose_estimate_x,pose_estimate_y,"
           "pose_estimate_yaw,longitudinal_residual,lateral_residual,"
           "yaw_residual,wheel_forward_velocity,raw_wheel_forward_velocity,"
           "scan_forward_velocity,scan_match_yaw_rate,"
           "scan_match_delta_forward,scan_match_delta_lateral,"
           "scan_match_delta_translation,scan_match_delta_yaw,"
           "prediction_delta_dt,prediction_delta_forward,"
           "prediction_delta_lateral,prediction_delta_x,prediction_delta_y,"
           "prediction_delta_forward_velocity,prediction_delta_translation,"
           "prediction_delta_yaw,wheel_twist_expected_delta,"
           "odom_pose_delta_dt,odom_pose_delta_forward,"
           "odom_pose_delta_lateral,odom_pose_delta_local_x,"
           "odom_pose_delta_local_y,"
           "odom_pose_delta_forward_velocity,odom_pose_delta_translation,"
           "odom_pose_delta_yaw,odom_latest_age,odom_history_size,"
           "front_point_count,front_point_fraction,front_weak,"
           "longitudinal_prior_active,"
           "longitudinal_motion_loss,straight_longitudinal_mismatch,"
           "straight_longitudinal_mismatch_streak,"
           "motion_loss_prior_hold_count,"
           "longitudinal_prior_weight,ceres_rotation_weight,"
           "longitudinal_prior_target_delta,"
           "longitudinal_prior_wheel_delta_scale,"
           "trigger_scan_forward_delta,odom_prior_translation,"
           "odom_prior_yaw_rate,imu_yaw_rate,imu_delta_dt,"
           "imu_delta_yaw,imu_delta_yaw_rate,command_latest_age,"
           "command_speed,command_steering_angle,command_expected_delta\n";
    quality_metrics_csv_.flush();
  }
  return true;
}

void LocalTrajectoryBuilder2D::MaybeWriteQualityMetricsCsv(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const transform::Rigid2d& pose_estimate,
    const LocalSlamQualityMetrics& quality_metrics,
    const bool inserted_to_submap, const int num_insertion_submaps) {
  if (!quality_metrics_csv_enabled_) {
    return;
  }

  quality_metrics_csv_
      << common::ToUniversal(time) << ','
      << quality_metrics.real_time_correlative_score << ','
      << quality_metrics.ceres_final_cost << ','
      << quality_metrics.translation_residual << ','
      << quality_metrics.rotation_residual << ','
      << quality_metrics.num_filtered_points << ','
      << static_cast<int>(quality_metrics.was_outlier) << ','
      << quality_metrics.medium_outlier_streak << ','
      << static_cast<int>(inserted_to_submap) << ','
      << num_insertion_submaps << ',' << pose_prediction.translation().x()
      << ',' << pose_prediction.translation().y() << ','
      << pose_prediction.rotation().angle() << ','
      << pose_estimate.translation().x() << ','
      << pose_estimate.translation().y() << ','
      << pose_estimate.rotation().angle() << ','
      << quality_metrics.longitudinal_residual << ','
      << quality_metrics.lateral_residual << ','
      << quality_metrics.yaw_residual << ','
      << quality_metrics.wheel_forward_velocity << ','
      << quality_metrics.raw_wheel_forward_velocity << ','
      << quality_metrics.scan_forward_velocity << ','
      << quality_metrics.scan_match_yaw_rate << ','
      << quality_metrics.scan_match_delta_forward << ','
      << quality_metrics.scan_match_delta_lateral << ','
      << quality_metrics.scan_match_delta_translation << ','
      << quality_metrics.scan_match_delta_yaw << ','
      << quality_metrics.prediction_delta_dt << ','
      << quality_metrics.prediction_delta_forward << ','
      << quality_metrics.prediction_delta_lateral << ','
      << quality_metrics.prediction_delta_x << ','
      << quality_metrics.prediction_delta_y << ','
      << quality_metrics.prediction_delta_forward_velocity << ','
      << quality_metrics.prediction_delta_translation << ','
      << quality_metrics.prediction_delta_yaw << ','
      << quality_metrics.wheel_twist_expected_delta << ','
      << quality_metrics.odom_pose_delta_dt << ','
      << quality_metrics.odom_pose_delta_forward << ','
      << quality_metrics.odom_pose_delta_lateral << ','
      << quality_metrics.odom_pose_delta_local_x << ','
      << quality_metrics.odom_pose_delta_local_y << ','
      << quality_metrics.odom_pose_delta_forward_velocity << ','
      << quality_metrics.odom_pose_delta_translation << ','
      << quality_metrics.odom_pose_delta_yaw << ','
      << quality_metrics.odom_latest_age << ','
      << quality_metrics.odom_history_size << ','
      << quality_metrics.front_point_count << ','
      << quality_metrics.front_point_fraction << ','
      << static_cast<int>(quality_metrics.front_weak) << ','
      << static_cast<int>(quality_metrics.longitudinal_prior_active) << ','
      << static_cast<int>(quality_metrics.longitudinal_motion_loss) << ','
      << static_cast<int>(quality_metrics.straight_longitudinal_mismatch)
      << ','
      << quality_metrics.straight_longitudinal_mismatch_streak << ','
      << quality_metrics.motion_loss_prior_hold_count << ','
      << quality_metrics.longitudinal_prior_weight << ','
      << quality_metrics.ceres_rotation_weight << ','
      << quality_metrics.longitudinal_prior_target_delta << ','
      << quality_metrics.longitudinal_prior_wheel_delta_scale << ','
      << quality_metrics.trigger_scan_forward_delta << ','
      << quality_metrics.odom_prior_translation << ','
      << quality_metrics.odom_prior_yaw_rate << ','
      << latest_imu_angular_velocity_z_ << ','
      << quality_metrics.imu_delta_dt << ','
      << quality_metrics.imu_delta_yaw << ','
      << quality_metrics.imu_delta_yaw_rate << ','
      << quality_metrics.command_latest_age << ','
      << quality_metrics.command_speed << ','
      << quality_metrics.command_steering_angle << ','
      << quality_metrics.command_expected_delta << '\n';
  quality_metrics_csv_.flush();
}

void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  if (last_imu_time_.has_value()) {
    const double dt = common::ToSeconds(imu_data.time - last_imu_time_.value());
    if (dt > 0. && dt < 1.) {
      integrated_imu_yaw_ += imu_data.angular_velocity.z() * dt;
    }
  }
  last_imu_time_ = imu_data.time;
  latest_imu_angular_velocity_z_ = imu_data.angular_velocity.z();
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  odometry_history_.push_back(odometry_data);
  const common::Time history_cutoff =
      odometry_data.time - common::FromSeconds(10.);
  while (odometry_history_.size() > 2 &&
         odometry_history_.front().time < history_cutoff) {
    odometry_history_.pop_front();
  }
  while (odometry_history_.size() > 500) {
    odometry_history_.pop_front();
  }
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()),
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant());
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
