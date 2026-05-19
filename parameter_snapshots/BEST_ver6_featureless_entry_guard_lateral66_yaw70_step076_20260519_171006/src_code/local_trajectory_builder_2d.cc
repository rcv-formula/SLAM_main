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
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

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
    LOG(WARNING) << "Could not open adaptive mapping config: " << config_path;
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
        continue;
      }
      values[key] = parsed_value;
    } catch (const std::exception& exception) {
      LOG(WARNING) << "Ignoring invalid adaptive mapping config value at "
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

double ComputePlanarDegeneracyRatio(const sensor::PointCloud& point_cloud) {
  if (point_cloud.size() < 3) {
    return 0.;
  }
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& point : point_cloud) {
    mean += point.position.head<2>().cast<double>();
  }
  mean /= static_cast<double>(point_cloud.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& point : point_cloud) {
    const Eigen::Vector2d centered =
        point.position.head<2>().cast<double>() - mean;
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(point_cloud.size());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return 0.;
  }
  const double small = std::max(1e-6, solver.eigenvalues()(0));
  const double large = std::max(0., solver.eigenvalues()(1));
  return large / small;
}

}  // namespace

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
  LoadAdaptiveStraightConfigFromYaml();
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
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
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
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  if (pose_observation) {
    quality_metrics->ceres_final_cost = summary.final_cost;
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    quality_metrics->translation_residual = residual_distance;
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    quality_metrics->rotation_residual = residual_angle;
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
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

void LocalTrajectoryBuilder2D::LoadAdaptiveStraightConfigFromYaml() {
  const char* config_path_env = std::getenv("MAPPING_ADAPTIVE_CONFIG");
  if (config_path_env == nullptr || std::string(config_path_env).empty()) {
    config_path_env = std::getenv("POSE_EXTRAPOLATOR_CONFIG");
  }
  if (config_path_env == nullptr || std::string(config_path_env).empty()) {
    config_path_env = std::getenv("WHEEL_ODOM_CONFIG");
  }
  if (config_path_env == nullptr || std::string(config_path_env).empty()) {
    LOG(INFO) << "MAPPING_ADAPTIVE_CONFIG/POSE_EXTRAPOLATOR_CONFIG is not set. "
              << "Using built-in adaptive straight mapping defaults.";
    return;
  }

  const auto config = LoadFlatDoubleConfig(config_path_env);
  if (config.empty()) {
    return;
  }
  adaptive_straight_enabled_ =
      ConfigValue(config, "adaptive_straight_enabled",
                  adaptive_straight_enabled_ ? 1. : 0.) != 0.;
  adaptive_straight_eigen_ratio_threshold_ =
      ConfigValue(config, "adaptive_straight_eigen_ratio_threshold",
                  adaptive_straight_eigen_ratio_threshold_);
  adaptive_straight_min_points_ = static_cast<int>(
      ConfigValue(config, "adaptive_straight_min_points",
                  adaptive_straight_min_points_));
  adaptive_straight_min_score_ =
      ConfigValue(config, "adaptive_straight_min_score",
                  adaptive_straight_min_score_);
  adaptive_straight_max_score_ =
      ConfigValue(config, "adaptive_straight_max_score",
                  adaptive_straight_max_score_);
  adaptive_straight_enter_streak_ = static_cast<int>(
      ConfigValue(config, "adaptive_straight_enter_streak",
                  adaptive_straight_enter_streak_));
  adaptive_straight_exit_streak_ = static_cast<int>(
      ConfigValue(config, "adaptive_straight_exit_streak",
                  adaptive_straight_exit_streak_));
  featureless_scan_longitudinal_blend_ =
      ConfigValue(config, "featureless_scan_longitudinal_blend",
                  featureless_scan_longitudinal_blend_);
  featureless_scan_lateral_blend_ =
      ConfigValue(config, "featureless_scan_lateral_blend",
                  featureless_scan_lateral_blend_);
  featureless_scan_yaw_blend_ =
      ConfigValue(config, "featureless_scan_yaw_blend",
                  featureless_scan_yaw_blend_);
  featureless_curve_translation_residual_threshold_ =
      ConfigValue(config, "featureless_curve_translation_residual_threshold",
                  featureless_curve_translation_residual_threshold_);
  featureless_curve_rotation_residual_threshold_ =
      ConfigValue(config, "featureless_curve_rotation_residual_threshold",
                  featureless_curve_rotation_residual_threshold_);
  featureless_curve_scan_longitudinal_blend_ =
      ConfigValue(config, "featureless_curve_scan_longitudinal_blend",
                  featureless_curve_scan_longitudinal_blend_);
  featureless_curve_scan_lateral_blend_ =
      ConfigValue(config, "featureless_curve_scan_lateral_blend",
                  featureless_curve_scan_lateral_blend_);
  featureless_curve_scan_yaw_blend_ =
      ConfigValue(config, "featureless_curve_scan_yaw_blend",
                  featureless_curve_scan_yaw_blend_);
  featureless_max_step_m_ =
      ConfigValue(config, "featureless_max_step_m", featureless_max_step_m_);
  LOG(INFO) << "Loaded adaptive straight mapping config from " << config_path_env
            << " enabled=" << adaptive_straight_enabled_
            << " eigen_ratio_threshold="
            << adaptive_straight_eigen_ratio_threshold_
            << " min_points=" << adaptive_straight_min_points_
            << " score_range=[" << adaptive_straight_min_score_ << ", "
            << adaptive_straight_max_score_ << "]"
            << " enter_streak=" << adaptive_straight_enter_streak_
            << " exit_streak=" << adaptive_straight_exit_streak_
            << " blends(longitudinal,lateral,yaw)=("
            << featureless_scan_longitudinal_blend_ << ", "
            << featureless_scan_lateral_blend_ << ", "
            << featureless_scan_yaw_blend_ << ")"
            << " curve_residual_thresholds(translation,rotation)=("
            << featureless_curve_translation_residual_threshold_ << ", "
            << featureless_curve_rotation_residual_threshold_ << ")"
            << " curve_blends(longitudinal,lateral,yaw)=("
            << featureless_curve_scan_longitudinal_blend_ << ", "
            << featureless_curve_scan_lateral_blend_ << ", "
            << featureless_curve_scan_yaw_blend_ << ")"
            << " max_step_m=" << featureless_max_step_m_;
}

void LocalTrajectoryBuilder2D::UpdateFeaturelessStraightMode(
    const double scan_match_score,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    LocalSlamQualityMetrics* const quality_metrics) {
  quality_metrics->num_filtered_points =
      static_cast<int>(filtered_gravity_aligned_point_cloud.size());
  quality_metrics->geometry_degeneracy_ratio =
      ComputePlanarDegeneracyRatio(filtered_gravity_aligned_point_cloud);

  const bool score_in_adaptive_range =
      std::isnan(scan_match_score) ||
      (scan_match_score >= adaptive_straight_min_score_ &&
       scan_match_score <= adaptive_straight_max_score_);
  const bool featureless_geometry =
      adaptive_straight_enabled_ &&
      quality_metrics->num_filtered_points >= adaptive_straight_min_points_ &&
      quality_metrics->geometry_degeneracy_ratio >=
          adaptive_straight_eigen_ratio_threshold_ &&
      score_in_adaptive_range;

  if (featureless_geometry) {
    ++featureless_straight_streak_;
    featureful_straight_streak_ = 0;
  } else {
    ++featureful_straight_streak_;
    featureless_straight_streak_ = 0;
  }

  if (!featureless_straight_mode_ &&
      featureless_straight_streak_ >= adaptive_straight_enter_streak_) {
    featureless_straight_mode_ = true;
  }
  if (featureless_straight_mode_ &&
      featureful_straight_streak_ >= adaptive_straight_exit_streak_) {
    featureless_straight_mode_ = false;
    featureless_step_limiter_initialized_ = false;
  }
  if (!featureless_straight_mode_) {
    featureless_step_limiter_initialized_ = false;
  }

  quality_metrics->featureless_straight_mode = featureless_straight_mode_;
  quality_metrics->featureless_straight_streak = featureless_straight_streak_;
  if (extrapolator_ != nullptr) {
    extrapolator_->SetFeaturelessStraightMode(
        featureless_straight_mode_,
        quality_metrics->geometry_degeneracy_ratio);
  }
}

transform::Rigid2d LocalTrajectoryBuilder2D::SelectPoseForAdaptiveStraight(
    const transform::Rigid2d& pose_prediction,
    const transform::Rigid2d& pose_estimate,
    const LocalSlamQualityMetrics& quality_metrics) {
  if (!quality_metrics.featureless_straight_mode) {
    if (featureless_max_step_m_ > 0.) {
      // Keep the last normal pose as the guard point for the first
      // featureless frame. Otherwise the entry frame can jump before the
      // limiter has a previous pose to compare against.
      featureless_last_pose_to_use_ = pose_estimate;
      featureless_step_limiter_initialized_ = true;
    } else {
      featureless_step_limiter_initialized_ = false;
    }
    return pose_estimate;
  }

  const double prediction_yaw = pose_prediction.rotation().angle();
  const bool curve_like_correction =
      quality_metrics.translation_residual >=
          featureless_curve_translation_residual_threshold_ ||
      std::abs(quality_metrics.rotation_residual) >=
          featureless_curve_rotation_residual_threshold_;
  const double longitudinal_blend =
      curve_like_correction ? featureless_curve_scan_longitudinal_blend_
                            : featureless_scan_longitudinal_blend_;
  const double lateral_blend =
      curve_like_correction ? featureless_curve_scan_lateral_blend_
                            : featureless_scan_lateral_blend_;
  const double yaw_blend =
      curve_like_correction ? featureless_curve_scan_yaw_blend_
                            : featureless_scan_yaw_blend_;
  const Eigen::Vector2d forward(std::cos(prediction_yaw),
                                std::sin(prediction_yaw));
  const Eigen::Vector2d lateral(-std::sin(prediction_yaw),
                                std::cos(prediction_yaw));
  const Eigen::Vector2d delta =
      pose_estimate.translation() - pose_prediction.translation();
  const Eigen::Vector2d blended_translation =
      pose_prediction.translation() +
      longitudinal_blend * delta.dot(forward) * forward +
      lateral_blend * delta.dot(lateral) * lateral;
  const double yaw_delta = common::NormalizeAngleDifference(
      pose_estimate.rotation().angle() - prediction_yaw);
  transform::Rigid2d result(
      blended_translation,
      prediction_yaw + yaw_blend * yaw_delta);

  if (featureless_max_step_m_ > 0.) {
    if (featureless_step_limiter_initialized_) {
      const Eigen::Vector2d delta =
          result.translation() - featureless_last_pose_to_use_.translation();
      const double step = delta.norm();
      if (step > featureless_max_step_m_) {
        const Eigen::Vector2d limited_translation =
            featureless_last_pose_to_use_.translation() +
            delta * (featureless_max_step_m_ / step);
        result = transform::Rigid2d(limited_translation, result.rotation());
      }
    }
    featureless_last_pose_to_use_ = result;
    featureless_step_limiter_initialized_ = true;
  }
  return result;
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
  // local map frame <- gravity-aligned frame
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud,
                &quality_metrics);
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  UpdateFeaturelessStraightMode(
      quality_metrics.real_time_correlative_score,
      filtered_gravity_aligned_point_cloud, &quality_metrics);
  quality_metrics.was_outlier = IsLocalSlamOutlier(&quality_metrics);
  const transform::Rigid2d pose_to_use_2d =
      quality_metrics.was_outlier
          ? pose_prediction
          : SelectPoseForAdaptiveStraight(pose_prediction, *pose_estimate_2d,
                                          quality_metrics);
  const transform::Rigid3d pose_to_use =
      transform::Embed3D(pose_to_use_2d) * gravity_alignment;
  extrapolator_->AddPose(time, pose_to_use);

  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_to_use_2d.cast<float>()));
  if (quality_metrics.was_outlier) {
    LOG_EVERY_N(WARNING, 20)
        << "Local SLAM outlier detected (using prediction and skipping submap update). "
        << "score=" << quality_metrics.real_time_correlative_score
        << " translation_residual=" << quality_metrics.translation_residual
        << " rotation_residual=" << quality_metrics.rotation_residual
        << " num_filtered_points=" << quality_metrics.num_filtered_points
        << " medium_outlier_streak=" << quality_metrics.medium_outlier_streak;
  }
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_to_use, gravity_alignment.rotation(), quality_metrics.was_outlier);
  MaybeWriteQualityMetricsCsv(
      time, pose_prediction, pose_to_use_2d, quality_metrics,
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
      MatchingResult{time, pose_to_use, std::move(range_data_in_local),
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
  if (is_outlier && options_.skip_submap_insertion_for_outliers()) {
    return nullptr;
  }
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
           "geometry_degeneracy_ratio,featureless_straight_mode,"
           "featureless_straight_streak,was_outlier,medium_outlier_streak,"
           "inserted_to_submap,num_insertion_submaps,pose_prediction_x,"
           "pose_prediction_y,pose_prediction_yaw,pose_estimate_x,"
           "pose_estimate_y,pose_estimate_yaw\n";
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
      << quality_metrics.geometry_degeneracy_ratio << ','
      << static_cast<int>(quality_metrics.featureless_straight_mode) << ','
      << quality_metrics.featureless_straight_streak << ','
      << static_cast<int>(quality_metrics.was_outlier) << ','
      << quality_metrics.medium_outlier_streak << ','
      << static_cast<int>(inserted_to_submap) << ','
      << num_insertion_submaps << ',' << pose_prediction.translation().x()
      << ',' << pose_prediction.translation().y() << ','
      << pose_prediction.rotation().angle() << ','
      << pose_estimate.translation().x() << ','
      << pose_estimate.translation().y() << ','
      << pose_estimate.rotation().angle() << '\n';
  quality_metrics_csv_.flush();
}

void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
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
