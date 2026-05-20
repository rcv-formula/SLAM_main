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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <array>
#include <cstdint>
#include <chrono>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalTrajectoryBuilder2D {
 public:
  struct LocalSlamQualityMetrics {
    double real_time_correlative_score =
        std::numeric_limits<double>::quiet_NaN();
    double ceres_final_cost = std::numeric_limits<double>::quiet_NaN();
    double translation_residual =
        std::numeric_limits<double>::quiet_NaN();
    double rotation_residual = std::numeric_limits<double>::quiet_NaN();
    int num_filtered_points = 0;
    bool was_outlier = false;
    bool hard_outlier = false;
    bool sustained_medium_outlier = false;
    int medium_outlier_streak = 0;
    double longitudinal_residual =
        std::numeric_limits<double>::quiet_NaN();
    double lateral_residual = std::numeric_limits<double>::quiet_NaN();
    double yaw_residual = std::numeric_limits<double>::quiet_NaN();
    double wheel_forward_velocity =
        std::numeric_limits<double>::quiet_NaN();
    double scan_forward_velocity =
        std::numeric_limits<double>::quiet_NaN();
    double scan_match_yaw_rate =
        std::numeric_limits<double>::quiet_NaN();
    double scan_match_delta_forward =
        std::numeric_limits<double>::quiet_NaN();
    double scan_match_delta_lateral =
        std::numeric_limits<double>::quiet_NaN();
    double scan_match_delta_translation =
        std::numeric_limits<double>::quiet_NaN();
    double scan_match_delta_yaw =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_dt =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_forward =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_lateral =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_x =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_y =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_forward_velocity =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_translation =
        std::numeric_limits<double>::quiet_NaN();
    double prediction_delta_yaw =
        std::numeric_limits<double>::quiet_NaN();
    double adaptive_odometry_weight =
        std::numeric_limits<double>::quiet_NaN();
    double wheel_twist_expected_delta =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_dt =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_forward =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_lateral =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_local_x =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_local_y =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_forward_velocity =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_translation =
        std::numeric_limits<double>::quiet_NaN();
    double odom_pose_delta_yaw =
        std::numeric_limits<double>::quiet_NaN();
    double odom_latest_age =
        std::numeric_limits<double>::quiet_NaN();
    int odom_history_size = 0;
    int front_point_count = 0;
    double front_point_fraction =
        std::numeric_limits<double>::quiet_NaN();
    bool front_weak = false;
    bool longitudinal_prior_active = false;
    bool longitudinal_replacement_active = false;
    bool longitudinal_motion_loss = false;
    bool straight_longitudinal_mismatch = false;
    int straight_longitudinal_mismatch_streak = 0;
    int motion_loss_prior_hold_count = 0;
    double longitudinal_blend_weight = 0.;
    double longitudinal_prior_weight = 0.;
    double ceres_occupied_space_weight_scale = 1.;
    double ceres_rotation_weight =
        std::numeric_limits<double>::quiet_NaN();
    double longitudinal_prior_target_delta =
        std::numeric_limits<double>::quiet_NaN();
    double longitudinal_prior_wheel_delta_scale =
        std::numeric_limits<double>::quiet_NaN();
    double trigger_scan_forward_delta =
        std::numeric_limits<double>::quiet_NaN();
    double odom_prior_translation =
        std::numeric_limits<double>::quiet_NaN();
    double odom_prior_yaw_rate =
        std::numeric_limits<double>::quiet_NaN();
    double imu_delta_dt = std::numeric_limits<double>::quiet_NaN();
    double imu_delta_yaw = std::numeric_limits<double>::quiet_NaN();
    double imu_delta_yaw_rate = std::numeric_limits<double>::quiet_NaN();
    double command_latest_age =
        std::numeric_limits<double>::quiet_NaN();
    double command_speed = std::numeric_limits<double>::quiet_NaN();
    double command_steering_angle =
        std::numeric_limits<double>::quiet_NaN();
    double command_expected_delta =
        std::numeric_limits<double>::quiet_NaN();
  };

  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  };
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    transform::Rigid3d published_local_pose;
    bool frozen_match_candidate_available = false;
    bool frozen_match_accepted = false;
    sensor::RangeData range_data_in_local;
    LocalSlamQualityMetrics quality_metrics;
    double scan_match_score;
    bool scan_match_score_valid;
    std::string localization_health_state;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  LocalTrajectoryBuilder2D(
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids,
      scan_matching::FrozenSubmapDataProvider frozen_submap_data_provider = {});
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment,
      const absl::optional<common::Duration>& sensor_duration);
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment,
      bool is_outlier);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      LocalSlamQualityMetrics* quality_metrics);
  bool IsLocalSlamOutlier(LocalSlamQualityMetrics* quality_metrics);
  absl::optional<transform::Rigid2d> InterpolateOdometry2D(
      common::Time time) const;
  absl::optional<transform::Rigid2d> InterpolateOrLatestOdometry2D(
      common::Time time) const;

  bool InitializeQualityMetricsCsvWriter();
  void MaybeWriteQualityMetricsCsv(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const transform::Rigid2d& pose_estimate,
      const LocalSlamQualityMetrics& quality_metrics, bool inserted_to_submap,
      int num_insertion_submaps);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  static constexpr size_t kFrozenSubmapMatchStatusCount =
      static_cast<size_t>(
          scan_matching::FrozenSubmapMatchStatus2D::
              kRejectedRotationCorrection) +
      1;

  struct FrozenSubmapTuningStats {
    int64_t num_attempts = 0;
    int64_t num_accepted = 0;
    int64_t num_score_samples = 0;
    int64_t num_margin_samples = 0;
    int64_t num_variance_samples = 0;
    int64_t num_correction_samples = 0;
    int64_t sum_candidates_in_search_radius = 0;
    int64_t sum_candidates_evaluated = 0;
    double sum_best_score = 0.;
    double sum_score_margin = 0.;
    double sum_score_variance = 0.;
    double sum_translation_correction = 0.;
    double sum_rotation_correction = 0.;
    std::array<int64_t, kFrozenSubmapMatchStatusCount> status_counts = {};
  };

  void AccumulateFrozenSubmapTuningStats(
      const scan_matching::FrozenSubmapMatchResult2D& result);
  void MaybeLogFrozenSubmapTuningDetail(
      const scan_matching::FrozenSubmapMatchResult2D& result) const;
  void MaybeLogFrozenSubmapTuningSummary();

  std::string UpdateLocalizationHealthState(
      const scan_matching::FrozenSubmapMatchResult2D& frozen_match_result,
      const LocalSlamQualityMetrics& quality_metrics);

  const proto::LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;

  MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;
  scan_matching::FrozenSubmapDataProvider frozen_submap_data_provider_;
  std::unique_ptr<scan_matching::FrozenSubmapScanMatcher2D>
      frozen_submap_scan_matcher_;

  std::unique_ptr<PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  absl::optional<double> last_thread_cpu_time_seconds_;
  absl::optional<common::Time> last_sensor_time_;
  double latest_scan_match_score_ = 0.;
  bool latest_scan_match_score_valid_ = false;

  int64_t frozen_submap_match_attempt_count_ = 0;
  FrozenSubmapTuningStats frozen_submap_tuning_stats_;
  std::string localization_health_state_ = "GOOD";
  int frozen_match_reject_streak_ = 0;
  int frozen_match_accept_streak_ = 0;
  int frozen_match_warning_streak_ = 0;
  int local_slam_outlier_streak_ = 0;
  int local_slam_hard_outlier_streak_ = 0;

  RangeDataCollator range_data_collator_;
  std::ofstream quality_metrics_csv_;
  bool quality_metrics_csv_enabled_ = false;
  int consecutive_medium_outlier_count_ = 0;
  double latest_imu_angular_velocity_z_ = 0.;
  double integrated_imu_yaw_ = 0.;
  double last_pose_integrated_imu_yaw_ = 0.;
  absl::optional<common::Time> last_imu_time_;
  absl::optional<common::Time> last_pose_estimate_time_;
  absl::optional<transform::Rigid2d> last_pose_estimate_2d_;
  int longitudinal_motion_loss_prior_hold_count_ = 0;
  int straight_longitudinal_mismatch_streak_ = 0;
  std::deque<sensor::OdometryData> odometry_history_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
