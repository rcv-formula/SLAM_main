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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.h"

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/internal/voxel_filter.h"

namespace cartographer {
namespace mapping {

proto::LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions2D options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_min_z(parameter_dictionary->GetDouble("min_z"));
  options.set_max_z(parameter_dictionary->GetDouble("max_z"));
  options.set_missing_data_ray_length(
      parameter_dictionary->GetDouble("missing_data_ray_length"));
  options.set_num_accumulated_range_data(
      parameter_dictionary->GetInt("num_accumulated_range_data"));
  options.set_voxel_filter_size(
      parameter_dictionary->GetDouble("voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_loop_closure_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("loop_closure_adaptive_voxel_filter")
              .get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
      mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      mapping::scan_matching::CreateCeresScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_frozen_submap_scan_matcher_options() =
      mapping::scan_matching::CreateFrozenSubmapScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("frozen_submap_scan_matcher")
              .get());
  *options.mutable_motion_filter_options() = mapping::CreateMotionFilterOptions(
      parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_extrapolator_options() = CreatePoseExtrapolatorOptions(
      parameter_dictionary->GetDictionary("pose_extrapolator").get());
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  *options.mutable_submaps_options() = CreateSubmapsOptions2D(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  options.set_log_local_quality_metrics_to_csv(
      parameter_dictionary->GetBool("log_local_quality_metrics_to_csv"));
  options.set_local_quality_metrics_csv_path(
      parameter_dictionary->GetString("local_quality_metrics_csv_path"));
  options.set_skip_submap_insertion_for_outliers(
      parameter_dictionary->GetBool("skip_submap_insertion_for_outliers"));
  options.set_outlier_min_correlative_score(
      parameter_dictionary->GetDouble("outlier_min_correlative_score"));
  options.set_outlier_max_translation_residual(
      parameter_dictionary->GetDouble("outlier_max_translation_residual"));
  options.set_outlier_max_rotation_residual(
      parameter_dictionary->GetDouble("outlier_max_rotation_residual"));
  options.set_outlier_min_num_filtered_points(
      parameter_dictionary->GetInt("outlier_min_num_filtered_points"));
  options.set_outlier_required_failures(
      parameter_dictionary->GetInt("outlier_required_failures"));
  options.set_outlier_medium_translation_residual(
      parameter_dictionary->GetDouble("outlier_medium_translation_residual"));
  options.set_outlier_medium_rotation_residual(
      parameter_dictionary->GetDouble("outlier_medium_rotation_residual"));
  options.set_outlier_medium_required_consecutive(
      parameter_dictionary->GetInt("outlier_medium_required_consecutive"));
  return options;
}

}  // namespace mapping
}  // namespace cartographer
