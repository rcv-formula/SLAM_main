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

#include "cartographer_ros/node_options.h"

#include <cstdlib>
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "glog/logging.h"

namespace cartographer_ros {
namespace {

void SetEnv(const std::string& name, const std::string& value) {
  setenv(name.c_str(), value.c_str(), 1);
}

void SetBoolEnvIfPresent(
    ::cartographer::common::LuaParameterDictionary* const dictionary,
    const std::string& lua_key, const std::string& env_key) {
  if (dictionary->HasKey(lua_key)) {
    SetEnv(env_key, dictionary->GetBool(lua_key) ? "true" : "false");
  }
}

void SetDoubleEnvIfPresent(
    ::cartographer::common::LuaParameterDictionary* const dictionary,
    const std::string& lua_key, const std::string& env_key) {
  if (dictionary->HasKey(lua_key)) {
    SetEnv(env_key, std::to_string(dictionary->GetDouble(lua_key)));
  }
}

void SetStringEnvIfPresent(
    ::cartographer::common::LuaParameterDictionary* const dictionary,
    const std::string& lua_key, const std::string& env_key) {
  if (dictionary->HasKey(lua_key)) {
    SetEnv(env_key, dictionary->GetString(lua_key));
  }
}

void ApplyDamviRuntimeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  if (!lua_parameter_dictionary->HasKey("damvi_runtime_options")) {
    return;
  }
  auto runtime_options =
      lua_parameter_dictionary->GetDictionary("damvi_runtime_options");

  SetBoolEnvIfPresent(runtime_options.get(), "wheel_odom_twist_only",
                      "WHEEL_ODOM_TWIST_ONLY");
  SetDoubleEnvIfPresent(runtime_options.get(), "wheel_odom_linear_scale",
                        "WHEEL_ODOM_LINEAR_SCALE");
  SetDoubleEnvIfPresent(
      runtime_options.get(), "longitudinal_prior_occupied_space_weight_scale",
      "CARTOGRAPHER_LONGITUDINAL_PRIOR_OCCUPIED_SPACE_WEIGHT_SCALE");

  SetBoolEnvIfPresent(runtime_options.get(), "adaptive_odometry_blend",
                      "CARTOGRAPHER_ADAPTIVE_ODOMETRY_BLEND");
  SetDoubleEnvIfPresent(
      runtime_options.get(), "adaptive_odometry_full_weight_yaw_rate",
      "CARTOGRAPHER_ADAPTIVE_ODOMETRY_FULL_WEIGHT_YAW_RATE");
  SetDoubleEnvIfPresent(
      runtime_options.get(), "adaptive_odometry_zero_weight_yaw_rate",
      "CARTOGRAPHER_ADAPTIVE_ODOMETRY_ZERO_WEIGHT_YAW_RATE");
  SetDoubleEnvIfPresent(runtime_options.get(), "adaptive_odometry_min_weight",
                        "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MIN_WEIGHT");
  SetDoubleEnvIfPresent(runtime_options.get(), "adaptive_odometry_max_weight",
                        "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MAX_WEIGHT");
  SetBoolEnvIfPresent(
      runtime_options.get(), "adaptive_odometry_mismatch_override",
      "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MISMATCH_OVERRIDE");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "adaptive_odometry_mismatch_ratio",
                        "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MISMATCH_RATIO");
  SetDoubleEnvIfPresent(
      runtime_options.get(), "adaptive_odometry_min_forward_delta",
      "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MIN_FORWARD_DELTA");
  SetDoubleEnvIfPresent(
      runtime_options.get(), "adaptive_odometry_mismatch_force_weight",
      "CARTOGRAPHER_ADAPTIVE_ODOMETRY_MISMATCH_FORCE_WEIGHT");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "adaptive_odometry_longitudinal_only",
                      "CARTOGRAPHER_ADAPTIVE_ODOMETRY_LONGITUDINAL_ONLY");

  SetBoolEnvIfPresent(runtime_options.get(), "clamp_local_lateral_residual",
                      "CARTOGRAPHER_CLAMP_LOCAL_LATERAL_RESIDUAL");
  SetDoubleEnvIfPresent(runtime_options.get(), "local_lateral_residual_max",
                        "CARTOGRAPHER_LOCAL_LATERAL_RESIDUAL_MAX");
  SetBoolEnvIfPresent(runtime_options.get(), "reject_local_slam_outliers",
                      "CARTOGRAPHER_REJECT_LOCAL_SLAM_OUTLIERS");
  SetDoubleEnvIfPresent(runtime_options.get(), "imu_yaw_weight",
                        "CARTOGRAPHER_IMU_YAW_WEIGHT");
  SetDoubleEnvIfPresent(runtime_options.get(), "wheel_odom_yaw_weight",
                        "WHEEL_ODOM_YAW_WEIGHT");

  SetBoolEnvIfPresent(runtime_options.get(), "restart_on_lost",
                      "CARTOGRAPHER_RESTART_ON_LOCALIZATION_LOST");
  SetDoubleEnvIfPresent(runtime_options.get(), "restart_lost_after_sec",
                        "CARTOGRAPHER_RESTART_LOST_AFTER_SEC");
  SetDoubleEnvIfPresent(runtime_options.get(), "restart_cooldown_sec",
                        "CARTOGRAPHER_RESTART_COOLDOWN_SEC");

  SetStringEnvIfPresent(runtime_options.get(),
                        "pose_graph_constraint_metrics_csv_path",
                        "POSE_GRAPH_CONSTRAINT_METRICS_CSV_PATH");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_ambiguous_constraint_downweight",
                      "POSE_GRAPH_AMBIGUOUS_CONSTRAINT_DOWNWEIGHT");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_ambiguous_constraint_reject",
                      "POSE_GRAPH_AMBIGUOUS_CONSTRAINT_REJECT");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_ambiguous_apply_to_tracking",
                      "POSE_GRAPH_AMBIGUOUS_APPLY_TO_TRACKING");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_ambiguous_apply_to_initial",
                      "POSE_GRAPH_AMBIGUOUS_APPLY_TO_INITIAL");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_ambiguous_apply_to_recovery",
                      "POSE_GRAPH_AMBIGUOUS_APPLY_TO_RECOVERY");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_ambiguous_constraint_reject_min_score",
                        "POSE_GRAPH_AMBIGUOUS_CONSTRAINT_REJECT_MIN_SCORE");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_ambiguous_constraint_min_translation",
                        "POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MIN_TRANSLATION");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_ambiguous_constraint_max_score_margin",
                        "POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MAX_SCORE_MARGIN");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_ambiguous_constraint_min_near_top_count",
                        "POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MIN_NEAR_TOP_COUNT");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_reject_ambiguous_full_submap",
                      "POSE_GRAPH_REJECT_AMBIGUOUS_FULL_SUBMAP");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_ambiguous_full_submap_max_score_margin",
                        "POSE_GRAPH_AMBIGUOUS_FULL_SUBMAP_MAX_SCORE_MARGIN");
  SetDoubleEnvIfPresent(
      runtime_options.get(), "pose_graph_ambiguous_full_submap_reject_min_score",
      "POSE_GRAPH_AMBIGUOUS_FULL_SUBMAP_REJECT_MIN_SCORE");
  SetDoubleEnvIfPresent(
      runtime_options.get(), "pose_graph_ambiguous_full_submap_min_near_top_count",
      "POSE_GRAPH_AMBIGUOUS_FULL_SUBMAP_MIN_NEAR_TOP_COUNT");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_bound_relocalization_to_prior",
                      "POSE_GRAPH_BOUND_RELOCALIZATION_TO_PRIOR");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_bound_tracking_global_to_prior",
                      "POSE_GRAPH_BOUND_TRACKING_GLOBAL_TO_PRIOR");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_tracking_prior_min_score",
                        "POSE_GRAPH_TRACKING_PRIOR_MIN_SCORE");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_tracking_max_translation_correction",
                        "POSE_GRAPH_TRACKING_MAX_TRANSLATION_CORRECTION");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_tracking_max_yaw_correction",
                        "POSE_GRAPH_TRACKING_MAX_YAW_CORRECTION");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_relocalization_prior_min_score",
                        "POSE_GRAPH_RELOCALIZATION_PRIOR_MIN_SCORE");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_relocalization_max_translation_correction",
                        "POSE_GRAPH_RELOCALIZATION_MAX_TRANSLATION_CORRECTION");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_relocalization_max_yaw_correction",
                        "POSE_GRAPH_RELOCALIZATION_MAX_YAW_CORRECTION");
  SetDoubleEnvIfPresent(runtime_options.get(),
                        "pose_graph_recovery_constraint_weight_scale",
                        "POSE_GRAPH_RECOVERY_CONSTRAINT_WEIGHT_SCALE");
  SetBoolEnvIfPresent(runtime_options.get(),
                      "pose_graph_disable_relocalization_after_initial",
                      "POSE_GRAPH_DISABLE_RELOCALIZATION_AFTER_INITIAL");
}

}  // namespace

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  ApplyDamviRuntimeOptions(lua_parameter_dictionary);

  NodeOptions options;
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
    options.publish_to_tf =
        lua_parameter_dictionary->GetBool("publish_to_tf");
  }
  if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
    options.publish_tracked_pose =
        lua_parameter_dictionary->GetBool("publish_tracked_pose");
  }
  if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
    options.use_pose_extrapolator =
        lua_parameter_dictionary->GetBool("use_pose_extrapolator");
  }
  return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace cartographer_ros
