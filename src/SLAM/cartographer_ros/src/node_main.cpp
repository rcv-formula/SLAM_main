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

#include <cstdlib>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_bool(use_initial_pose, false,
            "Start the default trajectory with an explicit initial pose "
            "relative to initial_pose_relative_to_trajectory_id.");
DEFINE_double(initial_pose_x, 0.,
              "Initial trajectory pose x in the relative trajectory frame.");
DEFINE_double(initial_pose_y, 0.,
              "Initial trajectory pose y in the relative trajectory frame.");
DEFINE_double(initial_pose_yaw, 0.,
              "Initial trajectory pose yaw in radians in the relative "
              "trajectory frame.");
DEFINE_int32(initial_pose_relative_to_trajectory_id, 0,
             "Trajectory ID that the initial pose is relative to. For normal "
             "single-map localization this is the frozen pbstream trajectory.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

int GetExecutorThreadCount() {
  constexpr int kDefaultExecutorThreads = 1;
  const char* value = std::getenv("CARTOGRAPHER_ROS_EXECUTOR_THREADS");
  if (value == nullptr) {
    return kDefaultExecutorThreads;
  }
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || parsed <= 0) {
    LOG(WARNING) << "Ignoring invalid CARTOGRAPHER_ROS_EXECUTOR_THREADS="
                 << value << ". Using " << kDefaultExecutorThreads << ".";
    return kDefaultExecutorThreads;
  }
  return static_cast<int>(parsed);
}

void Run() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    if (FLAGS_use_initial_pose) {
      ::cartographer::mapping::proto::InitialTrajectoryPose
          initial_trajectory_pose;
      initial_trajectory_pose.set_to_trajectory_id(
          FLAGS_initial_pose_relative_to_trajectory_id);
      const ::cartographer::transform::Rigid3d initial_pose(
          Eigen::Vector3d(FLAGS_initial_pose_x, FLAGS_initial_pose_y, 0.),
          Eigen::AngleAxisd(FLAGS_initial_pose_yaw, Eigen::Vector3d::UnitZ()));
      *initial_trajectory_pose.mutable_relative_pose() =
          ::cartographer::transform::ToProto(initial_pose);
      initial_trajectory_pose.set_timestamp(
          ::cartographer::common::ToUniversal(
              ::cartographer::common::FromUniversal(0)));
      *trajectory_options.trajectory_builder_options
           .mutable_initial_trajectory_pose() = initial_trajectory_pose;
      LOG(INFO) << "Starting trajectory with initial pose relative to "
                << FLAGS_initial_pose_relative_to_trajectory_id << ": x="
                << FLAGS_initial_pose_x << " y=" << FLAGS_initial_pose_y
                << " yaw=" << FLAGS_initial_pose_yaw;
    }
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  const int executor_threads = GetExecutorThreadCount();
  LOG(INFO) << "Spinning cartographer_node with MultiThreadedExecutor using "
            << executor_threads << " threads.";
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), executor_threads);
  executor.add_node(cartographer_node);
  executor.spin();

  node->FinishAllTrajectories();
  node->RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::rclcpp::shutdown();
}
