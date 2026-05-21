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

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {

class LongitudinalTranslationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation,
      const Eigen::Vector2d& target_heading) {
    return new ceres::AutoDiffCostFunction<
        LongitudinalTranslationDeltaCostFunctor2D, 1 /* residuals */,
        3 /* pose variables */>(
        new LongitudinalTranslationDeltaCostFunctor2D(
            scaling_factor, target_translation, target_heading.normalized()));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ *
                  ((pose[0] - x_) * heading_x_ + (pose[1] - y_) * heading_y_);
    return true;
  }

 private:
  LongitudinalTranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation,
      const Eigen::Vector2d& target_heading)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()),
        heading_x_(target_heading.x()),
        heading_y_(target_heading.y()) {}

  const double scaling_factor_;
  const double x_;
  const double y_;
  const double heading_x_;
  const double heading_y_;
};

}  // namespace

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_longitudinal_translation_weight(
      parameter_dictionary->GetDouble("longitudinal_translation_weight"));
  options.set_longitudinal_translation_min_speed(
      parameter_dictionary->GetDouble("longitudinal_translation_min_speed"));
  options.set_longitudinal_translation_max_yaw_rate(
      parameter_dictionary->GetDouble("longitudinal_translation_max_yaw_rate"));
  options.set_longitudinal_prior_wheel_delta_scale(
      parameter_dictionary->GetDouble("longitudinal_prior_wheel_delta_scale"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(
    const Eigen::Vector2d& target_translation,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid,
    transform::Rigid2d* const pose_estimate,
    ceres::Solver::Summary* const summary) const {
  Match(target_translation, Eigen::Vector2d::UnitX(), target_translation,
        0. /* longitudinal_translation_weight */, options_.rotation_weight(),
        initial_pose_estimate, point_cloud, grid, pose_estimate, summary);
}

void CeresScanMatcher2D::Match(
    const Eigen::Vector2d& target_translation,
    const Eigen::Vector2d& target_heading,
    const Eigen::Vector2d& longitudinal_target_translation,
    const double longitudinal_translation_weight,
    const double rotation_weight,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid,
    transform::Rigid2d* const pose_estimate,
    ceres::Solver::Summary* const summary) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_weight(), 0.);
  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
    case GridType::TSDF:
      problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D&>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
  }
  CHECK_GT(options_.translation_weight(), 0.);
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.translation_weight(), target_translation),
      nullptr /* loss function */, ceres_pose_estimate);
  if (longitudinal_translation_weight > 0.) {
    problem.AddResidualBlock(
        LongitudinalTranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            longitudinal_translation_weight, longitudinal_target_translation,
            target_heading),
        new ceres::HuberLoss(0.05), ceres_pose_estimate);
  }
  CHECK_GT(rotation_weight, 0.);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          rotation_weight, ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
