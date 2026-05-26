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

#ifndef CARTOGRAPHER_COMMON_INTERNAL_CERES_MANIFOLD_COMPAT_H_
#define CARTOGRAPHER_COMMON_INTERNAL_CERES_MANIFOLD_COMPAT_H_

#include <memory>
#include <vector>

#include "absl/memory/memory.h"
#include "ceres/ceres.h"
#include "ceres/version.h"

#if CERES_VERSION_MAJOR > 2 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
#include "ceres/autodiff_manifold.h"
#include "ceres/manifold.h"
#else
#include "ceres/autodiff_local_parameterization.h"
#include "ceres/local_parameterization.h"
#endif

namespace cartographer {
namespace common {

#if CERES_VERSION_MAJOR > 2 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
using CeresParameterization = ceres::Manifold;

inline void SetParameterization(
    ceres::Problem* const problem, double* const parameter_block,
    CeresParameterization* const parameterization) {
  problem->SetManifold(parameter_block, parameterization);
}

inline std::unique_ptr<CeresParameterization>
MakeQuaternionParameterization() {
  return absl::make_unique<ceres::QuaternionManifold>();
}

inline std::unique_ptr<CeresParameterization> MakeSubsetParameterization(
    const int size, const std::vector<int>& constant_parameters) {
  return absl::make_unique<ceres::SubsetManifold>(size, constant_parameters);
}

template <typename Functor, int kAmbientSize, int kTangentSize>
std::unique_ptr<CeresParameterization> MakeAutoDiffParameterization() {
  return absl::make_unique<
      ceres::AutoDiffManifold<Functor, kAmbientSize, kTangentSize>>();
}
#else
using CeresParameterization = ceres::LocalParameterization;

inline void SetParameterization(
    ceres::Problem* const problem, double* const parameter_block,
    CeresParameterization* const parameterization) {
  problem->SetParameterization(parameter_block, parameterization);
}

inline std::unique_ptr<CeresParameterization>
MakeQuaternionParameterization() {
  return absl::make_unique<ceres::QuaternionParameterization>();
}

inline std::unique_ptr<CeresParameterization> MakeSubsetParameterization(
    const int size, const std::vector<int>& constant_parameters) {
  return absl::make_unique<ceres::SubsetParameterization>(
      size, constant_parameters);
}

template <typename Functor, int kAmbientSize, int kTangentSize>
std::unique_ptr<CeresParameterization> MakeAutoDiffParameterization() {
  return absl::make_unique<ceres::AutoDiffLocalParameterization<
      Functor, kAmbientSize, kTangentSize>>();
}
#endif

inline void SetParameterization(
    ceres::Problem* const problem, double* const parameter_block,
    std::unique_ptr<CeresParameterization> parameterization) {
  if (parameterization != nullptr) {
    SetParameterization(problem, parameter_block, parameterization.release());
  }
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_INTERNAL_CERES_MANIFOLD_COMPAT_H_
