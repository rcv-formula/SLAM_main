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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_FROZEN_SUBMAP_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_FROZEN_SUBMAP_SCAN_MATCHER_2D_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/types/optional.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/proto/scan_matching/frozen_submap_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::FrozenSubmapScanMatcherOptions2D CreateFrozenSubmapScanMatcherOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

struct FrozenSubmapSnapshot2D {
  SubmapId id;
  std::shared_ptr<const Submap2D> submap;
  transform::Rigid2d frozen_local_to_map;
  transform::Rigid2d submap_to_map;
};

struct FrozenSubmapQueryResult2D {
  transform::Rigid2d local_to_map = transform::Rigid2d::Identity();
  std::vector<FrozenSubmapSnapshot2D> submaps;
};

using FrozenSubmapDataProvider = std::function<FrozenSubmapQueryResult2D()>;

enum class FrozenSubmapMatchStatus2D {
  kNotAttempted = 0,
  kAccepted,
  kRejectedNoCandidates,
  kRejectedLowScore,
  kRejectedLowMargin,
  kRejectedLowVariance,
  kRejectedTranslationCorrection,
  kRejectedRotationCorrection,
};

struct FrozenSubmapCandidateDebugInfo2D {
  SubmapId submap_id;
  float score = 0.f;
  double distance_to_submap = 0.;
  double translation_correction = 0.;
  double rotation_correction = 0.;
};

struct FrozenSubmapMatchResult2D {
  bool attempted = false;
  bool accepted = false;
  FrozenSubmapMatchStatus2D status =
      FrozenSubmapMatchStatus2D::kNotAttempted;
  transform::Rigid2d filtered_tracking_to_local = transform::Rigid2d::Identity();
  transform::Rigid2d filtered_tracking_to_map = transform::Rigid2d::Identity();
  int num_candidates_in_search_radius = 0;
  int num_candidates_evaluated = 0;
  float best_score = 0.f;
  float second_best_score = 0.f;
  double top_k_score_variance = 0.;
  double selected_distance_to_submap = 0.;
  double translation_correction = 0.;
  double rotation_correction = 0.;
  absl::optional<SubmapId> matched_submap_id;
  std::vector<FrozenSubmapCandidateDebugInfo2D> candidate_debug_info;
};

class FrozenSubmapScanMatcher2D {
 public:
  explicit FrozenSubmapScanMatcher2D(
      const proto::FrozenSubmapScanMatcherOptions2D& options);

  FrozenSubmapScanMatcher2D(const FrozenSubmapScanMatcher2D&) = delete;
  FrozenSubmapScanMatcher2D& operator=(const FrozenSubmapScanMatcher2D&) =
      delete;

  FrozenSubmapMatchResult2D Match(
      const FrozenSubmapQueryResult2D& query,
      const transform::Rigid2d& raw_tracking_to_local,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud) const;

 private:
  const proto::FrozenSubmapScanMatcherOptions2D options_;
  RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;
  CeresScanMatcher2D ceres_scan_matcher_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_FROZEN_SUBMAP_SCAN_MATCHER_2D_H_
