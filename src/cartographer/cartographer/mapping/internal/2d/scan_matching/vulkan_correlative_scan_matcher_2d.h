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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_VULKAN_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_VULKAN_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

bool ScoreCandidatesWithVulkan(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    double translation_delta_cost_weight, double rotation_delta_cost_weight,
    std::vector<Candidate2D>* candidates);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_VULKAN_CORRELATIVE_SCAN_MATCHER_2D_H_
