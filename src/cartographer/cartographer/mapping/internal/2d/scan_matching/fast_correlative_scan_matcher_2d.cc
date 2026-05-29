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

#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

std::atomic<int64_t> g_score_distribution_match_id{0};

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
class SlidingWindowMaximum {
 public:
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  options.set_log_score_distribution_to_csv(
      parameter_dictionary->GetBool("log_score_distribution_to_csv"));
  options.set_score_distribution_csv_path(
      parameter_dictionary->GetString("score_distribution_csv_path"));
  options.set_min_score_distribution_margin(
      parameter_dictionary->GetDouble("min_score_distribution_margin"));
  return options;
}

PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()),
      max_score_(1.f - grid.GetMinCorrespondenceCost()),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);
  const int max_width = 1 << (options.branch_and_bound_depth() - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack2D>(grid, options)) {
  score_distribution_csv_enabled_ = InitializeScoreDistributionCsvWriter();
}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate,
    ScoreDistributionSummary* score_distribution_summary) const {
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate, "local_window",
                                   score_distribution_summary);
}

bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate,
    ScoreDistributionSummary* score_distribution_summary) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate,
                                   "full_submap", score_distribution_summary);
}

bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate, const char* const match_type,
    ScoreDistributionSummary* score_distribution_summary) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  if (lowest_resolution_candidates.empty()) {
    if (score_distribution_summary != nullptr) {
      *score_distribution_summary = ScoreDistributionSummary{};
    }
    return false;
  }
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  const bool score_accepted = best_candidate.score > min_score;

  // Distribution-based ambiguity filter: reject matches where the top-1 and
  // top-2 lowest-resolution candidates are too close in score, indicating
  // perceptual aliasing (e.g., repetitive straight-track sections).
  bool distribution_accepted = true;
  if (score_accepted && options_.min_score_distribution_margin() > 0.0 &&
      lowest_resolution_candidates.size() >= 2) {
    const float margin = lowest_resolution_candidates[0].score -
                         lowest_resolution_candidates[1].score;
    if (margin < static_cast<float>(options_.min_score_distribution_margin())) {
      distribution_accepted = false;
      LOG_EVERY_N(INFO, 50) << "[" << match_type << "] Rejected by distribution filter: "
                << "top1=" << lowest_resolution_candidates[0].score
                << " top2=" << lowest_resolution_candidates[1].score
                << " margin=" << margin
                << " threshold=" << options_.min_score_distribution_margin();
    }
  }
  const bool accepted = score_accepted && distribution_accepted;

  if (score_distribution_summary != nullptr) {
    const float top1 = lowest_resolution_candidates.front().score;
    int near_top_count_0p02 = 0;
    for (const Candidate2D& candidate : lowest_resolution_candidates) {
      if (candidate.score < top1 - 0.02f) {
        break;
      }
      ++near_top_count_0p02;
    }
    score_distribution_summary->candidate_count =
        static_cast<int>(lowest_resolution_candidates.size());
    score_distribution_summary->top1_score = top1;
    score_distribution_summary->top2_score =
        lowest_resolution_candidates.size() > 1
            ? lowest_resolution_candidates[1].score
            : std::numeric_limits<float>::quiet_NaN();
    score_distribution_summary->top3_score =
        lowest_resolution_candidates.size() > 2
            ? lowest_resolution_candidates[2].score
            : std::numeric_limits<float>::quiet_NaN();
    score_distribution_summary->near_top_count_0p02 = near_top_count_0p02;
    score_distribution_summary->accepted = accepted;
    score_distribution_summary->final_best_score = best_candidate.score;
  }
  MaybeWriteScoreDistributionCsv(match_type, min_score,
                                 lowest_resolution_candidates,
                                 best_candidate.score, accepted);
  if (accepted) {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

bool FastCorrelativeScanMatcher2D::InitializeScoreDistributionCsvWriter() {
  if (!options_.log_score_distribution_to_csv()) {
    return false;
  }
  if (options_.score_distribution_csv_path().empty()) {
    LOG(WARNING) << "Fast correlative score distribution CSV logging is "
                    "enabled, but 'score_distribution_csv_path' is empty.";
    return false;
  }

  const std::string& path = options_.score_distribution_csv_path();
  bool write_header = true;
  {
    std::ifstream existing_file(path);
    write_header = !existing_file.good() ||
                   existing_file.peek() == std::ifstream::traits_type::eof();
  }

  score_distribution_csv_.open(path, std::ios::out | std::ios::app);
  if (!score_distribution_csv_.is_open()) {
    LOG(ERROR) << "Failed to open fast correlative score distribution CSV "
               << "file: " << path;
    return false;
  }
  score_distribution_csv_ << std::setprecision(17);
  if (write_header) {
    score_distribution_csv_
        << "match_id,match_type,min_score,accepted,final_best_score,"
           "candidate_count,score_min,score_max,score_mean,score_stddev,"
           "score_p50,score_p90,score_p95,score_p99,top1,top2,top3,"
           "top1_top2_margin,top1_top3_margin,near_top_0p01,"
           "near_top_0p02,near_top_0p05\n";
    score_distribution_csv_.flush();
  }
  return true;
}

void FastCorrelativeScanMatcher2D::MaybeWriteScoreDistributionCsv(
    const char* const match_type, const float min_score,
    const std::vector<Candidate2D>& candidates, const float final_best_score,
    const bool accepted) const {
  if (!score_distribution_csv_enabled_ || candidates.empty()) {
    return;
  }

  const auto score_at_percentile = [&candidates](const double percentile) {
    const double clamped_percentile = std::min(1., std::max(0., percentile));
    const int index = common::RoundToInt(
        (1. - clamped_percentile) * (candidates.size() - 1));
    return candidates[index].score;
  };
  const float top1 = candidates[0].score;
  const float top2 =
      candidates.size() > 1 ? candidates[1].score
                            : std::numeric_limits<float>::quiet_NaN();
  const float top3 =
      candidates.size() > 2 ? candidates[2].score
                            : std::numeric_limits<float>::quiet_NaN();
  double sum = 0.;
  for (const Candidate2D& candidate : candidates) {
    sum += candidate.score;
  }
  const double mean = sum / candidates.size();
  double squared_error_sum = 0.;
  for (const Candidate2D& candidate : candidates) {
    const double error = candidate.score - mean;
    squared_error_sum += error * error;
  }
  const double stddev = std::sqrt(squared_error_sum / candidates.size());

  const auto count_near_top = [&candidates, top1](const float delta) {
    int count = 0;
    for (const Candidate2D& candidate : candidates) {
      if (candidate.score < top1 - delta) {
        break;
      }
      ++count;
    }
    return count;
  };

  std::lock_guard<std::mutex> lock(score_distribution_csv_mutex_);
  score_distribution_csv_
      << g_score_distribution_match_id.fetch_add(1) << ',' << match_type << ','
      << min_score << ',' << static_cast<int>(accepted) << ','
      << final_best_score << ',' << candidates.size() << ','
      << candidates.back().score << ',' << top1 << ',' << mean << ','
      << stddev << ',' << score_at_percentile(0.50) << ','
      << score_at_percentile(0.90) << ',' << score_at_percentile(0.95) << ','
      << score_at_percentile(0.99) << ',' << top1 << ',' << top2 << ','
      << top3 << ',' << top1 - top2 << ',' << top1 - top3 << ','
      << count_near_top(0.01f) << ',' << count_near_top(0.02f) << ','
      << count_near_top(0.05f) << '\n';
  score_distribution_csv_.flush();
}

std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  for (const Candidate2D& candidate : candidates) {
    if (candidate.score <= min_score) {
      break;
    }
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
