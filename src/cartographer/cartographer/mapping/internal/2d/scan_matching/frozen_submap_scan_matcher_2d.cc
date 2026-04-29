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

#include "cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

struct CandidateEvaluation {
  const FrozenSubmapSnapshot2D* snapshot = nullptr;
  double distance_to_submap = 0.;
  float score = 0.f;
  transform::Rigid2d tracking_to_frozen_local = transform::Rigid2d::Identity();
  transform::Rigid2d tracking_to_map = transform::Rigid2d::Identity();
};

double ComputeScoreVariance(const std::vector<CandidateEvaluation>& candidates,
                            const int top_k) {
  const int sample_size = std::min(top_k, static_cast<int>(candidates.size()));
  if (sample_size < 2) {
    return 0.;
  }
  double mean = 0.;
  for (int i = 0; i < sample_size; ++i) {
    mean += candidates[i].score;
  }
  mean /= sample_size;

  double variance = 0.;
  for (int i = 0; i < sample_size; ++i) {
    const double delta = candidates[i].score - mean;
    variance += delta * delta;
  }
  return variance / sample_size;
}

double RotationCorrectionMagnitude(
    const transform::Rigid2d& raw_tracking_to_map,
    const transform::Rigid2d& corrected_tracking_to_map) {
  return std::abs(common::NormalizeAngleDifference(
      corrected_tracking_to_map.rotation().angle() -
      raw_tracking_to_map.rotation().angle()));
}

double TranslationCorrectionMagnitude(
    const transform::Rigid2d& raw_tracking_to_map,
    const transform::Rigid2d& corrected_tracking_to_map) {
  return (corrected_tracking_to_map.translation() -
          raw_tracking_to_map.translation())
      .norm();
}

void PopulateSelectedCandidate(
    const CandidateEvaluation& candidate,
    const transform::Rigid2d& raw_tracking_to_map,
    FrozenSubmapMatchResult2D* const result) {
  result->matched_submap_id = candidate.snapshot->id;
  result->selected_distance_to_submap = candidate.distance_to_submap;
  result->translation_correction = TranslationCorrectionMagnitude(
      raw_tracking_to_map, candidate.tracking_to_map);
  result->rotation_correction = RotationCorrectionMagnitude(
      raw_tracking_to_map, candidate.tracking_to_map);
}

void PopulateCandidateDebugInfo(
    const std::vector<CandidateEvaluation>& candidates,
    const transform::Rigid2d& raw_tracking_to_map, const int top_candidates,
    FrozenSubmapMatchResult2D* const result) {
  if (top_candidates <= 0) {
    return;
  }
  const int num_candidates =
      std::min(top_candidates, static_cast<int>(candidates.size()));
  result->candidate_debug_info.reserve(num_candidates);
  for (int i = 0; i < num_candidates; ++i) {
    const CandidateEvaluation& candidate = candidates[i];
    result->candidate_debug_info.push_back(FrozenSubmapCandidateDebugInfo2D{
        candidate.snapshot->id,
        candidate.score,
        candidate.distance_to_submap,
        TranslationCorrectionMagnitude(raw_tracking_to_map,
                                       candidate.tracking_to_map),
        RotationCorrectionMagnitude(raw_tracking_to_map,
                                    candidate.tracking_to_map)});
  }
}

}  // namespace

proto::FrozenSubmapScanMatcherOptions2D CreateFrozenSubmapScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FrozenSubmapScanMatcherOptions2D options;
  options.set_enabled(parameter_dictionary->GetBool("enabled"));

  const std::string apply_mode_string =
      parameter_dictionary->GetString("apply_mode");
  proto::FrozenSubmapScanMatcherOptions2D_ApplyMode apply_mode;
  CHECK(proto::FrozenSubmapScanMatcherOptions2D_ApplyMode_Parse(
      apply_mode_string, &apply_mode))
      << "Unknown FrozenSubmapScanMatcherOptions2D_ApplyMode kind: "
      << apply_mode_string;
  options.set_apply_mode(apply_mode);

  options.set_search_radius(parameter_dictionary->GetDouble("search_radius"));
  options.set_max_submaps_to_match(
      parameter_dictionary->GetNonNegativeInt("max_submaps_to_match"));
  options.set_use_realtime_correlative_scan_matching(
      parameter_dictionary->GetBool("use_realtime_correlative_scan_matching"));
  options.set_use_ceres_scan_matching(
      parameter_dictionary->GetBool("use_ceres_scan_matching"));
  options.set_min_realtime_correlative_score(
      parameter_dictionary->GetDouble("min_realtime_correlative_score"));
  options.set_min_score_margin(
      parameter_dictionary->GetDouble("min_score_margin"));
  options.set_score_variance_top_k(
      parameter_dictionary->GetNonNegativeInt("score_variance_top_k"));
  options.set_min_score_variance(
      parameter_dictionary->GetDouble("min_score_variance"));
  options.set_max_translation_correction(
      parameter_dictionary->GetDouble("max_translation_correction"));
  options.set_max_rotation_correction(
      parameter_dictionary->GetDouble("max_rotation_correction"));
  options.set_tuning_log_enabled(
      parameter_dictionary->HasKey("tuning_log_enabled")
          ? parameter_dictionary->GetBool("tuning_log_enabled")
          : false);
  options.set_tuning_log_log_rejections(
      parameter_dictionary->HasKey("tuning_log_log_rejections")
          ? parameter_dictionary->GetBool("tuning_log_log_rejections")
          : true);
  options.set_tuning_log_log_acceptances(
      parameter_dictionary->HasKey("tuning_log_log_acceptances")
          ? parameter_dictionary->GetBool("tuning_log_log_acceptances")
          : false);
  options.set_tuning_log_detail_every_n_scans(
      parameter_dictionary->HasKey("tuning_log_detail_every_n_scans")
          ? parameter_dictionary->GetNonNegativeInt(
                "tuning_log_detail_every_n_scans")
          : 20);
  options.set_tuning_log_summary_every_n_scans(
      parameter_dictionary->HasKey("tuning_log_summary_every_n_scans")
          ? parameter_dictionary->GetNonNegativeInt(
                "tuning_log_summary_every_n_scans")
          : 100);
  options.set_tuning_log_top_candidates(
      parameter_dictionary->HasKey("tuning_log_top_candidates")
          ? parameter_dictionary->GetNonNegativeInt("tuning_log_top_candidates")
          : 3);
  options.set_test_mode_publish_filtered_odom(
      parameter_dictionary->HasKey("test_mode_publish_filtered_odom")
          ? parameter_dictionary->GetBool("test_mode_publish_filtered_odom")
          : false);

  *options.mutable_real_time_correlative_scan_matcher_options() =
      mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      mapping::scan_matching::CreateCeresScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());

  CHECK_GE(options.search_radius(), 0.);
  CHECK_GT(options.max_submaps_to_match(), 0);
  CHECK_GE(options.min_realtime_correlative_score(), 0.);
  CHECK_GE(options.min_score_margin(), 0.);
  CHECK_GT(options.score_variance_top_k(), 0);
  CHECK_GE(options.min_score_variance(), 0.);
  CHECK_GE(options.max_translation_correction(), 0.);
  CHECK_GE(options.max_rotation_correction(), 0.);
  CHECK_GE(options.tuning_log_detail_every_n_scans(), 0);
  CHECK_GE(options.tuning_log_summary_every_n_scans(), 0);
  CHECK_GE(options.tuning_log_top_candidates(), 0);
  CHECK(options.use_realtime_correlative_scan_matching() ||
        options.use_ceres_scan_matching());
  return options;
}

FrozenSubmapScanMatcher2D::FrozenSubmapScanMatcher2D(
    const proto::FrozenSubmapScanMatcherOptions2D& options)
    : options_(options),
      real_time_correlative_scan_matcher_(
          options.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

FrozenSubmapMatchResult2D FrozenSubmapScanMatcher2D::Match(
    const FrozenSubmapQueryResult2D& query,
    const transform::Rigid2d& raw_tracking_to_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) const {
  FrozenSubmapMatchResult2D result;
  result.filtered_tracking_to_local = raw_tracking_to_local;
  result.filtered_tracking_to_map = query.local_to_map * raw_tracking_to_local;
  if (!options_.enabled() || query.submaps.empty() ||
      filtered_gravity_aligned_point_cloud.empty()) {
    return result;
  }

  result.attempted = true;
  const transform::Rigid2d raw_tracking_to_map = result.filtered_tracking_to_map;

  std::vector<CandidateEvaluation> candidate_evaluations;
  candidate_evaluations.reserve(query.submaps.size());
  for (const auto& snapshot : query.submaps) {
    if (!snapshot.submap || snapshot.submap->grid() == nullptr) {
      continue;
    }
    const double distance_to_submap =
        (snapshot.submap_to_map.translation() - raw_tracking_to_map.translation())
            .norm();
    if (distance_to_submap > options_.search_radius()) {
      continue;
    }
    candidate_evaluations.push_back(
        CandidateEvaluation{&snapshot, distance_to_submap, 0.f,
                            transform::Rigid2d::Identity(),
                            transform::Rigid2d::Identity()});
  }
  result.num_candidates_in_search_radius = candidate_evaluations.size();
  if (candidate_evaluations.empty()) {
    result.status = FrozenSubmapMatchStatus2D::kRejectedNoCandidates;
    return result;
  }

  std::sort(candidate_evaluations.begin(), candidate_evaluations.end(),
            [](const CandidateEvaluation& lhs, const CandidateEvaluation& rhs) {
              return lhs.distance_to_submap < rhs.distance_to_submap;
            });
  if (candidate_evaluations.size() >
      static_cast<size_t>(options_.max_submaps_to_match())) {
    candidate_evaluations.resize(options_.max_submaps_to_match());
  }
  if (!options_.use_realtime_correlative_scan_matching()) {
    candidate_evaluations.resize(1);
  }
  result.num_candidates_evaluated = candidate_evaluations.size();

  for (auto& candidate : candidate_evaluations) {
    const transform::Rigid2d raw_tracking_to_frozen_local =
        candidate.snapshot->frozen_local_to_map.inverse() * raw_tracking_to_map;
    transform::Rigid2d refined_tracking_to_frozen_local =
        raw_tracking_to_frozen_local;
    if (options_.use_realtime_correlative_scan_matching()) {
      candidate.score = real_time_correlative_scan_matcher_.Match(
          raw_tracking_to_frozen_local, filtered_gravity_aligned_point_cloud,
          *candidate.snapshot->submap->grid(),
          &refined_tracking_to_frozen_local);
    }
    if (options_.use_ceres_scan_matching()) {
      ceres::Solver::Summary summary;
      ceres_scan_matcher_.Match(
          raw_tracking_to_frozen_local.translation(),
          refined_tracking_to_frozen_local,
          filtered_gravity_aligned_point_cloud,
          *candidate.snapshot->submap->grid(),
          &refined_tracking_to_frozen_local, &summary);
    }
    candidate.tracking_to_frozen_local = refined_tracking_to_frozen_local;
    candidate.tracking_to_map =
        candidate.snapshot->frozen_local_to_map *
        refined_tracking_to_frozen_local;
  }

  if (options_.use_realtime_correlative_scan_matching()) {
    std::sort(candidate_evaluations.begin(), candidate_evaluations.end(),
              [](const CandidateEvaluation& lhs,
                 const CandidateEvaluation& rhs) {
                if (lhs.score == rhs.score) {
                  return lhs.distance_to_submap < rhs.distance_to_submap;
                }
                return lhs.score > rhs.score;
              });

    result.best_score = candidate_evaluations.front().score;
    if (candidate_evaluations.size() > 1) {
      result.second_best_score = candidate_evaluations[1].score;
    }
    result.top_k_score_variance = ComputeScoreVariance(
        candidate_evaluations, options_.score_variance_top_k());
  }

  const CandidateEvaluation& best_candidate = candidate_evaluations.front();
  PopulateSelectedCandidate(best_candidate, raw_tracking_to_map, &result);
  PopulateCandidateDebugInfo(candidate_evaluations, raw_tracking_to_map,
                             options_.tuning_log_top_candidates(), &result);

  if (options_.use_realtime_correlative_scan_matching()) {
    if (result.best_score < options_.min_realtime_correlative_score()) {
      result.status = FrozenSubmapMatchStatus2D::kRejectedLowScore;
      return result;
    }
    if (candidate_evaluations.size() > 1 &&
        result.best_score - result.second_best_score <
            options_.min_score_margin()) {
      result.status = FrozenSubmapMatchStatus2D::kRejectedLowMargin;
      return result;
    }
    if (std::min(options_.score_variance_top_k(),
                 static_cast<int>(candidate_evaluations.size())) >= 2 &&
        result.top_k_score_variance < options_.min_score_variance()) {
      result.status = FrozenSubmapMatchStatus2D::kRejectedLowVariance;
      return result;
    }
  }

  if (result.translation_correction > options_.max_translation_correction()) {
    result.status =
        FrozenSubmapMatchStatus2D::kRejectedTranslationCorrection;
    return result;
  }
  if (result.rotation_correction > options_.max_rotation_correction()) {
    result.status = FrozenSubmapMatchStatus2D::kRejectedRotationCorrection;
    return result;
  }

  result.accepted = true;
  result.status = FrozenSubmapMatchStatus2D::kAccepted;
  result.filtered_tracking_to_map = best_candidate.tracking_to_map;
  result.filtered_tracking_to_local =
      query.local_to_map.inverse() * best_candidate.tracking_to_map;
  return result;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
