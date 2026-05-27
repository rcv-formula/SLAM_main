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

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kNumSubmapScanMatchersMetric = metrics::Gauge::Null();

bool EnvBool(const char* name, const bool default_value) {
  const char* value = std::getenv(name);
  if (value == nullptr) {
    return default_value;
  }
  const std::string text(value);
  if (text == "1" || text == "true" || text == "TRUE" || text == "on") {
    return true;
  }
  if (text == "0" || text == "false" || text == "FALSE" || text == "off") {
    return false;
  }
  return default_value;
}

double EnvDouble(const char* name, const double default_value) {
  const char* value = std::getenv(name);
  if (value == nullptr) {
    return default_value;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  return end == value ? default_value : parsed;
}

std::mutex* GetConstraintMetricsCsvMutex() {
  static auto* const mutex = new std::mutex;
  return mutex;
}

std::ofstream* GetConstraintMetricsCsv() {
  static auto* const csv = []() -> std::ofstream* {
    const char* path = std::getenv("POSE_GRAPH_CONSTRAINT_METRICS_CSV_PATH");
    if (path == nullptr || std::string(path).empty()) {
      return nullptr;
    }
    auto* stream = new std::ofstream(path, std::ios::out | std::ios::app);
    if (!stream->is_open()) {
      LOG(WARNING) << "Failed to open pose graph constraint metrics CSV: "
                   << path;
      delete stream;
      return nullptr;
    }
    if (stream->tellp() == 0) {
      *stream
          << "status,match_full_submap,submap_trajectory_id,submap_index,"
             "node_trajectory_id,node_index,point_count,is_outlier,"
             "score,min_score,initial_pose_x,initial_pose_y,initial_pose_yaw,"
             "fast_pose_x,fast_pose_y,fast_pose_yaw,ceres_pose_x,ceres_pose_y,"
             "ceres_pose_yaw,constraint_x,constraint_y,constraint_yaw,"
             "initial_to_final_translation,initial_to_final_yaw,"
             "fcsm_candidate_count,fcsm_top1,fcsm_top2,fcsm_top1_top2_margin,"
             "fcsm_near_top_0p02,ambiguous_downweighted,weight_scale,"
             "translation_weight,rotation_weight\n";
      stream->flush();
    }
    stream->precision(17);
    return stream;
  }();
  return csv;
}

void MaybeWriteConstraintMetricsCsv(
    const char* status, const bool match_full_submap,
    const SubmapId& submap_id, const NodeId& node_id,
    const TrajectoryNode::Data* const constant_data, const float score,
    const double min_score, const transform::Rigid2d& initial_pose,
    const transform::Rigid2d& fast_pose, const transform::Rigid2d& ceres_pose,
    const transform::Rigid2d& constraint_transform, const double weight_scale,
    const double translation_weight, const double rotation_weight,
    const scan_matching::FastCorrelativeScanMatcher2D::ScoreDistributionSummary&
        score_summary,
    const bool ambiguous_downweighted) {
  std::ofstream* const csv = GetConstraintMetricsCsv();
  if (csv == nullptr) {
    return;
  }
  const transform::Rigid2d initial_to_final = initial_pose.inverse() * ceres_pose;
  std::lock_guard<std::mutex> lock(*GetConstraintMetricsCsvMutex());
  *csv << status << ',' << (match_full_submap ? 1 : 0) << ','
       << submap_id.trajectory_id << ',' << submap_id.submap_index << ','
       << node_id.trajectory_id << ',' << node_id.node_index << ','
       << constant_data->filtered_gravity_aligned_point_cloud.size() << ','
       << (constant_data->is_outlier ? 1 : 0) << ',' << score << ','
       << min_score << ',' << initial_pose.translation().x() << ','
       << initial_pose.translation().y() << ','
       << initial_pose.rotation().angle() << ','
       << fast_pose.translation().x() << ',' << fast_pose.translation().y()
       << ',' << fast_pose.rotation().angle() << ','
       << ceres_pose.translation().x() << ',' << ceres_pose.translation().y()
       << ',' << ceres_pose.rotation().angle() << ','
       << constraint_transform.translation().x() << ','
       << constraint_transform.translation().y() << ','
       << constraint_transform.rotation().angle() << ','
       << initial_to_final.translation().norm() << ','
       << std::abs(initial_to_final.normalized_angle()) << ','
       << score_summary.candidate_count << ',' << score_summary.top1_score
       << ',' << score_summary.top2_score << ','
       << (score_summary.top1_score - score_summary.top2_score) << ','
       << score_summary.near_top_count_0p02 << ','
       << (ambiguous_downweighted ? 1 : 0) << ','
       << weight_scale << ',' << translation_weight << ',' << rotation_weight
       << '\n';
  csv->flush();
}

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(absl::make_unique<common::Task>()),
      when_done_task_(absl::make_unique<common::Task>()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

ConstraintBuilder2D::~ConstraintBuilder2D() {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }

  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                      constant_data,
                      options_.global_localization_min_score(),
                      initial_relative_pose, *scan_matcher, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const double global_localization_min_score) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                      constant_data, global_localization_min_score,
                      transform::Rigid2d::Identity(), *scan_matcher,
                      constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::NotifyEndOfNode() {
  absl::MutexLock locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = absl::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = absl::make_unique<common::Task>();
}

const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  CHECK(grid);
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
  submap_scan_matcher.grid = grid;
  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
  auto scan_matcher_task = absl::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem(
      [&submap_scan_matcher, &scan_matcher_options]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            absl::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const double global_localization_min_score,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();
  scan_matching::FastCorrelativeScanMatcher2D::ScoreDistributionSummary
      score_summary;

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            global_localization_min_score, &score, &pose_estimate,
            &score_summary)) {
      CHECK_GT(score, global_localization_min_score);
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
    } else {
      MaybeWriteConstraintMetricsCsv(
          "fast_rejected", match_full_submap, submap_id, node_id,
          constant_data, score, global_localization_min_score, initial_pose,
          pose_estimate, pose_estimate, transform::Rigid2d::Identity(), 0.,
          0., 0., score_summary, false);
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score(), &score, &pose_estimate, &score_summary)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
    } else {
      MaybeWriteConstraintMetricsCsv(
          "fast_rejected", match_full_submap, submap_id, node_id,
          constant_data, score, options_.min_score(), initial_pose,
          pose_estimate, pose_estimate, transform::Rigid2d::Identity(), 0.,
          0., 0., score_summary, false);
      return;
    }
  }
  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  const transform::Rigid2d fast_pose_estimate = pose_estimate;
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  // Outlier nodes are still inserted into submaps (to prevent active submap
  // sparsity), but their constraints are down-weighted so the pose graph
  // optimizer does not trust their imprecise pose estimates strongly.
  double weight_scale = constant_data->is_outlier ? 0.1 : 1.0;
  const transform::Rigid2d initial_to_final =
      initial_pose.inverse() * pose_estimate;
  const double top1_top2_margin =
      score_summary.candidate_count >= 2
          ? score_summary.top1_score - score_summary.top2_score
          : std::numeric_limits<double>::infinity();
  const bool ambiguous_large_constraint =
      EnvBool("POSE_GRAPH_AMBIGUOUS_CONSTRAINT_DOWNWEIGHT", false) &&
      initial_to_final.translation().norm() >
          EnvDouble("POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MIN_TRANSLATION", 0.30) &&
      top1_top2_margin <
          EnvDouble("POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MAX_SCORE_MARGIN",
                    0.001) &&
      score_summary.near_top_count_0p02 >=
          EnvDouble("POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MIN_NEAR_TOP_COUNT", 20.);
  if (ambiguous_large_constraint) {
    weight_scale *=
        EnvDouble("POSE_GRAPH_AMBIGUOUS_CONSTRAINT_WEIGHT_SCALE", 0.2);
  }
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight() * weight_scale,
                                    options_.loop_closure_rotation_weight() * weight_scale},
                                   Constraint::INTER_SUBMAP});
  MaybeWriteConstraintMetricsCsv(
      "accepted", match_full_submap, submap_id, node_id, constant_data, score,
      match_full_submap ? global_localization_min_score : options_.min_score(),
      initial_pose, fast_pose_estimate, pose_estimate, constraint_transform,
      weight_scale, options_.loop_closure_translation_weight() * weight_scale,
      options_.loop_closure_rotation_weight() * weight_scale, score_summary,
      ambiguous_large_constraint);

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  absl::MutexLock locker(&mutex_);
  return num_finished_nodes_;
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
  auto* num_matchers = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_num_submap_scan_matchers",
      "Current number of constructed submap scan matchers");
  kNumSubmapScanMatchersMetric = num_matchers->Add({});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
