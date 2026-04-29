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

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class DummyFileResolver : public common::FileResolver {
 public:
  std::string GetFileContentOrDie(const std::string& unused_basename) override {
    LOG(FATAL) << "Not implemented";
    return std::string();
  }

  std::string GetFullPathOrDie(const std::string& unused_basename) override {
    LOG(FATAL) << "Not implemented";
    return std::string();
  }
};

std::unique_ptr<common::LuaParameterDictionary> MakeDictionary(
    const std::string& code) {
  return absl::make_unique<common::LuaParameterDictionary>(
      code, absl::make_unique<DummyFileResolver>());
}

proto::FrozenSubmapScanMatcherOptions2D MakeOptions(const std::string& extra) {
  auto parameter_dictionary = MakeDictionary(
      "return {"
      "enabled = true, "
      "apply_mode = \"FULL_PIPELINE\", "
      "search_radius = 1.0, "
      "max_submaps_to_match = 5, "
      "use_realtime_correlative_scan_matching = true, "
      "use_ceres_scan_matching = true, "
      "min_realtime_correlative_score = 0.5, "
      "min_score_margin = 0.0, "
      "score_variance_top_k = 2, "
      "min_score_variance = 0.0, "
      "max_translation_correction = 1.0, "
      "max_rotation_correction = 0.5, "
      "real_time_correlative_scan_matcher = {"
      "  linear_search_window = 0.2, "
      "  angular_search_window = 0.2, "
      "  translation_delta_cost_weight = 0.0, "
      "  rotation_delta_cost_weight = 0.0, "
      "}, "
      "ceres_scan_matcher = {"
      "  occupied_space_weight = 1.0, "
      "  translation_weight = 0.1, "
      "  rotation_weight = 1.0, "
      "  ceres_solver_options = {"
      "    use_nonmonotonic_steps = false, "
      "    max_num_iterations = 20, "
      "    num_threads = 1, "
      "  }, "
      "}, " +
      extra + "}");
  return CreateFrozenSubmapScanMatcherOptions2D(parameter_dictionary.get());
}

class FrozenSubmapScanMatcher2DTest : public ::testing::Test {
 protected:
  FrozenSubmapScanMatcher2DTest() {
    point_cloud_.push_back({Eigen::Vector3f{0.025f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.025f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.075f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.175f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.125f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.075f, 0.f}});
    point_cloud_.push_back({Eigen::Vector3f{-0.125f, 0.025f, 0.f}});
  }

  std::shared_ptr<const Submap2D> MakeFinishedSubmap() {
    auto submap = std::make_shared<Submap2D>(
        Eigen::Vector2f::Zero(),
        absl::make_unique<ProbabilityGrid>(
            MapLimits(0.05, Eigen::Vector2d(0.05, 0.25), CellLimits(6, 6)),
            &conversion_tables_),
        &conversion_tables_);
    auto parameter_dictionary = MakeDictionary(
        "return { "
        "insert_free_space = true, "
        "hit_probability = 0.7, "
        "miss_probability = 0.4, "
        "}");
    ProbabilityGridRangeDataInserter2D range_data_inserter(
        CreateProbabilityGridRangeDataInserterOptions2D(
            parameter_dictionary.get()));
    submap->InsertRangeData(
        sensor::RangeData{Eigen::Vector3f::Zero(), point_cloud_, {}},
        &range_data_inserter);
    submap->Finish();
    return submap;
  }

  FrozenSubmapQueryResult2D MakeQuery(
      const std::vector<FrozenSubmapSnapshot2D>& submaps) {
    FrozenSubmapQueryResult2D query;
    query.local_to_map = transform::Rigid2d::Identity();
    query.submaps = submaps;
    return query;
  }

  ValueConversionTables conversion_tables_;
  sensor::PointCloud point_cloud_;
};

TEST_F(FrozenSubmapScanMatcher2DTest, AcceptsHighConfidenceMatch) {
  FrozenSubmapScanMatcher2D matcher(MakeOptions(""));
  const auto submap = MakeFinishedSubmap();
  const auto result = matcher.Match(
      MakeQuery({FrozenSubmapSnapshot2D{
          SubmapId{0, 0}, submap, transform::Rigid2d::Identity(),
          transform::Rigid2d::Identity()}}),
      transform::Rigid2d::Translation({0.05, 0.0}), point_cloud_);
  EXPECT_TRUE(result.attempted);
  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(0., result.filtered_tracking_to_local.translation().x(), 1e-2);
  EXPECT_NEAR(0., result.filtered_tracking_to_local.translation().y(), 1e-2);
}

TEST_F(FrozenSubmapScanMatcher2DTest, RejectsAmbiguousBestScoreMargin) {
  FrozenSubmapScanMatcher2D matcher(MakeOptions("min_score_margin = 0.01, "));
  const auto first_submap = MakeFinishedSubmap();
  const auto second_submap = MakeFinishedSubmap();
  const auto result = matcher.Match(
      MakeQuery({FrozenSubmapSnapshot2D{
                     SubmapId{0, 0}, first_submap, transform::Rigid2d::Identity(),
                     transform::Rigid2d::Identity()},
                 FrozenSubmapSnapshot2D{
                     SubmapId{0, 1}, second_submap,
                     transform::Rigid2d::Identity(),
                     transform::Rigid2d::Identity()}}),
      transform::Rigid2d::Translation({0.05, 0.0}), point_cloud_);
  EXPECT_TRUE(result.attempted);
  EXPECT_FALSE(result.accepted);
  EXPECT_NEAR(result.best_score, result.second_best_score, 1e-6);
}

TEST_F(FrozenSubmapScanMatcher2DTest, RejectsLowTopKVariance) {
  FrozenSubmapScanMatcher2D matcher(MakeOptions(
      "min_score_margin = 0.0, min_score_variance = 1e-4, "));
  const auto first_submap = MakeFinishedSubmap();
  const auto second_submap = MakeFinishedSubmap();
  const auto result = matcher.Match(
      MakeQuery({FrozenSubmapSnapshot2D{
                     SubmapId{0, 0}, first_submap, transform::Rigid2d::Identity(),
                     transform::Rigid2d::Identity()},
                 FrozenSubmapSnapshot2D{
                     SubmapId{0, 1}, second_submap,
                     transform::Rigid2d::Identity(),
                     transform::Rigid2d::Identity()}}),
      transform::Rigid2d::Translation({0.05, 0.0}), point_cloud_);
  EXPECT_TRUE(result.attempted);
  EXPECT_FALSE(result.accepted);
  EXPECT_NEAR(0., result.top_k_score_variance, 1e-9);
}

TEST_F(FrozenSubmapScanMatcher2DTest, RejectsOversizedTranslationCorrection) {
  FrozenSubmapScanMatcher2D matcher(
      MakeOptions("max_translation_correction = 0.01, "));
  const auto submap = MakeFinishedSubmap();
  const auto result = matcher.Match(
      MakeQuery({FrozenSubmapSnapshot2D{
          SubmapId{0, 0}, submap, transform::Rigid2d::Identity(),
          transform::Rigid2d::Identity()}}),
      transform::Rigid2d::Translation({0.05, 0.0}), point_cloud_);
  EXPECT_TRUE(result.attempted);
  EXPECT_FALSE(result.accepted);
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
