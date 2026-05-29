/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"

#include <algorithm>
#include <cmath>

namespace cartographer {
namespace mapping {

Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b) {
  const Eigen::Vector3d v0 = a.normalized();
  const Eigen::Vector3d v1 = b.normalized();
  const double c = std::max(-1., std::min(1., v1.dot(v0)));

  if (c < -1. + Eigen::NumTraits<double>::dummy_precision()) {
    const Eigen::Vector3d axis = v0.unitOrthogonal();
    return Eigen::Quaterniond(0., axis.x(), axis.y(), axis.z());
  }

  const Eigen::Vector3d axis = v0.cross(v1);
  const double s = std::sqrt((1. + c) * 2.);
  const double inv_s = 1. / s;
  return Eigen::Quaterniond(s * 0.5, axis.x() * inv_s, axis.y() * inv_s,
                            axis.z() * inv_s)
      .normalized();
}

}  // namespace mapping
}  // namespace cartographer
