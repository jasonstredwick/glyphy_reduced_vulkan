/*
 * Copyright 2012 Google, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Google Author(s): Behdad Esfahbod
 */


#pragma once


#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cmath>
#include <limits>
#include <print>
#include <ranges>
#include <span>
#include <vector>
#include <utility>

#include "glyphy/glm.hpp"

#include "glyphy/extents.hpp"
#include "glyphy/glyph/glyphy_reduced/arc.hpp"
#include "glyphy/glyph/glyphy_reduced/endpoint.hpp"
#include "glyphy/math.hpp"


namespace glyphy {


using Path_t = std::span<Endpoint>;


constexpr const double EPSILON_f = static_cast<double>(std::numeric_limits<float>::epsilon());
constexpr const double EPSILON = 1e-9; // integer portion ranges [0, 2^14]
constexpr bool IsZero(const double d) { return glm::abs(d) < EPSILON_f; }
constexpr bool IsZero(const glm::dvec2& v) { return IsZero(v.x) && IsZero(v.y); }
constexpr bool IsEqual(const auto& lhs, const auto& rhs) { return IsZero(lhs - rhs); }
constexpr bool IsLessThanZero(const double d) { return d <= -EPSILON_f; }
constexpr bool IsLessThanEqualToZero(const double d) { return d < EPSILON_f; }
constexpr bool IsGreaterThanZero(const double d) { return d >= EPSILON_f; }
constexpr bool IsGreaterThanEqualToZero(const double d) { return d > -EPSILON_f; }


void AdjustWinding(std::span<Endpoint> endpoints);
std::vector<Path_t> ConvertEndpointsToPaths(std::span<Endpoint> endpoints);
std::vector<size_t> FindPaths(std::span<const Endpoint> endpoints);
bool PathWindingNeedsReverse(const std::vector<Path_t>& paths,
                             const size_t test_path_index,
                             const size_t test_endpoint_index,
                             const double max_len);
void ReverseWinding(Path_t path);


void AdjustWinding(std::span<Endpoint> endpoints) {
    auto paths = ConvertEndpointsToPaths(endpoints);

    Extents extents{};
    std::ranges::for_each(endpoints, [&extents](auto& e) { extents.Add(e.p); });
    const double max_len = 2.0 * glm::distance(glm::dvec2{extents.min_x, extents.min_y},
                                               glm::dvec2{extents.max_x, extents.max_y});

    size_t need_update = 0;
    for (auto [path_index, path] : paths | std::views::enumerate) {
        const size_t test_endpoint_index = 0;
        if (PathWindingNeedsReverse(paths, path_index, test_endpoint_index, max_len)) {
            need_update++;
            ReverseWinding(path);
        }
    }
}


std::vector<Path_t> ConvertEndpointsToPaths(std::span<Endpoint> endpoints) {
    std::vector<Path_t> paths{};
    auto start_it = endpoints.begin();
    while (start_it != endpoints.end()) {
        auto it = std::ranges::find_if(std::next(start_it), endpoints.end(), [](auto& e) {
            return !std::isfinite(e.d);
        });
        paths.push_back({start_it, it});
        start_it = it;
    }
    return paths;
}


std::vector<size_t> FindPaths(std::span<const Endpoint> endpoints) {
    std::vector<size_t> paths_start_index{};
    auto Filter_f = [](const auto& tup) { return !std::isfinite(std::get<1>(tup).d); };
    std::ranges::transform(endpoints | std::views::enumerate | std::views::filter(Filter_f),
                           std::back_inserter(paths_start_index),
                           [](const auto& tup) { return static_cast<size_t>(std::get<0>(tup)); });
    paths_start_index.push_back(endpoints.size());
    return paths_start_index;
}


bool PathWindingNeedsReverse(const std::vector<Path_t>& paths,
                             const size_t test_path_index,
                             const size_t test_endpoint_index,
                             const double max_len) {
    // change to .at once support for c++26 std::span::at is available
    const glm::dvec2 test_p0 = paths.at(test_path_index)[test_endpoint_index].p;
    const glm::dvec2 test_p1 = paths.at(test_path_index)[test_endpoint_index + 1].p;
    const glm::dvec2 p0 = Midpoint(test_p0, test_p1);
    const glm::dvec2 uv01 = glm::normalize(Ortho(test_p1 - test_p0)); // rotate counter-clockwise; i.e. left
    const glm::dvec2 p1 = p0 + (uv01 * max_len);

    double hits = 0.0;
    // can eventually replace nested for loops with std::generator
    for (auto [path_index, path] : paths | std::views::enumerate) {
        for (auto [endpoint_index, endpoint_pair] : path | std::views::pairwise | std::views::enumerate) {
            if (path_index == test_path_index && endpoint_index == test_endpoint_index) { continue; }

            const glm::dvec2 p2 = std::get<0>(endpoint_pair).p;
            const glm::dvec2 p3 = std::get<1>(endpoint_pair).p;
            const glm::dvec2 uv23 = glm::normalize(p3 - p2);

            // Ignore parallel segments; let the endpoints from connected segments be counted instead.
            if (IsZero(1.0 - glm::abs(glm::dot(uv01, uv23)))) { continue; }

            // Check segment against test segment
            const glm::dvec2 uv02 = glm::normalize(p2 - p0);
            const glm::dvec2 uv03 = glm::normalize(p3 - p0);
            const double cross0 = Cross(uv01, uv02);
            const double cross1 = Cross(uv01, uv03);
            if ((IsLessThanZero(cross0) && IsLessThanZero(cross1)) ||        // both to the right
                (IsGreaterThanZero(cross0) && IsGreaterThanZero(cross1))) {  // both to the left
                continue;
            }

            if (IsZero(cross0)) { // p2 is on the test segment if in front of p0
                if (IsGreaterThanEqualToZero(glm::dot(uv01, uv02))) { hits += 0.5; }
                continue;
            } else if (IsZero(cross1)) { // p3 is on the test segment if in front of p0
                if (IsGreaterThanEqualToZero(glm::dot(uv01, uv03))) { hits += 0.5; }
                continue;
            }

            // Check test segment against segment
            const glm::dvec2 uv20 = -uv02;
            const glm::dvec2 uv21 = glm::normalize(p1 - p2);
            const double cross2 = Cross(uv23, uv20);
            const double cross3 = Cross(uv23, uv21);
            if ((IsLessThanZero(cross2) && IsLessThanZero(cross3)) ||        // both to the right
                (IsGreaterThanZero(cross2) && IsGreaterThanZero(cross3))) {  // both to the left
                continue;
            }

            hits += 1.0;
        }
    }

    // ensure there is no dangling 0.5 hit (should always be a whole number because of closed paths)
    assert(static_cast<int64_t>(hits - std::floor(hits) + 0.5) == 0);

    // Want the cross product between test segment and test point to return negative distances when the point is in
    // the interior.  Therefore, the path should progress to clockwise inorder to ensure exterior points are to the
    // left of the test segment; i.e. have a positive distance.
    return static_cast<int64_t>(hits) % 2 == 1; // odd; points inward;
    //return static_cast<int64_t>(hits) % 2 == 0; // even; points outward
}


void ReverseWinding(Path_t path) {
    // This process is required because of the association of d with each pair; i.e. A->B d is associated with B.
    // After this process, the d value needs to be shifted to the other endpoint; i.e. A.  The value of d is negated
    // because the direction of rotation between A and B inverts and the sign is used to figure out the outward
    // direction from the center of the arc circle.
    auto Transform_f = [](const auto& e0, const auto& e1) { return Endpoint{.p=e0.p, .d=(e1.d == 0) ? 0 : -e1.d}; };
    std::ranges::transform(path | std::views::pairwise_transform(Transform_f), path.begin(), std::identity{});
    path.back().d = std::numeric_limits<double>::infinity();
    std::ranges::reverse(path);
}


}
