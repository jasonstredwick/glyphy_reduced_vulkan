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
#include <ranges>
#include <span>
#include <vector>
#include <utility>

#include "glyphy/glm.hpp"

#include "glyphy/glyph/glyphy_reduced/arc.hpp"
#include "glyphy/glyph/glyphy_reduced/endpoint.hpp"
#include "glyphy/math.hpp"


namespace glyphy::outline {


constexpr const double EPSILON = 1e-9; // integer portion ranges [0, 2^14]
bool IsZero(const double d) { return glm::abs(d) < EPSILON; }
bool IsZero(const glm::dvec2& v) { return IsZero(v.x) && IsZero(v.y); }
bool IsEqual(const auto& lhs, const auto& rhs) { return IsZero(lhs - rhs); }


#if 0
void UpdateWindingFromEvenOdd(std::vector<Endpoint>& endpoints, bool inverse);

void UpdateWindingFromEvenOdd(std::vector<Endpoint>& endpoints, bool inverse) {
}




#else

bool EvenOdd(const std::vector<Endpoint>& endpoints, size_t subset_start, size_t subset_total);
bool ProcessContour(std::vector<Endpoint>& endpoints, size_t subset_start, size_t subset_total, bool inverse);
void Reverse(std::span<Endpoint>& endpoints);
bool Winding(const std::span<const Endpoint>& endpoints);
bool WindingFromEvenOdd(std::vector<Endpoint>& endpoints, bool inverse);


double Categorize(double v, double ref) {
    return v < ref - EPSILON ? -1.0 : v > ref + EPSILON ? 1.0 : 0.0;
}


bool EvenOdd(const std::vector<Endpoint>& endpoints, size_t subset_start, size_t subset_total) {
    /***
      * Algorithm:
      *
      * - For a point on the contour, draw a halfline in a direction
      *   (eg. decreasing x) to infinity,
      * - Count how many times it crosses all other contours,
      * - Pay special attention to points falling exactly on the halfline,
      *   specifically, they count as +.5 or -.5, depending the direction
      *   of crossing.
      *
      * All this counting is extremely tricky:
      *
      * - Floating point equality cannot be relied on here,
      * - Lots of arc analysis needed,
      * - Without having a point that we know falls /inside/ the contour,
      *   there are legitimate cases that we simply cannot handle using
      *   this algorithm.  For example, imagine the following glyph shape:
      *
      *         +---------+
      *         | +-----+ |
      *         |  \   /  |
      *         |   \ /   |
      *         +----o----+
      *
      *   If the glyph is defined as two outlines, and when analysing the
      *   inner outline we happen to pick the point denoted by 'o' for
      *   analysis, there simply is no way to differentiate this case from
      *   the following case:
      *
      *         +---------+
      *         |         |
      *         |         |
      *         |         |
      *         +----o----+
      *             / \
      *            /   \
      *           +-----+
      *
      *   However, in one, the triangle should be filled in, and in the other
      *   filled out.
      *
      *   One way to work around this may be to do the analysis for all endpoints
      *   on the outline and take majority.  But even that can fail in more
      *   extreme yet legitimate cases, such as this one:
      *
      *           +--+--+
      *           | / \ |
      *           |/   \|
      *           +     +
      *           |\   /|
      *           | \ / |
      *           +--o--+
      *
      *   The only correct algorithm I can think of requires a point that falls
      *   fully inside the outline.  While we can try finding such a point (not
      *   dissimilar to the winding algorithm), it's beyond what I'm willing to
      *   implement right now.
      */

    const glm::dvec2 ps0 = endpoints[subset_start].p;

    double count = 0;
    glm::dvec2 p0{0, 0};
    // TODO: convert to take + drop + join views and iterator over endpoints
    for (size_t index=0; index < endpoints.size(); ++index) {
        const Endpoint& endpoint = endpoints[index];
        if (glm::isinf(endpoint.d)) { p0 = endpoint.p; continue; }
        Arc arc{.p0=p0, .p1=endpoint.p, .d=endpoint.d};
        p0 = endpoint.p;

        // Skip our own contour
        if (index >= subset_start && index < subset_start + subset_total) { continue; }

        // End-point y's compared to the ref point; lt, eq, or gt
        double s0 = Categorize(arc.p0.y, ps0.y);
        double s1 = Categorize(arc.p1.y, ps0.y);

        if (IsZero(arc.d)) { // Line
            if (!s0 || !s1) { //IsZero(s0) || IsZero(s1)) {
                // Add +.5 / -.5 for each endpoint on the halfline, depending on crossing direction.
                std::array<glm::dvec2, 2> tangents = Tangents(arc);
                if (!s0 && arc.p0.x < ps0.x + EPSILON) { count += 0.5 * Categorize(tangents[0].y, 0); }
                if (!s1 && arc.p1.x < ps0.x + EPSILON) { count += 0.5 * Categorize(tangents[1].y, 0); }
                continue;
            }

            if (s0 == s1) { continue; } //IsZero(s0 - s1)) { continue; } // Segment fully above or below the halfline

            // Find x pos that the line segment would intersect the half-line.
            double x = arc.p0.x + (arc.p1.x - arc.p0.x) * ((ps0.y - arc.p0.y) / (arc.p1.y - arc.p0.y));
            if (x >= ps0.x - EPSILON) { continue; } // Does not intersect halfline

            count++; // Add one for full crossing
            continue;
        } else { // Arc
            if (!s0 || !s1) {//IsZero(s0) || IsZero(s1)) {
                // Add +.5 / -.5 for each endpoint on the halfline, depending on crossing direction.
                std::array<glm::dvec2, 2> tangents = Tangents(arc);

                // Arc-specific logic: If the tangent has dy==0, use the other endpoint's y value to decide which way
                // the arc will be heading.
                if (IsZero(tangents[0].y)) { tangents[0].y = +Categorize(arc.p1.y, ps0.y); }
                if (IsZero(tangents[1].y)) { tangents[1].y = -Categorize(arc.p0.y, ps0.y); }
                if (s0 != 0.0 && arc.p0.x < ps0.x + EPSILON) { count += 0.5 * Categorize(tangents[0].y, 0); }
                if (s1 != 0.0 && arc.p1.x < ps0.x + EPSILON) { count += 0.5 * Categorize(tangents[1].y, 0); }
                //if (IsZero(s0) && arc.p0.x < ps0.x + EPSILON) { count += 0.5 * Categorize(tangents[0].y, 0); }
                //if (IsZero(s1) && arc.p1.x < ps0.x + EPSILON) { count += 0.5 * Categorize(tangents[1].y, 0); }
            }

            glm::dvec2 center = Center(arc);
            double radius = Radius(arc);
            if (center.x - radius >= ps0.x) { continue; } // No chance
            // Solve for arc crossing line with y = p.y
            double dy = ps0.y - center.y;
            double x2 = radius * radius - dy * dy;
            if (x2 <= EPSILON) { continue; } // Negative delta, no crossing
            double dx = glm::sqrt(x2);

            /* There's two candidate points on the arc with the same y as the ref point. */
            const std::array<glm::dvec2, 2> candidate_points{glm::dvec2{center.x - dx, ps0.y},
                                                             glm::dvec2{center.x + dx, ps0.y}};
            for (const auto& candidate_point : candidate_points) {
                // Make sure we don't double-count endpoints that fall on the halfline as we already accounted for
                // those above.
                if (!IsEqual(candidate_point, arc.p0) &&
                    !IsEqual(candidate_point, arc.p1) &&
                    candidate_point.x < ps0.x - EPSILON &&
                    WedgeContainsPoint(arc, candidate_point))
                {
                    count += 1; // Add one for full crossing
                }
            }
        }
    }

    return !(static_cast<int64_t>(glm::floor(count)) & 1);
}


bool ProcessContour(std::vector<Endpoint>& endpoints, size_t subset_start, size_t subset_total, bool inverse) {
    /***
     * Algorithm:
     *
     * - Find the Winding direction and even-odd number,
     * - If the two disagree, reverse the contour, inplace.
     */
    if (!subset_total) { return false; }
    assert(subset_total >= 3); // Don't expect this; need at least three points

    std::span<Endpoint> subset{endpoints.data() + subset_start, subset_total};
    assert(subset.front().p == subset.back().p); // Don't expect this; need a closed contour

    if (inverse ^ Winding(subset) ^ EvenOdd(endpoints, subset_start, subset_total)) {
        Reverse(subset);
        return true;
    }

    return false;
}


void Reverse(std::span<Endpoint>& endpoints) {
    if (endpoints.empty()) { return; }

    // shift d's first  ... why?
    double d0 = endpoints[0].d;
    for (size_t i=0; i<endpoints.size(); ++i) {
        double d1 = endpoints[i + 1].d;
        endpoints[i].d = glm::isinf(d1) ? std::numeric_limits<double>::infinity() : -d1;
    }
    endpoints.back().d = d0;
    std::ranges::reverse(endpoints);
}


bool Winding(const std::span<const Endpoint>& endpoints) {
    /*
     * Algorithm:
     *
     * - Approximate arcs with triangles passing through the mid- and end-points,
     * - Calculate the area of the contour,
     * - Return sign.
     */
    double area = 0;
    for (unsigned int i = 1; i < endpoints.size(); i++) {
        const glm::dvec2& p0 = endpoints[i - 1].p;
        const glm::dvec2& p1 = endpoints[i].p;
        double d = endpoints[i].d;
        assert(!glm::isinf(d));
        area += Cross(p0, p1);
        area -= 0.5 * d * glm::dot(p1 - p0, p1 - p0);
    }
    return area < 0;
}


// Returns true if outline was modified
bool WindingFromEvenOdd(std::vector<Endpoint>& endpoints) {
    /*
     * Algorithm:
     *
     * - Process one contour at a time.
     * - No short circuit on ret == true, as ProcessContour call chain also modifies endpoints.
     */

    if (endpoints.empty()) { return false; }

    bool inverse = false;
    bool ret = false;
    size_t start = 0; // starting with the first endpoint, include endpoints until you reach one with d == infinity
    // [infinite, (finite)*]
    // according to ProcessContour: [infinite, (finite)(2+)]
    for (size_t i = 1; i < endpoints.size(); i++) {
        const Endpoint& endpoint = endpoints[i];
        if (glm::isinf(endpoint.d)) {
            if (ProcessContour(endpoints, start, i - start, inverse)) { ret = true; }
            start = i;
        }
    }
    return ret || ProcessContour(endpoints, start, endpoints.size() - start, inverse);
}
#endif


}
