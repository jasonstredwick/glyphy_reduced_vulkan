#pragma once


#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <numeric>
#include <ranges>
#include <span>
#include <type_traits>
#include <vector>

#include "glyphy/glm.hpp"

#include "glyphy/glyph/glyphy/arc.hpp"
#include "glyphy/bezier.hpp"
#include "glyphy/math.hpp"


/***
 * Variant from Glyphy
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-arc-bezier.hh
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-arc.cc
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-arcs-bezier.hh
 */


namespace glyphy {


/***
 * Forward Declarations
 */


Bezier ApproximateArcWithBezier(const Arc& arc, double& error);


/***
 * Concepts
 */


template <typename T>
concept DeviationApproximator_c = requires(double a, double b) {
    { T::ApproximateDeviation(a, b) } -> std::same_as<double>;
};


template <typename T>
concept ErrorApproximator_c = requires(const Bezier &a, const Arc &b) {
    { T::ApproximateError(a, b) } -> std::same_as<double>;
};


/***
 * Implementations
 */


// Should range comparisons be used here?  Or is that why it is called Exact?
// Why are the first two values 0 and 1 evaluated?  According to the formula they both evaluate to 0;
//        (3 * t   * (1-t) * (d0 * (1 - t) + d1 * t))
// [t=0]   3 * 0   * 1     * ... = 0
// [t=1]   3 * 1   * 0     * ... = 0
// [t=0.5] 3 * 0.5 * 0.5   * (d0 * 0.5     + d1 * 0.5) =  0.75 * 0.5*(d0 + d1)
//
// [d0 == d1, t=0.5] -> 0.75 * 0.5 * (d0 + d1) = 0.75 * 0.5 * 2 * d0 = 0.75 * d0
struct MaxDeviationApproximatorExact {
    // Returns 3 max(abs(d₀ t (1-t)² + d₁ t² (1-t)) for 0≤t≤1.
    static double ApproximateDeviation(double d0, double d1) {
        double candidates[4] = {0, 1};
        unsigned int num_candidates = 2;
        if (d0 == d1) { candidates[num_candidates++] = 0.5; }
        else {
            double delta = d0*d0 - d0*d1 + d1*d1;
            double t2 = 1.0 / (3.0 * (d0 - d1));
            double t0 = (2.0 * d0 - d1) * t2;
            if (delta == 0) { candidates[num_candidates++] = t0; }
            else if (delta > 0) {
                // This code can be optimized to avoid the sqrt if the solution is not feasible (ie. lies outside
                // (0,1)).  I have implemented that in cairo-spline.c:_cairo_spline_bound().  Can be reused here.
                double t1 = glm::sqrt(delta) * t2;
                candidates[num_candidates++] = t0 - t1;
                candidates[num_candidates++] = t0 + t1;
            }
        }

        double e = 0;
        for (unsigned int i = 0; i < num_candidates; i++) {
            double t = candidates[i];
            if (t < 0.0 || t > 1.0) { continue; }
            double ee = glm::abs(3.0 * t * (1.0 - t) * (d0 * (1.0 - t) + d1 * t));
            e = glm::max(e, ee);
        }

        return e;
    }
};


template <DeviationApproximator_c MaxDeviationApproximator>
struct ErrorApproximatorBehdad {
    static double ApproximateError(const Bezier &b0, const Arc &arc) {
        assert(b0.p0 == arc.p0);
        assert(b0.p3 == arc.p1);
        double error_arc = 0;
        Bezier b1 = ApproximateArcWithBezier(arc, error_arc);
        assert(b0.p0 == b1.p0);
        assert(b0.p3 == b1.p3);

        glm::dvec2 v0 = b1.p1 - b0.p1;
        glm::dvec2 v1 = b1.p2 - b0.p2;
        glm::dvec2 b = glm::normalize(b0.p3 - b0.p0);
        v0 = Rebase(v0, b);
        v1 = Rebase(v1, b);
        glm::dvec2 v(MaxDeviationApproximator::ApproximateDeviation(v0.x, v1.x),
                     MaxDeviationApproximator::ApproximateDeviation(v0.y, v1.y));

        /* Edge cases: If d*d is too close too large default to a weak bound. */
        if (arc.d * arc.d > 1.0 - 1e-4) { return error_arc + glm::length(v); }

        /* If the wedge doesn't contain control points, default to weak bound. */
        if (!WedgeContainsPoint(arc, b0.p1) || !WedgeContainsPoint(arc, b0.p2)) { return error_arc + glm::length(v); }

        /* If straight line, return the max ortho deviation. */
        if (glm::abs(arc.d) < 1e-6) { return error_arc + v.y; }

        /* We made sure that abs(arc.d) < 1 */
        double tan_half_alpha = glm::abs(tan2atan(arc.d));
        double tan_v = v.x / v.y;
        if (glm::abs(tan_v) <= tan_half_alpha) { return error_arc + glm::length(v); }
        double c2 = glm::length(arc.p1 - arc.p0) * 0.5;
        double error_b = glm::length(glm::dvec2(c2 + v.x, c2 / tan_half_alpha + v.y)) - Radius(arc);
        assert(error_b >= 0);
        return error_arc + error_b;
    }
};


template <ErrorApproximator_c ErrorApproximator>
struct ApproximatorMidpointSimple {
    static const Arc ApproximateBezierWithArc(const Bezier &bezier, double& error) {
        // Should Midpoint be LerpPoint(bezier, 0.5)? Otherwise the point may not be on the curve.
        Arc arc = CreateArc(bezier.p0, bezier.p3, Midpoint(bezier), false);
        error = ErrorApproximator::ApproximateError(bezier, arc);
        return arc;
    }
};


template <ErrorApproximator_c ErrorApproximator>
struct ApproximatorMidpointTwoPart {
    static const Arc ApproximateBezierWithArc(const Bezier &bezier, double& error, double mid_t=0.5) {
        std::array<Bezier, 2> split_beziers = Split(bezier, mid_t);
        const Bezier& left_bezier = split_beziers[0];
        const Bezier& right_bezier = split_beziers[1];
        glm::dvec2 mid_point = right_bezier.p0;

        //Arc left_arc = CreateArc(bezier.p0, mid_point, bezier.p3, true);
        //Arc right_arc = CreateArc(mid_point, bezier.p3, bezier.p0, true);
        Arc left_arc = CreateArc(left_bezier.p0, left_bezier.p3, LerpPoint(left_bezier, 0.5), false);
        Arc right_arc = CreateArc(right_bezier.p0, right_bezier.p3, LerpPoint(right_bezier, 0.5), false);

        double e0 = ErrorApproximator::ApproximateError(left_bezier, left_arc);
        double e1 = ErrorApproximator::ApproximateError(right_bezier, right_arc);
        error = glm::max(e0, e1);

        return CreateArc(bezier.p0, bezier.p3, mid_point, false);
    }
};


template <class Approximator>
class ApproximatorSpringSystem {
    static void CalcArcs(const Bezier& bezier,
                         const std::span<double>& t,
                         const Approximator& appx,
                         std::vector<double>& error,
                         std::vector<Arc>& arcs,
                         double& max_error,
                         double& min_error) {
        arcs.clear();
        max_error = -std::numeric_limits<double>::infinity();
        min_error =  std::numeric_limits<double>::infinity();
        for (size_t i : std::views::iota(static_cast<size_t>(0), t.size() - 1)) {
            arcs.push_back(appx.ApproximateBezierWithArc(GetSegment(bezier, t[i], t[i + 1]), error[i]));
            max_error = glm::max(max_error, error[i]);
            min_error = glm::min(min_error, error[i]);
        }
    }

    static void Jiggle(const Bezier& b,
                       std::span<double>& t,
                       const Approximator& appx,
                       std::vector<double>& error,
                       std::vector<Arc>& arcs,
                       double& max_error,
                       double& min_error,
                       double tolerance,
                       size_t& n_jiggle) {
        double conditioner = tolerance * 0.01;
        size_t N = t.size() - 1;
        size_t max_jiggle = static_cast<size_t>(std::log2(N) + 1);
        for (size_t jiggle_count = 0; jiggle_count < max_jiggle; jiggle_count++) {
            // Recompute the segment lengths based on the error for each generated arc previously computed.
            double total = 0;
            for (size_t i = 0; i < N; i++) {
                double segment_length = t[i + 1] - t[i];
                double k_inv = segment_length * glm::pow(error[i] + conditioner, -0.3);
                total += k_inv;
                error[i] = k_inv;
            }

            t[0] = 0.0;
            for (size_t i : std::views::iota(static_cast<size_t>(1), t.size())) {
                t[i] = t[i - 1] + (error[i - 1] / total) + (2.0 * std::numeric_limits<double>::epsilon());
            }
            std::ranges::for_each(t, [max_t=t.back()](auto& v) { v /= max_t; });
            t[N] = 1.0; // Do this to get real 1.0, not .9999999999999998!

            for (size_t i = 0; i < N; i++) {
                double k_inv = error[i];
                // Need to ensure minimum segment length in order to avoid NaNs.
                double segment_length = std::ranges::max(k_inv / total, 2.0 * std::numeric_limits<double>::epsilon());
                t[i + 1] = t[i] + segment_length;
            }
            t[N] = 1.0; // Do this to get real 1.0, not .9999999999999998!

            // Calculate arcs with new segment lengths
            CalcArcs(b, t, appx, error, arcs, max_error, min_error);
            n_jiggle++;
            if (max_error < tolerance || (2 * min_error - max_error > tolerance)) {
                break;
            }
        }
    }

public:
    static std::vector<Arc> ApproximateBezierWithArcs(const Bezier &bezier,
                                                      const Approximator& appx,
                                                      double tolerance,
                                                      double& perror,
                                                      size_t max_segments=100) {
        // Handle fully-degenerate cases.
        glm::dvec2 v1(bezier.p1 - bezier.p0);
        glm::dvec2 v2(bezier.p2 - bezier.p0);
        glm::dvec2 v3(bezier.p3 - bezier.p0);
        if (Cross(v1, v2) == 0.0 && Cross(v2, v3) == 0.0) {
            // Curve has no area;  If endpoints are NOT the same, replace with a line segment.  Otherwise fully skip.
            perror = 0;
            if (bezier.p0 != bezier.p1) { return {Arc{bezier.p0, bezier.p3, 0}}; }
            return {};
        }

        std::vector<Arc> arcs{};
        arcs.reserve(max_segments);
        std::vector<double> error(max_segments, 0.0);
        std::vector<double> bezier_segment_positions(max_segments + 1); // [0..1]
        double max_error = -std::numeric_limits<double>::infinity();
        double min_error =  std::numeric_limits<double>::infinity();
        size_t n_jiggle = 0; // was used to count the total number of jiggles performed

        // Technically speaking we can bsearch for n.
        for (size_t N : std::views::iota(static_cast<size_t>(1), max_segments)) {
            std::span<double> t{bezier_segment_positions.data(), N + 1};

            // Subdivide curve into N equal sized segments.
            t[0] = 0.0;
            std::ranges::transform(
                std::views::iota(static_cast<int64_t>(1), static_cast<int64_t>(N)),
                std::next(t.begin()),
                [&N](auto i) -> double { return static_cast<double>(i) / static_cast<double>(N); });
            t[N] = 1.0; // Do this out of the loop to get real 1.0, not .9999999999999998!

            CalcArcs(bezier, t, appx, error, arcs, max_error, min_error);

            // Added; as it seems that if the original arcs error is good then why jiggle.
            if (max_error <= tolerance) { break; }

            // Jiggle happens at most once per CalcArcs. Provides small variation in segment size based on the
            // error for each segment.
            for (size_t i = 0; i < N; i++) {
                if (error[i] <= tolerance) {
                    Jiggle(bezier, t, appx, error, arcs, max_error, min_error, tolerance, n_jiggle);
                    break;
                }
            }

            if (max_error <= tolerance) { break; }
        }
        perror = max_error;
        return arcs;
    }
};


using MaxDeviationApproximatorDefault = MaxDeviationApproximatorExact;
using ErrorApproximatorDefault = ErrorApproximatorBehdad<MaxDeviationApproximatorDefault>;
using ApproximatorDefault = ApproximatorMidpointTwoPart<ErrorApproximatorDefault>;


Bezier ApproximateArcWithBezier(const Arc& arc, double& error) {
    glm::dvec2 dp = arc.p1 - arc.p0;
    glm::dvec2 pp = Ortho(dp);
    double d_sq = arc.d * arc.d;
    error = glm::length(dp) * glm::pow(glm::abs(arc.d), 5) / (54.0 * (1.0 + d_sq));
    dp *= ((1.0 - d_sq) / 3.0);
    pp *= (2.0 * arc.d / 3.0);
    glm::dvec2 p0s = arc.p0 + dp - pp;
    glm::dvec2 p1s = arc.p1 - dp - pp;
    return {arc.p0, p0s, p1s, arc.p1};
}


}
