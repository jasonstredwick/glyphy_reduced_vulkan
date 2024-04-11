#pragma once


#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <expected>
#include <limits>
#include <numeric>
#include <ranges>
#include <span>
#include <type_traits>
#include <vector>

#include "glyphy/glm.hpp"

#include "glyphy/glyph/glyphy_reduced/arc.hpp"
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


template <typename T>
concept Approximator_c = requires(const Bezier& a, const Arc& b) {
    { T::ApproximateBezierWithArc(a) } -> std::same_as<Arc>;
    { T::ApproximateError(a, b) } -> std::same_as<double>;
};


/***
 * Implementations
 */


struct ErrorApproximatorJMS {
    template <typename Rng_t>
    requires std::ranges::input_range<Rng_t> && std::same_as<std::ranges::range_value_t<Rng_t>, double>
    static std::vector<glm::dvec2> LerpPointsFromLine(const glm::dvec2& p0, const glm::dvec2& p1, Rng_t&& ts) {
        std::vector<glm::dvec2> results{};
        results.reserve(ts.size());
        std::ranges::transform(ts, std::back_inserter(results), [&p0, &p1](auto t) { return glm::mix(p0, p1, t); });
        return results;
    }

    static double ApproximateError(const Bezier &bezier, const Arc &arc) {
        if (!std::isfinite(arc.d)) { return arc.d; }
        static const std::array<double, 5> t_vals{0.25, 0.5, 0.75};
        const std::vector<glm::dvec2> arc_points = (glm::abs(arc.d) < std::numeric_limits<double>::epsilon()) ?
            ErrorApproximatorJMS::LerpPointsFromLine(arc.p0, arc.p1, t_vals) :
            LerpPoints(arc, t_vals);
        return std::ranges::fold_left(std::views::zip(t_vals, arc_points), 0.0, [&bezier](auto v, auto&& data) {
            const auto [t, arc_point] = data;
            return v + glm::distance(arc_point, LerpPoint(bezier, t));
        });
    }
};


template <ErrorApproximator_c ErrorApproximator>
struct ApproximatorMidpointSimple {
    static Arc ApproximateBezierWithArc(const Bezier &bezier) {
        return CreateArc(bezier.p0, bezier.p3, LerpPoint(bezier, 0.5));
    }

    static double ApproximateError(const Bezier& bezier, const Arc& arc) {
        if (!std::isfinite(arc.d)) { return arc.d; }
        return ErrorApproximator::ApproximateError(bezier, arc);
    }
};


template <Approximator_c Approximator>
class ApproximatorSpringSystem {
    static void CalcArcs(const Bezier& bezier,
                         const Approximator& appx,
                         const std::span<double>& ts,
                         std::vector<double>& segment_errors,
                         std::vector<Arc>& arcs) {
        for (size_t i : std::views::iota(static_cast<size_t>(0), ts.size() - 1)) {
            Bezier bezier_segment = GetSegment(bezier, ts[i], ts[i + 1]);
            arcs.push_back(appx.ApproximateBezierWithArc(bezier_segment));
            segment_errors[i] = appx.ApproximateError(bezier_segment, arcs.back());
        }
    }

    static size_t Jiggle(const Bezier& b,
                         const Approximator& appx,
                         const double tolerance,
                         std::span<double>& ts,
                         std::vector<double>& segment_errors,
                         std::vector<Arc>& arcs) {
        const double conditioner = tolerance * 0.01;
        const size_t N = ts.size() - 1;
        const size_t max_jiggle = static_cast<size_t>(std::log2(N) + 1);

        const double sum = std::accumulate(segment_errors.begin(), segment_errors.end(), 0.0);
        std::ranges::for_each(segment_errors, [sum](auto& e) { e /= sum; });

        size_t jiggle_count = 0;
        for (; jiggle_count < max_jiggle; ++jiggle_count) {
            // Recompute the segment lengths based on the error for each generated arc previously computed.
            double total = 0;
            for (size_t i : std::views::iota(static_cast<size_t>(0), N)) {
                const double segment_length = ts[i + 1] - ts[i];
                const double k_inv = segment_length * glm::pow(segment_errors[i] + conditioner, -0.3);
                total += k_inv;
                segment_errors[i] = k_inv;
            }

            ts[0] = 0.0;
            for (size_t i : std::views::iota(static_cast<size_t>(1), ts.size())) {
                ts[i] = ts[i - 1] + (segment_errors[i - 1] / total) + (2.0 * std::numeric_limits<double>::epsilon());
            }
            std::ranges::for_each(ts, [max_t=ts.back()](auto& t) { t /= max_t; });
            ts[N] = 1.0; // Do this to get real 1.0, not .9999999999999998!

            // Calculate arcs with new segment lengths
            arcs.clear();
            segment_errors.clear();
            CalcArcs(b, appx, ts, segment_errors, arcs);

            const auto [min_error, max_error] = std::ranges::minmax(segment_errors);
            if (max_error < tolerance || (2.0 * min_error - max_error > tolerance)) { break; }
        }

        return jiggle_count;
    }

public:
    static std::vector<Arc> ApproximateBezierWithArcs(const Bezier &bezier,
                                                      const Approximator& appx,
                                                      const double tolerance,
                                                      const size_t max_segments) {
        // Handle fully-degenerate cases.
        glm::dvec2 v1(bezier.p1 - bezier.p0);
        glm::dvec2 v2(bezier.p2 - bezier.p0);
        glm::dvec2 v3(bezier.p3 - bezier.p0);
        if (Cross(v1, v2) == 0.0 && Cross(v2, v3) == 0.0) {
            // Curve has no area;  If endpoints are NOT the same, replace with a line segment.  Otherwise fully skip.
            if (bezier.p0 != bezier.p1) { return {{.p0=bezier.p0, .p1=bezier.p3, .d=0.0}}; }
            return {};
        }

        std::vector<Arc> arcs1{}, arcs2{}; // reusable buffers
        arcs1.reserve(max_segments);
        arcs2.reserve(max_segments);
        std::vector<Arc>* working_arcs = &arcs1;
        std::vector<Arc>* best_arcs = &arcs2;
        std::vector<double> segment_errors{}; // reusable buffer
        segment_errors.reserve(max_segments);
        std::vector<double> bezier_segment_positions(max_segments + 1); // [0..1]; reusable buffer
        size_t total_jiggle_count = 0;

        best_arcs->push_back({.p0=bezier.p0, .p1=bezier.p3, .d=0.0});
        double best_error = appx.ApproximateError(bezier, best_arcs->front());

        for (size_t N : std::views::iota(static_cast<size_t>(1), max_segments)) {
            std::span<double> ts{bezier_segment_positions.data(), N + 1};

            // Subdivide curve into N equal sized segments.
            ts[0] = 0.0;
            std::ranges::transform(
                std::views::iota(static_cast<int64_t>(1), static_cast<int64_t>(N)),
                std::next(ts.begin()),
                [&N](auto i) -> double { return static_cast<double>(i) / static_cast<double>(N); });
            ts[N] = 1.0; // Do this out of the loop to get real 1.0, not .9999999999999998!

            working_arcs->clear();
            segment_errors.clear();

            CalcArcs(bezier, appx, ts, segment_errors, *working_arcs);
            double max_error = std::ranges::max(segment_errors);
            if (max_error < best_error) {
                std::ranges::swap(working_arcs, best_arcs);
                best_error = max_error;
                // Added; as it seems that if the original arcs error is good then why jiggle.
                if (best_error <= tolerance) { break; }
            }

            // Jiggle happens at most once per CalcArcs. Provides small variation in segment size based on the
            // error for each segment.
            for (const auto& error : segment_errors) {
                if (error <= tolerance) {
                    total_jiggle_count += Jiggle(bezier, appx, tolerance, ts, segment_errors, *working_arcs);
                    break;
                }
            }
            max_error = std::ranges::max(segment_errors);
            if (max_error < best_error) {
                std::ranges::swap(working_arcs, best_arcs);
                best_error = max_error;
                if (best_error <= tolerance) { break; }
            }
        }

        std::vector<Arc> results = *best_arcs;
        return results;
    }
};


}
