#pragma once


#include <limits>
#include <vector>

#include "glyphy/glm.hpp"

#include "glyphy/glyph/glyphy_reduced/arc_bezier.hpp"
#include "glyphy/glyph/glyphy_reduced/endpoint.hpp"
#include "glyphy/harfbuzz/glyph_extractor.hpp"


/***
 * Based on glyphy_arc_accumulator_t
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-arcs.cc
 */


namespace glyphy {


constexpr const double DEFAULT_EXTRACT_TOLERANCE = 0.5;//5e-4; //0.5; // 1.0 / 2048.0; // 5e-4
constexpr const size_t DEFAULT_MAX_SEGMENTS = 100;


/***
 * As a glyph is shaped, it is converted to one or more series of endpoints where each series represents a closed
 * loop.  A d value of infinity is used to signify the beginning of each series and all series are stored linearly
 * within a vector.
 */
struct GlyphExtractor : public glyphy::harfbuzz::GlyphExtractor {
    struct Options {
        double tolerance{DEFAULT_EXTRACT_TOLERANCE};
        size_t max_segments{DEFAULT_MAX_SEGMENTS};
    };

    std::vector<Endpoint> endpoints{};
    Options options{};

    GlyphExtractor() noexcept = default;
    GlyphExtractor(const Options& options) noexcept : options{options} {}

    void Accumulate(const glm::dvec2& p, double d) {
        if (IsCurrent(p)) { return; }
        else if (glm::isinf(d)) {
            // Emit moveto lazily, for cleaner outlines
            need_moveto = true;
            current_point = p;
            return;
        }
        if (need_moveto) {
            endpoints.emplace_back(current_point, std::numeric_limits<double>::infinity());
            start_point = current_point;
            need_moveto = false;
        }
        endpoints.emplace_back(p, d);
        current_point = p;
    }

    void ExtendFromBezier(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) override {
        //using Approximator = ApproximatorMidpointTwoPart<ErrorApproximatorDefault>;
        using Approximator = ApproximatorMidpointSimple<ErrorApproximatorJMS>;
        using System = ApproximatorSpringSystem<Approximator>;
        std::vector<Arc> arcs = System::ApproximateBezierWithArcs(Bezier{p0, p1, p2, p3},
                                                                  Approximator{},
                                                                  options.tolerance,
                                                                  options.max_segments);
        MoveToExec(p0);
        for (const auto& arc : arcs) { Accumulate(arc.p1, arc.d); }
    }

    void ExtendLine(const glm::dvec2& p) override { Accumulate(p, 0.0); }

    void MoveToExec(const glm::dvec2& p) override {
        if (endpoints.empty() || !IsCurrent(p)) {
            Accumulate(p, std::numeric_limits<double>::infinity());
        }
    }
};


}
