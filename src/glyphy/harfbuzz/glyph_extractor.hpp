#pragma once


#include <type_traits>
#include <vector>

#include "glyphy/glm.hpp"
#include <glm/gtc/epsilon.hpp>


/***
 * Based on glyphy_arc_accumulator_t
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-arcs.cc
 */


namespace glyphy::harfbuzz {


// Values between [16, 16384]; https://harfbuzz.github.io/harfbuzz-hb-face.html#hb-face-get-upem
constexpr const size_t MAX_UPEM_DIGITS = 5;
constexpr const double UPEM_EPSILON = 1e-11;
constexpr const float UPEM_EPSILON_f = 1e-2f;


/***
 * Each font glyph is a series of closed path outlines each formed from a series of connected lines, arcs, and
 * bezier curves.  These paths are extracted and aggregated here during text shaping.
 *
 * The GlyphExtractor is a base class that maps shaping functions to aggregation and storage functions.  There are
 * a number of virtual functions that allow aggregation to be customized.
 */
struct GlyphExtractor {
    glm::dvec2 start_point{0, 0};
    glm::dvec2 current_point{0, 0};
    bool need_moveto{true};

    virtual ~GlyphExtractor() = default;

    bool IsCurrent(const glm::dvec2& p) {
        return glm::all(glm::epsilonEqual(p, current_point, UPEM_EPSILON));
    }

    // Harfbuzz shaping, state machine callback commands
    void ClosePath() {
        if (!need_moveto && !IsCurrent(start_point)) {
            ExtendLine(start_point);
        }
    }

    void CubicTo(double control1_x, double control1_y, double control2_x, double control2_y, double to_x, double to_y)
    {
        ExtendFromBezier(current_point, {control1_x, control1_y}, {control2_x, control2_y}, {to_x, to_y});
    }

    void LineTo(double to_x, double to_y) {
        ExtendLine({to_x, to_y});
    }

    void MoveTo(double to_x, double to_y) {
        ClosePath();
        MoveToExec({to_x, to_y});
    }

    void QuadraticTo(double control_x, double control_y, double to_x, double to_y) {
        glm::dvec2 p1{control_x, control_y};
        glm::dvec2 p2{to_x, to_y};
        ExtendFromBezier(current_point, glm::mix(current_point, p1, 2.0/3.0), glm::mix(p2, p1, 2.0/3.0), p2);
    }

    // Internal management
    virtual void ExtendFromBezier(const glm::dvec2& p0, const glm::dvec2& p1,
                                  const glm::dvec2& p2, const glm::dvec2& p3) = 0;
    virtual void ExtendLine(const glm::dvec2& p) = 0;
    virtual void MoveToExec(const glm::dvec2& p) = 0;
};


}
