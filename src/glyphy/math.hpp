#pragma once


#include "glyphy/glm.hpp"


/***
 * Variant from Glyphy
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-geometry.hh
 */


namespace glyphy {


double Atan(const glm::dvec2& v) { return glm::atan(v.y, v.x); }
double Cross(const glm::dvec2& lhs, const glm::dvec2& rhs) { return lhs.x * rhs.y - lhs.y * rhs.x; }
double DistanceToSegment(const glm::dvec2 p0, const glm::dvec2 p1, const glm::dvec2 p);
glm::dvec2 Midpoint(const glm::dvec2& a, const glm::dvec2& b) { return (a + b) / 2.0; }
glm::dvec2 Ortho(const glm::dvec2& v) { return {-v.y, v.x}; }
glm::dvec2 OrthoC(const glm::dvec2& v) { return {v.y, -v.x}; }
constexpr auto OrthoCC = Ortho;
glm::dvec2 Rebase(const glm::dvec2& v, const glm::dvec2& bx, const glm::dvec2& by) { return {glm::dot(v, bx),
                                                                                             glm::dot(v, by)}; }
glm::dvec2 Rebase(const glm::dvec2& v, const glm::dvec2& bx)                       { return Rebase(v, bx, Ortho(bx)); }
// half angle formula
double cos2atan(double d) { return (1.0 - d*d) / (1.0 + d*d); } // returns cos(2 * atan(d))
double sin2atan(double d) { return 2.0 * d / (1.0 + d*d); }     // returns sin(2 * atan(d))
double tan2atan(double d) { return 2.0 * d / (1.0 - d*d); }     // returns tan(2 * atan(d))


// review lengyel pg. 109
// https://iquilezles.org/articles/distfunctions2d/
double DistanceToSegment(const glm::dvec2 p0, const glm::dvec2 p1, const glm::dvec2 p) {
    const glm::dvec2 v_01 = p1 - p0;
    const glm::dvec2 v_0p = p - p0;
    double h = glm::clamp(glm::dot(v_0p, v_01) / glm::dot(v_01, v_01), 0.0, 1.0);
    return glm::length(v_0p - v_01 * h);
};


}
