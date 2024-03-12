#pragma once


#include <array>
#include <limits>
#include <vector>

#include "glyphy/glm.hpp"
#include "glm/gtc/constants.hpp"

#include "glyphy/glyph/glyphy/endpoint.hpp"
#include "glyphy/math.hpp"


/***
 * Variant of Arc class and functionality from Glyphy
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-geometry.hh
 */


namespace glyphy {


/***
 * Forward declarations
 */
struct Arc;

// Creation functions
Arc CreateArc(const glm::dvec2 &p0, const glm::dvec2 &p1, const glm::dvec2 &pm);

// Descriptive functions
glm::dvec2 Center(const Arc&);
double ExtendedDist(const Arc& arc, const glm::dvec2& p);
double Radius(const Arc&);
double RadiusSq(const Arc&);
double SegmentDistanceToPoint(const Arc& arc, const glm::dvec2& p);
double SdfFromArcList(const std::vector<Endpoint>& endpoints, const glm::dvec2& p_center);
std::array<glm::dvec2, 2> Tangents(const Arc&);
bool WedgeContainsPoint(const Arc&, const glm::dvec2&);


/***
 * Implementation
 */


struct Arc {
    glm::dvec2 p0{0, 0};
    glm::dvec2 p1{0, 0};
    double d{0};
    bool operator<=>(const Arc&) const = default;
};


glm::dvec2 Center(const Arc& arc) {
    return Midpoint(arc.p0, arc.p1) + Ortho(arc.p1 - arc.p0) / (2.0 * tan2atan(arc.d));
}


Arc CreateArc(const glm::dvec2 &p0, const glm::dvec2 &p1, const glm::dvec2 &pm, bool complement) {
    // If the midpoint is equal to at least one of the endpoints then mark as line (or point if all three are equal).
    if (glm::all(glm::equal(p0, pm)) || glm::all(glm::equal(p1, pm))) { return {.p0=p0, .p1=p1, .d=0}; }

    double angle1 = Atan(p1 - pm);
    double angle0 = Atan(p0 - pm);
    double angle = angle1 - angle0;
    double half_angle = angle / 2.0;
    double final_angle = half_angle - glm::half_pi<double>();
    double d = glm::tan(final_angle);
    return {.p0=p0, .p1=p1, .d=d};
}


double ExtendedDist(const Arc& arc, const glm::dvec2& p) {
    glm::dvec2 m = glm::mix(arc.p0, arc.p1, 0.5);
    glm::dvec2 dp = arc.p1 - arc.p0;
    glm::dvec2 pp = Ortho(dp);
    double d2 = tan2atan(arc.d);
    if (glm::dot(p - m, arc.p1 - m) < 0) {
        return glm::dot(p - arc.p0, glm::normalize(pp + dp * d2));
    } else {
        return glm::dot(p - arc.p1, glm::normalize(pp - dp * d2));
    }

    // Alternative form from shader
#if 0
    glm::dvec2 m = glm::mix(arc.p0, arc.p1, 0.5);
    double d2 = tan2atan(arc.d);
    if (glm::dot(p - m, arc.p1 - m) < 0.0) { return glm::dot(p - arc.p0, glm::normalize((arc.p1 - arc.p0) * glm::dmat2(+d2, -1, +1, +d2))); }
    else                                   { return glm::dot(p - arc.p1, glm::normalize((arc.p1 - arc.p0) * glm::dmat2(-d2, -1, +1, -d2))); }
#endif
}


double Radius(const Arc& arc) {
    return glm::abs(glm::length(arc.p1 - arc.p0)) / (2.0 * tan2atan(arc.d));
}


double SegmentDistanceToPoint(const Arc& arc, const glm::dvec2& p) {
    const glm::dvec2& p0 = arc.p0;
    const glm::dvec2& p1 = arc.p1;
    if (p0 == p1) { return 0.0; }

    glm::dvec2 normal = Ortho(p1 - p0);
    double c = glm::dot(normal, p0);

    // shortest vector from point to line
    double mag = -(glm::dot(normal, p) - c) / glm::length(normal);
    glm::dvec2 z = glm::normalize(normal) * mag + p;
    // Check if z is between p0 and p1.
    bool does_contain = (glm::abs(p1.y - p0.y) > glm::abs(p1.x - p0.x)) ?
        ((z.y - p0.y > 0 && p1.y - p0.y > z.y - p0.y) || (z.y - p0.y < 0 && p1.y - p0.y < z.y - p0.y)) :
        ((0 < z.x - p0.x && z.x - p0.x < p1.x - p0.x) || (0 > z.x - p0.x && z.x - p0.x > p1.x - p0.x));
    if (does_contain) { return mag; }

    double dist_p_p0 = glm::distance(p, p0);
    double dist_p_p1 = glm::distance(p, p1);
    return std::ranges::min(dist_p_p0, dist_p_p1) * (-(glm::dot(normal, p) - c) < 0 ? -1 : 1);
}


double SdfFromArcList(const std::vector<Endpoint>& endpoints, const glm::dvec2& p_center, const double epsilon) {
    if (endpoints.empty()) { return 1.0; }

    glm::dvec2 p_prev = endpoints[0].p;
    Arc closest_arc{.p0=p_prev, .p1=p_prev, .d=0.0};

    double min_dist = std::numeric_limits<double>::infinity();
    int side = 0;
    for (const auto& endpoint : endpoints) {
        Arc arc{.p0=p_prev, .p1=endpoint.p, .d=endpoint.d};
        p_prev = endpoint.p;
        if (!std::isfinite(endpoint.d)) { continue; }

        if (WedgeContainsPoint(arc, p_center)) {
            if (glm::abs(arc.d) < epsilon) { return SegmentDistanceToPoint(arc, p_center); }
            double arc_radius = Radius(arc);
            double dist = glm::distance(Center(arc), p_center);
            double dist_delta = dist - arc_radius;
            bool is_negative = (dist < arc_radius) ^ (arc.d < 0);
            double sdist = glm::abs(dist_delta) * (is_negative ? -1.0 : 1.0);
            double udist = glm::abs(sdist) * (1.0 - epsilon);
            if (udist <= min_dist) {
                min_dist = udist;
                side = sdist >= 0 ? -1 : +1;
            }
        } else {
            double udist = std::ranges::min(glm::distance(p_center, arc.p0), glm::distance(p_center, arc.p1));
            if (udist < min_dist) {
                min_dist = udist;
                side = 0; /* unsure */
                closest_arc = arc;
            } else if (side == 0 && udist == min_dist) {
                /* If this new distance is the same as the current minimum,
                * compare extended distances.  Take the sign from the arc
                * with larger extended distance. */
                double old_ext_dist = ExtendedDist(closest_arc, p_center);
                double new_ext_dist = ExtendedDist(arc, p_center);
                double ext_dist = glm::abs(new_ext_dist) <= glm::abs(old_ext_dist) ? old_ext_dist : new_ext_dist;

                /* For emboldening and stuff: */
                // min_dist = fabs (ext_dist);
                side = ext_dist >= 0 ? +1 : -1;
            }
        }
    }

    if (side == 0) {
        // Technically speaking this should not happen, but it does.  So try to fix it.
        double ext_dist = ExtendedDist(closest_arc, p_center);
        side = ext_dist >= 0 ? +1 : -1;
    }

    return side * min_dist;
}


std::array<glm::dvec2, 2> Tangents(const Arc& arc) {
    glm::dvec2 dp = 0.5 * (arc.p1 - arc.p0);
    glm::dvec2 pp = Ortho(dp) * -sin2atan(arc.d);
    dp = dp * cos2atan(arc.d);
    return {dp + pp, dp - pp};
}


bool WedgeContainsPoint(const Arc& arc, const glm::dvec2& p) {
    auto t = Tangents(arc);
    if (std::abs(arc.d) <= 1.0) {
        return glm::dot((p - arc.p0), t[0]) >= 0.0 && glm::dot((p - arc.p1), t[1]) <= 0.0;
    } else {
        return glm::dot((p - arc.p0), t[0]) >= 0.0 || glm::dot((p - arc.p1), t[1]) <= 0.0;
    }
}


}
