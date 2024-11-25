#pragma once


#include <array>
#include <limits>
#include <span>
#include <vector>

#include "glyphy/glm.hpp"
#include "glm/gtc/constants.hpp"

#include "glyphy/glyph/glyphy_reduced/endpoint.hpp"
#include "glyphy/math.hpp"


/***
 * Variant of Arc class and functionality from Glyphy
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-geometry.hh
 */


namespace glyphy {


/***
 * Constants
 */
constexpr const double ONE_DEGREE = 1.0 * (glm::pi<glm::f64>() / 180.0);


/***
 * Forward declarations
 */
struct Arc;


// Creation functions
Arc CreateArc(const glm::dvec2 &p0, const glm::dvec2 &p1, const glm::dvec2 &pm);

// Descriptive functions
glm::dvec2 Center(const Arc&);
template <typename Rng_t> std::vector<glm::dvec2> LerpPoints(const Arc& arc, Rng_t&& ts);
double RadiusSq(const Arc&);
std::array<glm::dvec2, 2> Tangents(const Arc&);
bool WedgeContainsPoint(const Arc&, const glm::dvec2&);


/***
 * Implementation
 */


/***
 * Arc is defined by its two endpoints and the tangent of half the angle between the vector formed by the endpoints and
 * the center of the circle defined by the arc.
 *
 * NOTE: The code is written in function form in order to parallel the same functionality implemented for the shader.
 *
 * Some details worth noting when creating and utilizing an arc represented this way.
 * 1. All functions consider the arc going from p0 to p1 in a counter-clockwise fashion.
 * 2. A value of zero for d means the arc should be treated like a line, but could also represent a point if the
 *    endpoints are the same.
 * 3. The angle encoded in d is always part of the right triangle formed by an endpoint, the midpoint of the chord
 *    <p0, p1>, and the center of the circle formed by the arc.  Therefore the value of the angle is always of the
 *    range [0, pi/2].
 * 4. Extracting the angle from d using arctan is valid because it meets the domain restrictions required to uniquely
 *    calculate the inverse.
 * 5. The angle encoded in d is divided by 2 so it can be reconstituted for solving sin/cos/tan of that angle
 *    using formulaic simplifications rather than explicitly calling trig functions.  These simplifications rely on the
 *    use of 2 * angle.  See sin2atan, cos2atan, and tan2atan.
 * 6. The sign of d may be modified to ensure the vector always points from the circle center to bisect the chord.
 */
struct Arc {
    glm::dvec2 p0{0, 0};
    glm::dvec2 p1{0, 0};
    double d{0};
    bool operator<=>(const Arc&) const = default;
};


glm::dvec2 Center(const Arc& arc) {
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    glm::dvec2 pm = Midpoint(arc.p0, arc.p1);
    return pm + Ortho(pm - arc.p0) * tan2atan(arc.d);
    // Alternative:
    //glm::dvec2 v = (arc.p1 - arc.p0) * 0.5;
    //return arc.p0 + v + Ortho(v) * tan2atan(arc.d);
}


/***
 * Create an arc given two endpoints and a point along the arc between the endpoints.
 *
 * The method used to create an arc relies on inscribed angles about a point, pm, along the arc and the circle center.
 * The inscribed angle, theta, is the angle between the vectors <p0 - pm> and <p1 - pm>.
 *     cos(theta) <  0 -> minor arc
 *     cos(theta) == 0 -> semi-circle
 *     cos(theta) >  0 -> major arc | theta defines the remaining minor arc
 *
 * The vector opposite beta should point inward towards the center creating the right triangle p0, center, and
 * the point halfway along the chord.  Therefore, beta will be negated if the counter clockwise rotation of the
 * vector <p0, p1> would have it point away from the center.  This is required depending on the order of p0 and p1
 * along the arc.
 */
Arc CreateArc(const glm::dvec2 &p0, const glm::dvec2 &p1, const glm::dvec2 &pm) {
    // If the midpoint is equal to at least one of the endpoints then mark as line (or point if all three are equal).
    if (glm::all(glm::equal(p0, pm)) || glm::all(glm::equal(p1, pm))) { return {.p0=p0, .p1=p1, .d=0.0}; }

    const glm::dvec2 pm0 = p0 - pm;
    const glm::dvec2 pm1 = p1 - pm;
    const glm::dvec2 upm0 = glm::normalize(pm0);
    const glm::dvec2 upm1 = glm::normalize(pm1);
    const double cos_theta = glm::dot(upm0, upm1);
    if (cos_theta < glm::cos(glm::pi<glm::f64>() - ONE_DEGREE)) {
        // If the angle is too great; i.e. approaching equal but opposite then consider it a line.
        return {.p0=p0, .p1=p1, .d=0.0};
    }
    if (cos_theta > -std::numeric_limits<double>::epsilon()) {
        // Only handle arcs for angles greater than pi/2; (theta < pi/2) -> semi-circle to full circle.
        // Assumes that the caller should break up the arc into smaller units and call again if desired.
        return {.p0=p0, .p1=p1, .d=std::numeric_limits<double>::infinity()};
    }

    const double inscribed_theta = glm::acos(cos_theta);
    //const double central_theta = 2.0 * inscribed_theta;
    //const double alpha = glm::two_pi<glm::f64>() - central_theta;

    const glm::dvec2 p01 = p1 - p0;
    const glm::dvec2 up01 = glm::normalize(p01);
    const glm::dvec2 up0m = -upm0;
    const double sin_theta = Cross(up01, up0m);
    const bool is_pm_left = sin_theta >= 0.0;

    double beta = beta = inscribed_theta - glm::half_pi<glm::f64>();
    if (is_pm_left) { beta *= -1.0; } // center is to the right; p0 -> p1 is clockwise
    return {.p0=p0, .p1=p1, .d=glm::tan(beta / 2.0)};


#if 0
    const glm::dvec2 pm0 = p0 - pm;
    const glm::dvec2 pm1 = p1 - pm;
    const glm::dvec2 upm0 = glm::normalize(pm0);
    const glm::dvec2 upm1 = glm::normalize(pm1);
    const double cos_theta = glm::dot(upm0, upm1);
    const double inscribed_theta = glm::acos(cos_theta);
    //const double central_theta = 2.0 * inscribed_theta;
    //const double alpha = glm::two_pi<glm::f64>() - central_theta;

    const glm::dvec2 p01 = p1 - p0;
    const glm::dvec2 up01 = glm::normalize(p01);
    const double sin_theta = Cross(up01, upm0);
    const bool is_left = sin_theta >= 0.0;

    double beta = 0;
    if (cos_theta < 0) {
        beta = inscribed_theta - glm::half_pi<glm::f64>();
        if (is_left) { beta *= -1.0; }
    } else if (cos_theta > 0) {
        beta = glm::half_pi<glm::f64>() - inscribed_theta;
        if (!is_left) { beta *= -1.0; }
    }
    return {.p0=p0, .p1=p1, .d=glm::tan(beta / 2.0)};
#endif



    //const glm::dvec2 v = 0.5 * p01;
    //glm::dvec2 v_perp = Ortho(v) * glm::tan(beta);

    //if (alpha > 2.0 * glm::two_thirds<glm::f64>() * glm::pi<glm::f64>()) {
    //    // divide into three arcs
    //} else if (alpha > glm::two_thirds<glm::f64>() * glm::pi<glm::f64>()) {
    //    // divide into two arcs
    //} else {
    //}



    //const double sin_theta = Cross(upm0, upm1);






    //if (glm::all(glm::equal(p0, p1))) { return {.p0=p0, .p1=p1, .d=0}; } // not sure; full circle
    //if (glm::abs(sin_theta) < glm::epsilon<glm::f64>()) { return {.p0=p0, .p1=p1, .d=0}; } // line
    //if (glm::abs(cos_theta) < glm::epsilon<glm::f64>()) { return {.p0=p0, .p1=p1, .d=0}; } // figure out; semi-circle/half-circle

    //const double theta_i = glm::abs(glm::atan(sin_theta, cos_theta));
    //const double beta = cos_theta > 0.0 ? glm::half_pi<double>() - theta_i : theta_i - glm::half_pi<double>();
    /***
     * beta is the angle between the vector <p1 - p0> and the center of the circle.  This value is encoded in the
     * d data member.  A couple other notes to understand the encoding of d.
     *     * beta is divided by 2 so it can be reconstituted while solving for sin/cos/tan of beta using the
     *       formulaic simplifications with 2*theta rather than explicitly call a trig function on an angle.
     *       See sin2atan, cos2atan, and tan2atan.
     *     * As beta is an angle in a right triangle, it is always in the range [0, pi/2].  However, as often times
     *       a vector is needed and not just the magnitude of one of the sides, the <p1 - p0> vector (side adjacent)
     *       and its perpendicular vector (side opposite) are used.  To be able to orient the perpendicular vector
     *       towards or away from the center, a sign is applied to d.  The sign is negative if pm is to the left of
     *       <p1 - p0> and the rotation needs to be clockwise.  It is positive if the rotation is counter clockwise.
     *
     * pm is left  of <p1 - p0>, angle < pi/2 -> center left  of <p1 - p0>; rotation counter clockwise; sign +
     * pm is left  of <p1 - p0>, angle > pi/2 -> center right of <p1 - p0>; rotation         clockwise; sign -
     * pm is right of <p1 - p0>, angle < pi/2 -> center left  of <p1 - p0>; rotation         clockwise; sign -
     * pm is right of <p1 - p0>, angle > pi/2 -> center right of <p1 - p0>; rotation counter clockwise; sign +
     *
     * left  of <p1 - p0> ; cross(<p1, p0>, <pm, p0>) > 0; +
     * right of <p1 - p0> ; cross(<p1, p0>, <pm, p0>) < 0; -
     * angle < pi/2 ; cos_theta > 0; +
     * angle > pi/2 ; cos_theta < 0; -
     *
     * d | c | s
     * _________
     * +   +   +
     * -   +   -
     * +   -   -
     * -   -   +
     */
    /*
    const double d = glm::sign(cos_theta * Cross(p1 - p0, pm - p0)) * beta / 2.0;
    // half the angle between pm0 and pm1
    // bool complement; // was function parameter
    //const double d = glm::tan(((Atan(pm1) - Atan(pm0)) / 2.0) - (complement ? 0 : glm::half_pi<glm::f64>()));
    return Arc{.p0=p0, .p1=p1, .d=d};
    */
}


// Assumes Arc is not a line (arc.d == 0) or invalid (ard.d == infinity)
template <typename Rng_t>
requires std::ranges::input_range<Rng_t> && std::same_as<std::ranges::range_value_t<Rng_t>, double>
std::vector<glm::dvec2> LerpPoints(const Arc& arc, Rng_t&& ts) {
    std::vector<glm::dvec2> results{};
    results.reserve(ts.size());

    const glm::dvec2 arc_center = Center(arc);
    // +beta == counter-clockwise; -beta clockwise
    // arc.d = tan(beta/2)
    // tan(-theta) == -tan(theta)
    const double beta = 2.0 * glm::atan(abs(arc.d));
    // 2 * (pi/2 - beta) -> pi - 2beta; (pi/2 - beta) is half the angle from p0 to p1 about the center

    const double theta = 2.0 * (glm::half_pi<glm::f32>() - beta);

    // Center the circle at the origin.
    const double x = arc.p0.x - arc_center.x;
    const double y = arc.p0.y - arc_center.y;

    // Rotate p0 about the origin
    for (auto& t : ts) {
        const double dt = t * theta;
        const double cos_dt = glm::cos(dt);
        const double sin_dt = glm::sin(dt);
        // Shift circle back to arc_center
        results.push_back(arc_center + glm::dvec2{x * cos_dt - y * sin_dt, x * sin_dt + y * cos_dt});
    }

    return results;
}


double Radius(const Arc& arc) {
    return glm::distance(Center(arc), arc.p0);
    // Alternative 1
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    //glm::dvec2 v = (arc.p1 - arc.p0) / 2.0;
    //return glm::length(v + Ortho(v) * tan2atan(arc.d));
    // Alternative 2: (assumes arc.d is stored in tan(theta / 2))
    // cos(t) = A / H; H = A / cos(t); A = |p1 - p0| / 2.0; H = |p1 - p0| / (2.0 * cos(t))
    // cos(x) == cos(x); cos(-x) == cos(x)
    //return glm::length(arc.p1 - arc.p0) / (2.0 * cos2atan(arc.d));
}


double RadiusSq(const Arc& arc) {
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    glm::dvec2 v = (arc.p1 - arc.p0) / 2.0;
    glm::dvec2 w = v + Ortho(v) * tan2atan(arc.d);
    return glm::dot(w, w);
}


std::array<glm::dvec2, 2> Tangents(const Arc& arc) {
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    //
    // Find tangents to arc.p0 and arc.p1 where both tangents point into the wedge formed by the arc.  That means
    // that <center - arc.p0> is rotated counter clockwise and <center - arc.p1> is rotated clockwise.
    // Returns vectors with magnitudes twice the length of the radius vector.
    glm::dvec2 v = (arc.p1 - arc.p0);
    glm::dvec2 vp = Ortho(v) * tan2atan(arc.d);
    return {Ortho(v + vp), OrthoC(-v + vp)};
}


 bool WedgeContainsPoint(const Arc& arc, const glm::dvec2& p) {
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    // need tan2atan to ensure Ortho points in the correct direction
    // -1.0 causes vp to point away from center towards chord p0, p1
    glm::dvec2 vp = -1.0 * glm::normalize(Ortho(arc.p1 - arc.p0) * tan2atan(arc.d));
    glm::dvec2 center = Center(arc);
    return glm::dot(vp, glm::normalize(arc.p0 - center)) >= glm::dot(vp, glm::normalize(p - center));
}


}
