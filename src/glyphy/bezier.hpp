#pragma once


#include <array>

#include "glyphy/glm.hpp"

#include "glyphy/math.hpp"


/***
 * Variant of Bezier class and functionality from Glyphy
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-geometry.hh
 */


namespace glyphy {


struct Bezier {
    glm::dvec2 p0{0, 0};
    glm::dvec2 p1{0, 0};
    glm::dvec2 p2{0, 0};
    glm::dvec2 p3{0, 0};
};


Bezier GetSegment(const Bezier& bezier, const double t0, const double t1) {
    glm::dvec2 p01 = glm::mix(bezier.p0, bezier.p1, t0);
    glm::dvec2 p12 = glm::mix(bezier.p1, bezier.p2, t0);
    glm::dvec2 p23 = glm::mix(bezier.p2, bezier.p3, t0);
    glm::dvec2 p012 = glm::mix(p01, p12, t0);
    glm::dvec2 p123 = glm::mix(p12, p23, t0);
    glm::dvec2 p0123 = glm::mix(p012, p123, t0);

    glm::dvec2 q01 = glm::mix(bezier.p0, bezier.p1, t1);
    glm::dvec2 q12 = glm::mix(bezier.p1, bezier.p2, t1);
    glm::dvec2 q23 = glm::mix(bezier.p2, bezier.p3, t1);
    glm::dvec2 q012 = glm::mix(q01, q12, t1);
    glm::dvec2 q123 = glm::mix(q12, q23, t1);
    glm::dvec2 q0123 = glm::mix(q012, q123, t1);

    return Bezier{p0123,
                  p0123 + (p123 - p0123) * ((t1 - t0) / (1.0 - t0)),
                  q0123 + (q012 - q0123) * ((t1 - t0) / t1),
                  q0123};
}


glm::dvec2 LerpPoint(const Bezier& bezier, const double t) {
    glm::dvec2 p01 = glm::mix(bezier.p0, bezier.p1, t);
    glm::dvec2 p12 = glm::mix(bezier.p1, bezier.p2, t);
    glm::dvec2 p23 = glm::mix(bezier.p2, bezier.p3, t);
    glm::dvec2 p012 = glm::mix(p01, p12, t);
    glm::dvec2 p123 = glm::mix(p12, p23, t);
    glm::dvec2 p0123 = glm::mix(p012, p123, t);
    return p0123;
}


glm::dvec2 Midpoint(const Bezier& bezier) {
    glm::dvec2 p01 = Midpoint(bezier.p0, bezier.p1);
    glm::dvec2 p12 = Midpoint(bezier.p1, bezier.p2);
    glm::dvec2 p23 = Midpoint(bezier.p2, bezier.p3);
    glm::dvec2 p012 = Midpoint(p01, p12);
    glm::dvec2 p123 = Midpoint(p12, p23);
    glm::dvec2 p0123 = Midpoint(p012, p123);
    return p0123;
}


std::array<Bezier, 2> Split(const Bezier& bezier, const double t) {
    glm::dvec2 p01 = glm::mix(bezier.p0, bezier.p1, t);
    glm::dvec2 p12 = glm::mix(bezier.p1, bezier.p2, t);
    glm::dvec2 p23 = glm::mix(bezier.p2, bezier.p3, t);
    glm::dvec2 p012 = glm::mix(p01, p12, t);
    glm::dvec2 p123 = glm::mix(p12, p23, t);
    glm::dvec2 p0123 = glm::mix(p012, p123, t);
    return {Bezier{.p0=bezier.p0, .p1=p01, .p2=p012, .p3=p0123}, Bezier{.p0=p0123, .p1=p123, .p2=p23, .p3=bezier.p3}};
}


}
