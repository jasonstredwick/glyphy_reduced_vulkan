#pragma once


#include <algorithm>
#include <cmath>
#include <limits>

#include "glyphy/glm.hpp"


/***
 * Variant from Glyphy
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-extents.cc
 */


namespace glyphy {


struct Extents {
    double min_x{ std::numeric_limits<double>::infinity()};
    double min_y{ std::numeric_limits<double>::infinity()};
    double max_x{-std::numeric_limits<double>::infinity()};
    double max_y{-std::numeric_limits<double>::infinity()};

    constexpr void Add(const glm::dvec2& p) {
        min_x = std::ranges::min(p.x, min_x);
        min_y = std::ranges::min(p.y, min_y);
        max_x = std::ranges::max(p.x, max_x);
        max_y = std::ranges::max(p.y, max_y);
    }

    constexpr void Clear() {
        min_x =  std::numeric_limits<double>::infinity();
        min_y =  std::numeric_limits<double>::infinity();
        max_x = -std::numeric_limits<double>::infinity();
        max_y = -std::numeric_limits<double>::infinity();
    }

    constexpr void Extend(const Extents& other) {
        min_x = std::ranges::min(min_x, other.min_x);
        min_y = std::ranges::min(min_y, other.min_y);
        max_x = std::ranges::max(max_x, other.max_x);
        max_y = std::ranges::max(max_y, other.max_y);
    }

    constexpr bool Includes(const glm::dvec2& p) const {
        return min_x <= p.x && p.x <= max_x &&
               min_y <= p.y && p.y <= max_y;
    }

    inline bool IsEmpty() const {
        return !std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) || !std::isfinite(max_y);
    }

    constexpr void Scale(const glm::dvec2& scale) {
        min_x *= scale.x;
        max_x *= scale.x;
        min_y *= scale.y;
        max_y *= scale.y;
    }
};


}
