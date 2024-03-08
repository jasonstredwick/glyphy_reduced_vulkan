#pragma once


#include <limits>

#include "glyphy/glm.hpp"


namespace glyphy {


struct Endpoint {
    glm::dvec2 p{0.0, 0.0};
    double d{std::numeric_limits<double>::infinity()};
    double dummy{0.0};
};


}
