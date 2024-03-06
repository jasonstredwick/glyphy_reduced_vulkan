#pragma once


#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

#include "glyphy/extents.hpp"
#include "glyphy/glyph_data.hpp"
#include "glyphy/glyph/glyphy/endpoint.hpp"
#include "glyphy/glyph/glyphy/extractor.hpp"
//#include "glyphy/glyph/glyphy/outline.hpp"
#include "glyphy/harfbuzz/fontface.hpp"
#include "glyphy/harfbuzz/shape.hpp"


namespace glyphy {


constexpr const float GLYPHY_EPSILON = 1e-5f;
constexpr const float GLYPHY_INFINITY = 1e9f;
/***
 * Atlas id is used to look up data in arrays.  Since the value may end up in a float the max size of the array and
 * index is the largest integer with no exponent that can be stored in a float (minus 1).  Reserving the last digit
 * for out of range value.
 *     std::pow(2, std::numeric_limits<float>::digits)) - 1
 *
 * TODO convert to function call once std::pow is constexpr
 */
constexpr const size_t MAX_ATLAS_ID = 16777215;
//constexpr const double ENLIGHTEN_MAX = 0.01; /* Per EM */
//constexpr const double EMBOLDEN_MAX = 0.024; /* Per EM */
//constexpr const double MIN_FONT_SIZE = 14.0;


struct AtlasDataUnit {
    glm::vec2 p{0.0f, 0.0f};
    float d{GLYPHY_INFINITY};
    float dummy{0.0f};
};


GlyphData<AtlasDataUnit> ExtractGlyph(glyphy::harfbuzz::FontFace& fontface, const uint32_t glyph_index) {
    const uint32_t upem = fontface.Upem();
    const double upem_d = static_cast<double>(upem);
    const double scale = 1.0 / upem_d;
    //const double font_size = MIN_FONT_SIZE;
    //const double faraway = upem_d / (font_size * std::numbers::sqrt2); // 103
    //const double enlighten = upem_d * ENLIGHTEN_MAX;                   // 20.48
    //const double embolden = upem_d * EMBOLDEN_MAX;                     // 49.152
    const double tolerance = 0.5;//upem_d * DEFAULT_EXTRACT_TOLERANCE; // in font design units

    GlyphExtractor glyph_extractor{tolerance};
    glyphy::harfbuzz::FontGetGlyphShape<GlyphExtractor, double>(fontface.Font(), glyph_index, &glyph_extractor);
    if (!glyph_extractor.endpoints.size()) { return {.upem=upem}; }

    // May reverse subset of endpoints and negate their d value;
    // but otherwise no addition, removal, or modification of endpoints.
    //glyphy::outline::WindingFromEvenOdd(glyph_extractor.endpoints, false);

    // Scale and make adjustments to ensure glyphy lies on [0, 1] ranges.  Return adjustments so they can be taken
    // into account when preparing for rendering.
    std::ranges::for_each(glyph_extractor.endpoints, [scale](auto& e) { e.p *= scale; });
    Extents extents{};
    std::ranges::for_each(glyph_extractor.endpoints, [&extents](auto& p) { extents.Add(p); }, &Endpoint::p);
    double width = extents.max_x - extents.min_x;
    double height = extents.max_y - extents.min_y;
    std::ranges::for_each(glyph_extractor.endpoints, [&width, &height, &extents](auto& p) {
        p.x = (p.x - extents.min_x) / width;
        p.y = (p.y - extents.min_y) / height;
    }, &Endpoint::p);

    // Convert Glyph data and information to shader compatible as needed.
    GlyphData<AtlasDataUnit> glyph_data{
        .dims={width, height},
        .offsets={extents.min_x, extents.min_y},
        .upem=upem
    };
    glyph_data.atlas_data.reserve(glyph_extractor.endpoints.size());
    std::ranges::transform(glyph_extractor.endpoints, std::back_inserter(glyph_data.atlas_data), [](auto& endpoint) {
        return AtlasDataUnit{
            .p={static_cast<float>(endpoint.p.x), static_cast<float>(endpoint.p.y)},
            .d=(std::isfinite(endpoint.d) ? static_cast<float>(endpoint.d) : GLYPHY_INFINITY)
        };
    });
    return glyph_data;
}


}
