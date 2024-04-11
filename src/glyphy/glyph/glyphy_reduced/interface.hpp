#pragma once


#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

#include "glyphy/extents.hpp"
#include "glyphy/glyph_data.hpp"
#include "glyphy/glyph/glyphy_reduced/arc.hpp"
#include "glyphy/glyph/glyphy_reduced/endpoint.hpp"
#include "glyphy/glyph/glyphy_reduced/extractor.hpp"
#include "glyphy/glyph/glyphy_reduced/outline.hpp"
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
    const double em_scale = 1.0 / static_cast<double>(upem);
    const double tolerance = DEFAULT_EXTRACT_TOLERANCE;// 0.5;//em_scale * DEFAULT_EXTRACT_TOLERANCE; // in font design units
    const size_t max_segments = 100;

    GlyphExtractor glyph_extractor{{.tolerance=tolerance, .max_segments=max_segments}};
    glyphy::harfbuzz::FontGetGlyphShape<GlyphExtractor, double>(fontface.Font(), glyph_index, &glyph_extractor);
    if (glyph_extractor.endpoints.empty()) { return {.upem=upem}; }

    // Scale to the unit em.
    std::ranges::for_each(glyph_extractor.endpoints, [em_scale](auto& e) { e.p *= em_scale; });

    // May reverse subset of endpoints and negate their d value;
    // but otherwise no addition, removal, or modification of endpoints.
    glyphy::AdjustWinding(glyph_extractor.endpoints);

    // Constrain surface to the area containing shape data.
    Extents extents{};
    std::ranges::for_each(glyph_extractor.endpoints, [&extents](auto& p) { extents.Add(p); }, &Endpoint::p);
    const glm::dvec2 offsets{extents.min_x, extents.min_y};
    const glm::dvec2 dims{extents.max_x - extents.min_x, extents.max_y - extents.min_y};
    const double uniform_dim = std::ranges::max(dims.x, dims.y);

    // Extract "side" information used for sdf interior calculations
    glm::vec4 atlas_info{
        std::numeric_limits<float>::infinity(), static_cast<float>(glyph_extractor.endpoints.size()), 0.0, 0.0};

    // Normalize data to [0..1] range.
    std::ranges::for_each(glyph_extractor.endpoints, [&dims, &offsets, uniform_dim](auto& p) {
        p.x = (p.x - offsets.x) / uniform_dim;
        p.y = (p.y - offsets.y) / uniform_dim;
    }, &Endpoint::p);

    // Compile result
    GlyphData<AtlasDataUnit> glyph_data{
        .offsets=offsets,
        .dims=dims,
        .atlas_info=atlas_info,
        .uniform_dim=uniform_dim,
        .upem=upem
    };
    glyph_data.atlas_data.reserve(glyph_extractor.endpoints.size());
    // Convert data for shader.
    std::ranges::transform(glyph_extractor.endpoints, std::back_inserter(glyph_data.atlas_data), [](auto& endpoint) {
        return AtlasDataUnit{
            .p={static_cast<float>(endpoint.p.x), static_cast<float>(endpoint.p.y)},
            //.d=(std::isfinite(endpoint.d) ? static_cast<float>(0.0f) : GLYPHY_INFINITY)
            .d=(std::isfinite(endpoint.d) ? static_cast<float>(endpoint.d) : GLYPHY_INFINITY)
        };
    });
    return glyph_data;
}


glm::vec4 UpdateAtlasInfo(const glm::vec4& atlas_info, const size_t atlas_id) {
    return {static_cast<float>(atlas_id), atlas_info[1], atlas_info[2], atlas_info[3]};
}


}
