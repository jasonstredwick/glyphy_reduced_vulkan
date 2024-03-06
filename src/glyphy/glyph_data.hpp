#pragma once


#include <cstdint>
#include <vector>

#include "glyphy/glm.hpp"


namespace glyphy {


/***
 * GlyphData contains the information needed to render a glyph and is relative the original glyph size defined by the
 * font face upem and the positions given by Harfbuzz.  The information provided by Harfbuzz seem to already be
 * transformed and may extend beyond the bounds of the original glyph surface.  For example, the lowercase letter 'y'
 * has negative data points and constrained to the lower left.
 *
 *     *atlas_data- Contains the data used to render the glyph in the shader.  Positional data is expected to be
 *      scaled to the range [0, 1] for both x and y.
 *     *dims- The relative width and height of the atlas_data to the original unit glyph.
 *     *offsets- The origin of the atlas_data from the original unit glyph.
 *     *upem- The font face upem used to calculate the GlyphData.
 */
template <typename AtlasDataUnit_t>
struct GlyphData {
    glm::dvec2 dims{0.0, 0.0};
    glm::dvec2 offsets{0.0, 0.0};
    uint32_t upem{0};
    std::vector<AtlasDataUnit_t> atlas_data{};
};


}
