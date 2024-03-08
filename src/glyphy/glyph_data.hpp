#pragma once


#include <cstdint>
#include <vector>

#include "glyphy/glm.hpp"

#include "glyphy/extents.hpp"


namespace glyphy {


/***
 * GlyphData contains the information needed to render a glyph and is relative the original glyph size defined by the
 * font face upem and the positions given by Harfbuzz.  The information provided by Harfbuzz seem to already be
 * transformed and may extend beyond the bounds of the original glyph surface.  For example, the lowercase letter 'y'
 * has negative data points and constrained to the lower left.
 *
 * Glyph data is scaled to the unit em by dividing by the upem.  Then the glyph data is constrained to the area
 * containing shape information padded to a square using the larger axis.  Finally, the data is normalized to a range
 * of [0..1] for both x and y axes forming a unit square for the shader.
 *
 *     *atlas_data- Contains the data used to render the glyph in the shader.
 *     *dims- The dimensions of the glyph shape data in em units.
 *     *offsets- The offset of the glyph shape data in em units from the original glyph origin.
 *     *uniform_dim- The square dimension of the glyph render surface.
 *     *upem- The font face upem; i.e. units per em.
 */
template <typename AtlasDataUnit_t>
struct GlyphData {
    glm::dvec2 offsets{0.0, 0.0};
    glm::dvec2 dims{0.0, 0.0};
    double uniform_dim{0.0};
    uint32_t upem{0};
    std::vector<AtlasDataUnit_t> atlas_data{};
};


}
