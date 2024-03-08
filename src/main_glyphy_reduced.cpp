#include <print>
#include <stdexcept>
#include <string_view>

#include "glyphy/atlas.hpp"
#include "glyphy/buffer.hpp"
#include "glyphy/cache.hpp"
#include "glyphy/glyph/glyphy_reduced/interface.hpp"
#include "glyphy/harfbuzz/font_library.hpp"
#include "glyphy/harfbuzz/fontface.hpp"
#include "glyphy/shader.hpp"

#include "vulkan_app.hpp"


int main(int argc, char** argv) {
    std::println("Start");

    try {
        glyphy::harfbuzz::FontLibrary font_library{};
        auto font_id = font_library.GetOrInsertId("default-font.ttf");
        glyphy::harfbuzz::FontFace& fontface = font_library.GetOrCreateFont(font_id);
        const size_t atlas_chunk_size = 64;
        const size_t atlas_initial_reserve = 128 * 256;
        glyphy::Atlas<glyphy::AtlasDataUnit, glyphy::MAX_ATLAS_ID> atlas{atlas_chunk_size, atlas_initial_reserve};
        glyphy::GlyphCache glyph_cache{fontface, atlas, glyphy::ExtractGlyph};

        std::string_view text{"REDUCED: Hi my name is Jason.\nREDUCED: Line 2.\nREDUCED: Is this line 3?"};
        glyphy::Buffer text_buffer{};
        text_buffer.MoveTo({0.0, 0.0});
        glyphy::shader::TextVertexData text_vertex_data = text_buffer.AddText(text, glyph_cache, 14);

        Run(atlas.Buffer(), text_vertex_data, std::string_view{"shader.glyphy_reduced.frag.spv"});
    } catch (std::exception const& exp) {
        std::println("Exception caught\n{}", exp.what());
    }

    std::println("End");
    return 0;
}
