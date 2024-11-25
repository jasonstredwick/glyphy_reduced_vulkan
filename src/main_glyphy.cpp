#include <print>
#include <stdexcept>
#include <string_view>

#include "glyphy/atlas.hpp"
#include "glyphy/buffer.hpp"
#include "glyphy/cache.hpp"
#include "glyphy/glyph/glyphy/interface.hpp"
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
        glyphy::GlyphCache glyph_cache{fontface, atlas, glyphy::ExtractGlyph, glyphy::UpdateAtlasInfo};

        std::string_view text{"Hi my name is Jason.\nLine 2.\nIs this line 3?"};
        glyphy::Buffer text_buffer{};
        text_buffer.MoveTo({0.0, 0.0});
        glyphy::shader::TextVertexData text_vertex_data = text_buffer.AddText(text, glyph_cache, 14);

        float u_contrast{1.0f};
        float u_gamma_adjust{1.0f};
        float u_outline_thickness{1.0f};
        float u_boldness{0.0f};
        glyphy::shader::GlyphInfo glyph_state{.dim={u_contrast, u_gamma_adjust, u_outline_thickness, u_boldness}};

        Run(atlas.Buffer(), text_vertex_data, glyph_state, std::string_view{"shader.glyphy.frag.spv"});
    } catch (std::exception const& exp) {
        std::println("Exception caught\n{}", exp.what());
    }

    std::println("End");
    return 0;
}
