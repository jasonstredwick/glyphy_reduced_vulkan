#pragma once


#include <memory>
#include <string>
#include <vector>

#include <harfbuzz/hb.h>

#include "glyphy/harfbuzz/fontface.hpp"
#include "glyphy/harfbuzz/utility.hpp"


namespace glyphy::harfbuzz {


/***
 * Object to manage a Harfbuzz buffer and provide those buffer related functions that are utilized.
 */
struct Buffer {
    std::unique_ptr<hb_buffer_t, decltype(&hb_buffer_destroy)> buffer_ptr{hb_buffer_create(), &hb_buffer_destroy};

    void AddUTF8(const char* utf8, int text_length, unsigned int item_offset, int item_length) {
        hb_buffer_add_utf8(BufferRaw(), utf8, text_length, item_offset, item_length);
    }
    void AddUTF8(const std::string& utf8, int text_length, unsigned int item_offset, int item_length) {
        hb_buffer_add_utf8(BufferRaw(), utf8.c_str(), text_length, item_offset, item_length);
    }
    hb_buffer_t* BufferRaw() { return buffer_ptr.get(); }
    void Clear() { hb_buffer_clear_contents(BufferRaw()); }
    auto GlyphInfos() { return AsVector(BufferRaw(), hb_buffer_get_glyph_infos); }
    auto GlyphInfosAsSpan() { return AsSpan(BufferRaw(), hb_buffer_get_glyph_infos); }
    auto GlyphPositions() { return AsVector(BufferRaw(), hb_buffer_get_glyph_positions); }
    auto GlyphPositionsAsSpan() { return AsSpan(BufferRaw(), hb_buffer_get_glyph_positions); }
    void GuessSegmentProperties() { hb_buffer_guess_segment_properties(BufferRaw()); }
    void Shape(FontFace& fontface, const std::vector<hb_feature_t>& shaping_features={}) {
        hb_shape(fontface.Font(),
                 BufferRaw(),
                 shaping_features.empty() ? nullptr : shaping_features.data(),
                 shaping_features.empty() ? 0 : static_cast<unsigned int>(shaping_features.size()));
    }
};


}
