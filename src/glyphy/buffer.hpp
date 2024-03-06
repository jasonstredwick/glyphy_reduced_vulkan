/*
 * Copyright 2012 Google, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Google Author(s): Behdad Esfahbod
 */


#pragma once


#include <array>
#include <cstring>
#include <memory>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include "glyphy/glm.hpp"

#include "glyphy/cache.hpp"
#include "glyphy/harfbuzz/buffer.hpp"
#include "glyphy/harfbuzz/fontface.hpp"
#include "glyphy/shader.hpp"


/***
 * Variant of Buffer class and functionality from Glyphy
 * https://github.com/behdad/glyphy/blob/master/demo/demo-buffer.h
 * https://github.com/behdad/glyphy/blob/master/demo/demo-buffer.cc
 */


namespace glyphy {


struct Buffer {
    glm::dvec2 cursor{};
    std::string utf8_contents{};

    void Clear() {
        cursor = {0.0, 0.0};
        utf8_contents.clear();
    }

    glyphy::shader::TextVertexData AddText(const std::string_view& text, auto& glyph_cache, double font_size);
    void MoveTo(const glm::dvec2& p) { cursor = p; }
};


glyphy::shader::TextVertexData Buffer::AddText(const std::string_view& text, auto& glyph_cache, double font_size) {
    uint32_t index = 0;
    glyphy::shader::TextVertexData result{};
    result.vertices.reserve(text.size() * 4);
    result.vertex_data.reserve(text.size() * 4);
    result.indices.reserve(text.size() * 6);
    std::string utf8_string{text};
    const char* utf8 = utf8_string.c_str();

    //Check for success, but need to verify race conditions for multithreaded buffer creation.  Perhaps a better way?
    //if (!hb_buffer_allocation_successful()) { throw std::runtime_error{}; }
    glyphy::harfbuzz::Buffer buffer{};

    glm::dvec2 top_left = cursor;
    cursor.y += font_size; // * font->ascent

    while (utf8) {
        const char *end = std::strchr(utf8, '\n');

        buffer.Clear();
        buffer.AddUTF8(utf8, end ? end - utf8 : -1, 0, -1);
        buffer.GuessSegmentProperties();
        buffer.Shape(glyph_cache.GetFontFace());

        std::vector<hb_glyph_info_t> infos = buffer.GlyphInfos();
        std::vector<hb_glyph_position_t> positions = buffer.GlyphPositions();
        for (auto [info, pos] : std::views::zip(infos, positions)) {
            auto glyph_index = info.codepoint;

            const GlyphInfo& glyph_info = glyph_cache.GetOrExtractGlyph(glyph_index);
            const double upem_d = static_cast<double>(glyph_info.upem);
            const double harfbuzz_offset_x = static_cast<double>(pos.x_offset) / upem_d; // scale to unit glyph
            const double harfbuzz_offset_y = static_cast<double>(pos.y_offset) / upem_d; // scale to unit glyph
            const double harfbuzz_advance_x = static_cast<double>(pos.x_advance) / upem_d; // scale to unit glyph
            const double harfbuzz_advance_y = static_cast<double>(pos.y_advance) / upem_d; // scale to unit glyph
            const glm::dvec2 position{
                cursor.x + (harfbuzz_offset_x + glyph_info.offsets.x) * font_size,
                cursor.y + (harfbuzz_offset_y + glyph_info.offsets.y) * font_size
            };
            const glm::dvec2 font_dims{
                glyph_info.dims.x * font_size,
                glyph_info.dims.y * font_size
            };

            if (glyph_info.num_units) {
                const auto encoded_vertices = glyphy::shader::EncodeVertex(position, font_dims);
                const auto encoded_vertex_data = glyphy::shader::EncodeVertexData(glyph_info.atlas_id,
                                                                                  glyph_info.num_units);

                // TODO: Once available, update to append_range
                result.vertices.push_back(encoded_vertices[0]);
                result.vertices.push_back(encoded_vertices[1]);
                result.vertices.push_back(encoded_vertices[2]);
                result.vertices.push_back(encoded_vertices[3]);

                result.vertex_data.push_back(encoded_vertex_data[0]);
                result.vertex_data.push_back(encoded_vertex_data[1]);
                result.vertex_data.push_back(encoded_vertex_data[2]);
                result.vertex_data.push_back(encoded_vertex_data[3]);

                result.indices.push_back(index);
                result.indices.push_back(index+1);
                result.indices.push_back(index+2);

                result.indices.push_back(index+1);
                result.indices.push_back(index+2);
                result.indices.push_back(index+3);

                index += 4;
            }

            const double advance_x = harfbuzz_advance_x * font_size;
            const double advance_y = harfbuzz_advance_y * font_size;

            cursor.x += advance_x;
        }

        if (end) {
            cursor.y += font_size;
            cursor.x = top_left.x;
            utf8 = end + 1;
        } else {
            utf8 = nullptr;
        }
    }

    return result;
}


}
