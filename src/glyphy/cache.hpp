#pragma once


#include <cstdint>
#include <map>
#include <tuple>

#include "glyphy/glm.hpp"

#include "glyphy/atlas.hpp"
#include "glyphy/glyph_data.hpp"
#include "glyphy/harfbuzz/fontface.hpp"


namespace glyphy {


struct alignas(64) GlyphCachedInfo {
    size_t atlas_id;
    size_t num_units;
    glm::dvec2 offsets{0.0, 0.0};
    glm::dvec2 dims{0.0, 0.0};
    double uniform_dim{0.0};
    uint32_t upem;
};


template <typename Atlas_t, typename ExtractGlyphFunc_t>
class GlyphCache {
    glyphy::harfbuzz::FontFace& fontface{nullptr};
    Atlas_t& atlas{nullptr};
    ExtractGlyphFunc_t& ExtractGlyph_f{nullptr};
    std::map<uint32_t, GlyphCachedInfo> glyph_info_cache{};

public:
    GlyphCache(glyphy::harfbuzz::FontFace& fontface, Atlas_t& atlas, ExtractGlyphFunc_t& F) noexcept
    : fontface{fontface}, atlas{atlas}, ExtractGlyph_f{F}
    {}
    GlyphCache(const GlyphCache&) = delete;
    GlyphCache(GlyphCache&&) = default;
    ~GlyphCache() noexcept = default;
    GlyphCache& operator=(const GlyphCache&) = delete;
    GlyphCache& operator=(GlyphCache&&) = default;

    glyphy::harfbuzz::FontFace& GetFontFace() const noexcept { return fontface; }

    const GlyphCachedInfo& GetOrExtractGlyph(const uint32_t glyph_index) {
        if (auto it = glyph_info_cache.find(glyph_index); it != glyph_info_cache.end()) { return it->second; }

        GlyphData<Atlas_t::Unit_t> glyph_data = ExtractGlyph_f(fontface, glyph_index);
        size_t atlas_id = atlas.Bind(glyph_data.atlas_data);
        glyph_info_cache[glyph_index] = {
            .atlas_id=atlas_id,
            .num_units=glyph_data.atlas_data.size(),
            .offsets=glyph_data.offsets,
            .dims=glyph_data.dims,
            .uniform_dim=glyph_data.uniform_dim,
            .upem=glyph_data.upem
        };

        return glyph_info_cache[glyph_index];
    }
};


}
