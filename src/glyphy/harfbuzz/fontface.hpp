#pragma once


#include <stdexcept>
#include <utility>

#include <harfbuzz/hb.h>


namespace glyphy::harfbuzz {


class FontFace {
    hb_face_t* face{nullptr};
    hb_font_t* font{nullptr};

public:
    FontFace() noexcept = default;
    FontFace(hb_blob_t* blob, uint32_t typeface_index)
    : face{hb_face_create(blob, static_cast<unsigned int>(typeface_index))},
      font{hb_font_create(face)}
    {
        if (!face || !font) { throw std::runtime_error{"FontFace must be constructed with non-null pointers."}; }
    }
    FontFace(const FontFace&) = delete;
    FontFace(FontFace&& other) noexcept { *this = std::move(other); }
    ~FontFace() noexcept {
        if (font != nullptr) { hb_font_destroy(font); }
        if (face != nullptr) { hb_face_destroy(face); }
    }
    FontFace& operator=(const FontFace&) = delete;
    FontFace& operator=(FontFace&& other) noexcept {
        face = std::exchange(other.face, nullptr);
        font = std::exchange(other.font, nullptr);
        return *this;
    }

    const hb_face_t* Face() const noexcept { return face; }
          hb_face_t* Face()       noexcept { return face; }
    const hb_font_t* Font() const noexcept { return font; }
          hb_font_t* Font()       noexcept { return font; }
    unsigned int Upem() const noexcept { return hb_face_get_upem(face); }
};


}
