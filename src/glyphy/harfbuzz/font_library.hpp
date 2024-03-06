#pragma once


#include <cstdint>
#include <filesystem>
#include <format>
#include <map>
#include <memory>
#include <ranges>
#include <stdexcept>

#include <harfbuzz/hb.h>

#include "glyphy/harfbuzz/fontface.hpp"


namespace glyphy::harfbuzz {


class FontLibrary {
    uint32_t available_id = 1;
    std::map<uint32_t, std::filesystem::path> font_id_path_map{};
    std::map<std::filesystem::path, uint32_t> font_path_id_map{};
    std::map<uint32_t, std::unique_ptr<hb_blob_t, decltype(&hb_blob_destroy)>> blob_cache{};
    std::map<uint32_t, uint32_t> face_count_map{};
    std::map<uint64_t, FontFace> fontface_cache{};

    constexpr uint64_t ToFontFaceId(const uint32_t font_id, const uint32_t typeface_index) const noexcept {
        return (static_cast<uint64_t>(font_id) << 32) + (static_cast<uint64_t>(typeface_index) & 0xffffffff);
    }

public:
    // Can't detect current reference counts, so just going to clear caches and let the current fonts exist in the
    // wild until they become unused.  All new calls to get a font will create new cached information.
    void ClearCaches() {
        face_count_map.clear();
        fontface_cache.clear();
        blob_cache.clear();
    }

    void ClearFontCache(const uint32_t font_id, const uint32_t typeface_index) {
        auto count_it = face_count_map.find(font_id);
        if (count_it == face_count_map.end()) { return; }

        uint32_t count = count_it->second;
        if (typeface_index > count) { return; }

        const uint64_t font_face_id = ToFontFaceId(font_id, typeface_index);
        auto fontface_it = fontface_cache.find(font_face_id);
        if (fontface_it != fontface_cache.end()) { fontface_cache.erase(fontface_it); }
    }

    void ClearFontCaches(const uint32_t font_id) {
        auto count_it = face_count_map.find(font_id);
        if (count_it == face_count_map.end()) { return; }
        uint32_t count = count_it->second;

        for (uint32_t typeface_index : std::views::iota(static_cast<uint32_t>(0), count)) {
            const uint64_t font_face_id = ToFontFaceId(font_id, typeface_index);
            auto fontface_it = fontface_cache.find(font_face_id);
            if (fontface_it != fontface_cache.end()) { fontface_cache.erase(fontface_it); }
        }
    }

    std::filesystem::path GetFontPath(const uint32_t font_id) const noexcept {
        if (auto iter = font_id_path_map.find(font_id); iter != font_id_path_map.end()) { return iter->second; }
        return {};
    }

    FontFace& GetOrCreateFont(const uint32_t font_id, uint32_t typeface_index=0) {
        const uint64_t fontface_id = ToFontFaceId(font_id, typeface_index);

        if (auto iter = fontface_cache.find(fontface_id); iter != fontface_cache.end()) { return iter->second; }

        auto blob_it = blob_cache.find(font_id);
        if (blob_it == blob_cache.end()) {
            throw std::runtime_error{std::format("Font (id={}) not registered.", font_id)};
        } else if (blob_it->second.get() == nullptr) { // if the blob pointer is null then load it
            auto path_it = font_id_path_map.find(font_id);
            const auto& path = path_it->second;
            if (path.empty() || !std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
                throw std::runtime_error{
                    std::format("Failed to load font file ({}); file missing.", path.generic_string())};
            }
            blob_it->second.reset(hb_blob_create_from_file_or_fail(path.generic_string().c_str()));
            if (blob_it->second.get() == nullptr) {
                throw std::runtime_error{std::format("Failed to read font file ({})", path.generic_string())};
            }
            uint32_t num_faces = static_cast<uint32_t>(hb_face_count(blob_it->second.get()));
            if (!num_faces) { ++num_faces; }
            face_count_map[font_id] = num_faces;
        }

        auto face_count = face_count_map.at(font_id);
        if (typeface_index >= face_count) {
            throw std::runtime_error{std::format("Invalid typeface index; {} >= {}", typeface_index, face_count)};
        }

        auto emplace_it = fontface_cache.emplace(fontface_id, FontFace{blob_it->second.get(), typeface_index});
        if (emplace_it.second == false) {
            throw std::runtime_error{std::format("Failed to cache font ({}) face ({}) !", font_id, typeface_index)};
        }
        return emplace_it.first->second;
    }

    uint32_t GetOrInsertId(const std::filesystem::path& path) {
        if (path.empty() || !std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
            throw std::runtime_error{std::format("Failed to get id for font file ({}); empty or invalid file provided.",
                                                 path.generic_string())};
        }
        const std::filesystem::path abs_path = std::filesystem::absolute(path);

        if (auto iter = font_path_id_map.find(abs_path); iter != font_path_id_map.end()) { return iter->second; }
        uint32_t id = available_id++; // get then update
        font_id_path_map.emplace(id, abs_path);
        font_path_id_map.emplace(abs_path, id);
        blob_cache.emplace(id, typename decltype(blob_cache)::mapped_type{nullptr, &hb_blob_destroy});
        return id;
    }
};


}
