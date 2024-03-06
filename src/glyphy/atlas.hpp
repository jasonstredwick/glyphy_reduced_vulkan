#pragma once


#include <algorithm>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>


namespace glyphy {


template <typename ShaderUnit_t, size_t MAX_ID>
struct Atlas {
    using Unit_t = ShaderUnit_t;

    size_t available_index{0};
    size_t chunk_size{1};
    std::vector<Unit_t> buffer{};

    Atlas() noexcept = default;
    Atlas(size_t chunk_size) noexcept : chunk_size{chunk_size} {}
    Atlas(size_t chunk_size, size_t initial_reserve)
    : chunk_size{chunk_size}
    {
        const size_t chunk_by = this->chunk_size ? this->chunk_size : 1;
        const size_t new_capacity = ((initial_reserve / chunk_by) + (initial_reserve % chunk_by ? 1 : 0)) * chunk_by;
        buffer.reserve(new_capacity);
    }

    /***
     * Add data to the atlas and return the starting index of the data in the atlas as the id.  The data is stored
     * using a chunk size in order to allow for data alignment (default is 1: packed data).
     * *NOTE: If there is no data then MAX_ID is returned.
     * *NOTE: MAX_ID is not a valid id; similar to size being one past the end of an array.
     */
    size_t Bind(const std::vector<Unit_t>& data) {
        if (data.empty()) { return MAX_ID; }
        const size_t chunk_by = this->chunk_size ? this->chunk_size : 1;
        const size_t remaining = data.size() % chunk_by;
        const size_t num_alloc_units = ((data.size() / chunk_by) + (remaining ? 1 : 0)) * chunk_by;
        const size_t next_index = this->available_index + num_alloc_units;
        if (next_index >= MAX_ID) {
            throw std::runtime_error{"Allocation Error: Attempting to bind more data than the Atlas has available."};
        }
        if (next_index > buffer.capacity()) { buffer.reserve(next_index); }

        // TODO: When available convert to append_range
        // buffer.append_range(data);
        std::ranges::transform(data, std::back_inserter(buffer), [](const auto& e) { return e; });
        std::ranges::for_each(std::views::iota(static_cast<size_t>(0), remaining ? chunk_by - remaining : 0),
                              [&buffer=this->buffer](auto i) { buffer.push_back({}); });
        return std::exchange(this->available_index, next_index);
    }

    const auto& Buffer() const noexcept { return buffer; }

    static constexpr size_t MaxId() noexcept { return MAX_ID; }
};


}
