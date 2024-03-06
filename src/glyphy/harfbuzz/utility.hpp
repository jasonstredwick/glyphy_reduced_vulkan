#pragma once


#include <ranges>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>


namespace glyphy::harfbuzz {


/***
 * Harfbuzz utilizes C like functions to extract collections of data from opaque structures. These functions take a
 * higher level opaque object and returns a pointer to a collection for the requested information.  The function also
 * takes a pointer of unsigned int that is used to return the count of objects returned.
 *
 * High level pseudocode declaration for these functions-
 *
 * template <typename T, typename U>
 * U* GetU(T* t, unsigned int* num_U_objs_returned);
*/


constexpr auto AsSpan(auto* container, auto&& F) {
    using T = std::remove_pointer_t<decltype(F(container, std::declval<unsigned int*>()))>;
    unsigned int count = 0;
    auto* result = F(container, &count);
    return std::span<T>{result, static_cast<size_t>(count)};
}


constexpr auto AsVector(auto* container, auto&& F) {
    using T = std::remove_pointer_t<decltype(F(container, std::declval<unsigned int*>()))>;
    return AsSpan(container, F) | std::ranges::to<std::vector<T>>();
}


}