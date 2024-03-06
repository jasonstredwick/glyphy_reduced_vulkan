#pragma once


#include <concepts>
#include <type_traits>

#include <harfbuzz/hb.h>


/***
 * Code primarily extracted from Glyphy
 * https://github.com/behdad/glyphy/blob/master/src/glyphy-harfbuzz.h
 */


/***
 * Provide drawing hooks for Harfbuzz shaping.  DrawData can override default Harfbuzz draw functions if
 * the appropriate methods are provided.
 */


namespace glyphy::harfbuzz {


/***
 * Forward Declarations
 */
template <typename T, typename U> void FontGetGlyphShape(hb_font_t* font, hb_codepoint_t code_point, T* draw_data);


/***
 * Concepts
 */
template <typename T> concept ClosePath_c = requires(T t) { { t.ClosePath() }; };
template <typename T, typename U> concept CubicTo_c = requires(T t,
                                                               U control1_x, U control1_y,
                                                               U control2_x, U control2_y,
                                                               U x, U y)
    { { t.CubicTo(control1_x, control1_y, control2_x, control2_y, x, y) }; };
template <typename T, typename U> concept LineTo_c = requires(T t, U x, U y) { { t.LineTo(x, y) }; };
template <typename T, typename U> concept MoveTo_c = requires(T t, U x, U y) { { t.MoveTo(x, y) }; };
template <typename T, typename U> concept QuadraticTo_c = requires(T t, U control_x, U control_y, U x, U y)
    { { t.QuadraticTo(control_x, control_y, x, y) }; };


/***
 * Implementations
 */
template <typename T>
void ClosePathWrapper([[maybe_unused]] hb_draw_funcs_t* dfuncs,
                      void* v,
                      [[maybe_unused]] hb_draw_state_t* st,
                      [[maybe_unused]] void* user_data) {
    T* t = static_cast<T*>(v);
    t->ClosePath();
}


template <typename T, typename U>
requires std::is_floating_point_v<U> && std::is_convertible_v<float, U>
void CubicToWrapper([[maybe_unused]] hb_draw_funcs_t* dfuncs,
                    void* v,
                    [[maybe_unused]] hb_draw_state_t* st,
                    float control1_x,
                    float control1_y,
                    float control2_x,
                    float control2_y,
                    float to_x,
                    float to_y,
                    [[maybe_unused]] void* user_data) {
    T* t = static_cast<T*>(v);
    t->CubicTo(control1_x, control1_y, control2_x, control2_y, to_x, to_y);
}


template <typename T, typename U>
requires std::is_floating_point_v<U> && std::is_convertible_v<float, U>
void LineToWrapper([[maybe_unused]] hb_draw_funcs_t* dfuncs,
                   void* v,
                   [[maybe_unused]] hb_draw_state_t* st,
                   float to_x,
                   float to_y,
                   [[maybe_unused]] void* user_data) {
    T* t = static_cast<T*>(v);
    t->LineTo(to_x, to_y);
}


template <typename T, typename U>
requires std::is_floating_point_v<U> && std::is_convertible_v<float, U>
void MoveToWrapper([[maybe_unused]] hb_draw_funcs_t* dfuncs,
                   void* v,
                   [[maybe_unused]] hb_draw_state_t* draw_state,
                   float to_x,
                   float to_y,
                   [[maybe_unused]] void* user_data) {
    T* t = static_cast<T*>(v);
    t->MoveTo(to_x, to_y);
}


template <typename T, typename U>
requires std::is_floating_point_v<U> && std::is_convertible_v<float, U>
void QuadraticToWrapper([[maybe_unused]] hb_draw_funcs_t* dfuncs,
                        void* v,
                        [[maybe_unused]] hb_draw_state_t* st,
                        float control_x,
                        float control_y,
                        float to_x,
                        float to_y,
                        [[maybe_unused]] void* user_data) {
    T* t = static_cast<T*>(v);
    t->QuadraticTo(control_x, control_y, to_x, to_y);
}


template <typename T, typename U>
void FontGetGlyphShape(hb_font_t* font, hb_codepoint_t code_point, T* draw_data) {
    static hb_draw_funcs_t* dfuncs = nullptr;
    if (!dfuncs) {
        dfuncs = hb_draw_funcs_create();
        if constexpr (ClosePath_c<T>) {
            hb_draw_funcs_set_close_path_func(dfuncs, ClosePathWrapper<T>, nullptr, nullptr);
        }
        if constexpr (CubicTo_c<T, U>) {
            hb_draw_funcs_set_cubic_to_func(dfuncs, CubicToWrapper<T, U>, nullptr, nullptr);
        }
        if constexpr (LineTo_c<T, U>) {
            hb_draw_funcs_set_line_to_func(dfuncs, LineToWrapper<T, U>, nullptr, nullptr);
        }
        if constexpr (MoveTo_c<T, U>) {
            hb_draw_funcs_set_move_to_func(dfuncs, MoveToWrapper<T, U>, nullptr, nullptr);
        }
        if constexpr (QuadraticTo_c<T, U>) {
            hb_draw_funcs_set_quadratic_to_func(dfuncs, QuadraticToWrapper<T, U>, nullptr, nullptr);
        }
        hb_draw_funcs_make_immutable(dfuncs);
    }
    hb_font_get_glyph_shape(font, code_point, dfuncs, draw_data);
}


}
