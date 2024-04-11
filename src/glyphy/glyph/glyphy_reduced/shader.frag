#version 460
#extension GL_EXT_scalar_block_layout : enable


struct Endpoint_t {
    vec2 p;
    float d;
    float dummy;
};


layout(location=0) in vec4 atlas_info; // unique to quad; no interpolation because same value for all vertices
layout(location=1) in vec2 uv;   // interpolated position of the quad corners (0,0) .. (1,1)
layout(location=2) in vec2 dims; // unique to quad; no interpolation because same value for all vertices

//layout(        set=1, binding=0) uniform sampler2D u_atlas_tex;
//layout(set=0, binding=1) readonly buffer AtlasInfo { ivec4 u_atlas_info; }; // atlas texture info {width, height, unit_width, unit_h}
layout(set=0, binding=2) readonly buffer GlyphyStateBlock {
    vec4 gsb_vals;
};
layout(set=0, binding=3) readonly buffer EndpointBuffer { Endpoint_t endpoints[]; };
layout(location=0) out vec4 outColor;


#define GLYPHY_MAX_NUM_ENDPOINTS 32
#define GLYPHY_INFINITY 1e9
#define GLYPHY_EPSILON  1e-5
#define GLYPHY_SQRT2_2 0.70710678118654757 // sqrt(2) / 2
#define HALF_PI = 1.5707964;
#define PI = 3.1415927


float Cross(const vec2 lhs, const vec2 rhs) { return lhs.x * rhs.y - lhs.y * rhs.x; }
vec2 Ortho(const vec2 v) { return vec2(-v.y, v.x); }
bool IsInf(const float v) { return abs(v) >= GLYPHY_INFINITY * 0.5; }
bool IsZero(const float v) { return abs(v) <= GLYPHY_EPSILON * 2.0; }
// half angle formula
float cos2atan(const float d) { return (1.0 - d*d) / (1.0 + d*d); } // returns cos(2 * atan(d))
float sin2atan(const float d) { return 2.0 * d / (1.0 + d*d); }     // returns sin(2 * atan(d))
float tan2atan(const float d) { return 2.0 * d / (1.0 - d*d); }     // returns tan(2 * atan(d))
// trig/inverse trig relationships (squared values)
float cosatan_sq(const float d) { return 1.0 / (1.0 + d*d); } // non-squared: 1 / sqrt(1 + d*d)
float sinatan_sq(const float d) { return d*d / (1.0 + d*d); } // non-squared: d / sqrt(1 + d*d)
float tanatan_sq(const float d) { return d*d; } // non-squared: d


vec2 Midpoint(const vec2 a, const vec2 b) { return (a + b) / 2.0; }
vec2 Center(const vec2 p0, const vec2 p1, const float d) {
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    vec2 pm = Midpoint(p0, p1);
    return pm + Ortho(pm - p0) * tan2atan(d);
}


// Returns the shortest distance squared and updated side value for the target to the segment.
float DistanceSqToSegment(const vec2 v_01, const vec2 v_0p) {
    // https://iquilezles.org/articles/distfunctions2d/
    // See sdPolygon and sdSegment
    vec2 v_target = v_0p - v_01 * clamp(dot(v_0p, v_01) / dot(v_01, v_01), 0.0, 1.0);
    return dot(v_target, v_target);
}


float DistanceToArc(const vec2 p0, const vec2 p1, float d, const vec2 target) {
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    float tan_beta = tan2atan(d);
    vec2 pm = Midpoint(p0, p1);
    vec2 v_mc = Ortho(pm - p0) * tan_beta;
    vec2 center = pm + v_mc;
    vec2 v_ct = target - center;
    vec2 v_c0 = p0 - center;
    float radius_sq = dot(v_c0, v_c0);
    float t_dist_sq = dot(v_ct, v_ct);
    if (dot(target - p0, (p1 - p0) * mat2(1,  tan_beta, -tan_beta, 1)) >= 0.0 &&
	    dot(target - p1, (p1 - p0) * mat2(1, -tan_beta,  tan_beta, 1)) <= 0.0) {
        return t_dist_sq + radius_sq - 2.0 * sqrt(t_dist_sq * radius_sq);
    }
    return GLYPHY_INFINITY;
}


float SDF(const int start_index, const int num_endpoints, const vec2 target) {
    float dist_sq = GLYPHY_INFINITY;
    float side = 1.0;

    int i = start_index;
    int j = i + 1;
    int N = start_index + num_endpoints;
    for (; j<N; ++i, ++j) {
        if (IsInf(endpoints[j].d)) { continue; }

        vec2 p0 = endpoints[i].p;
        vec2 p1 = endpoints[j].p;
        float d = endpoints[j].d;

        vec2 v_01 = p1 - p0;
        vec2 v_0p = target - p0;

        // Winding number test; using a horizontal line through target determine the number of crossings
        // only looking at those segments that cross the line.  Crossing is affected by the direction of the crossing
        // and utilizes a boolean test of cross product to get the sign rather than computing the actual cross
        // product.
        bvec3 cond = bvec3(p0.y <= target.y, target.y < p1.y, v_01.x * v_0p.y > v_01.y * v_0p.x);
        if (all(cond) || all(not(cond))) { side = -side; }

        // line
        float value = GLYPHY_INFINITY;
        if (d == 0) {
            value = DistanceSqToSegment(v_01, v_0p);
        } else {
            value = DistanceToArc(p0, p1, d, target);
        }
        dist_sq = min(dist_sq, value);
    }

    return side * sqrt(dist_sq);
}


void main() {
    /* isotropic antialiasing */
    vec2 dpdx = dFdx(uv);
    vec2 dpdy = dFdy(uv);
    float mag = length(vec2(length(dpdx), length(dpdy))) * GLYPHY_SQRT2_2;

    float u_contrast = gsb_vals[0];
    float u_gamma_adjust = gsb_vals[1];
    float u_outline_thickness = gsb_vals[2];
    float u_boldness = gsb_vals[3];
    bool u_outline = u_outline_thickness < 0.0;

    int endpoint_start_index = int(atlas_info[0] + 0.5); // not certain why I need the +0.5; why not exact?
    int num_endpoints = int(atlas_info[1] + 0.5);  // not certain why I need the -0.5; why not exact?

    float gsdist = SDF(endpoint_start_index, num_endpoints, uv);
    //if (abs(gsdist) > 0.01) { discard; }
    //outColor = vec4(1, 1, 1, 1);

#if 1
    gsdist -= u_boldness;
    float sdist = gsdist / mag * u_contrast;

    vec4 color = vec4(1, 1, 1, 1);
    if (u_outline) { sdist = abs(sdist) - u_outline_thickness * 0.5; }
    if (sdist > 0) { discard; }
    float alpha = smoothstep(-0.75, +0.75, -sdist);
    if (u_gamma_adjust != 1.0f) { alpha = pow(alpha, 1.0 / u_gamma_adjust); }
    color = vec4(color.rgb, color.a * alpha);

    outColor = color;
#endif
}
