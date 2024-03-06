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
#define GLYPHY_SQRT2_2 0.70710678118654757 // 1 / sqrt(2)


float Cross(const vec2 lhs, const vec2 rhs) { return lhs.x * rhs.y - lhs.y * rhs.x; }
vec2 Ortho(const vec2 v) { return vec2(-v.y, v.x); }
bool IsInf(const float v) { return abs(v) >= GLYPHY_INFINITY * 0.5; }
bool IsZero(const float v) { return abs(v) <= GLYPHY_EPSILON * 2.0; }
float tan2atan(float d)  { return 2.0 * d / (1.0 - d*d); }     /* returns tan(2 * atan(d)) */


float DistanceToLine(const vec2 p0, const vec2 p1, const vec2 target) {
    // https://iquilezles.org/articles/distfunctions2d/
    vec2 v01 = p1 - p0;
    vec2 v0t = target - p0;
    float h = dot(v01, v0t) / dot(v01, v01);
    float is_inside = float(h >= 0 && h <= 1.0);
    return is_inside * length(v0t - v01 * h) + (1.0 - is_inside) * GLYPHY_INFINITY;
}


float DistanceToArc(const vec2 p0, const vec2 p1, float d, const vec2 target) {
    // tan(t) == tan(t); tan(-t) == -tan(t)
    // sign on d will propagate out to reverse the perpendicular vector clockwise if negative.
    vec2 v = (p1 - p0) * 0.5;
    vec2 vp = Ortho(v) * tan2atan(d);
    vec2 center = p0 + v + vp;

    vec2 vc0 = p0 - center;
    vec2 vct = target - center;
    float dist = abs(length(vct) - length(vc0));

    float is_inside = float(dot(vc0, -vp) <= dot(vct, -vp));
    return is_inside * dist + (1.0 - is_inside) * GLYPHY_INFINITY;
}


float SDF(int endpoint_start_index, int num_endpoints, float start_side_side, vec2 target) {
    float min_dist = GLYPHY_INFINITY;

    Endpoint_t endpoint = endpoints[endpoint_start_index];
    vec2 prev_p = endpoint.p;
    for (int i=1; i<num_endpoints; ++i, prev_p=endpoint.p) {
        endpoint = endpoints[endpoint_start_index + i];
        if (IsInf(endpoint.d)) { continue; }

        float value = GLYPHY_INFINITY;
        if (IsZero(endpoint.d)) {
            value = DistanceToLine(prev_p, endpoint.p, target);
        } else {
            value = DistanceToArc(prev_p, endpoint.p, endpoint.d, target);
        }
        float is_less = float(value < min_dist);
        min_dist = is_less * value + (1.0 - is_less) * min_dist;
    }

    return min_dist;
}


/***
 * main
 */
void main() {
    float u_contrast = gsb_vals[0];
    float u_gamma_adjust = gsb_vals[1];
    float u_outline_thickness = gsb_vals[2];
    float u_boldness = gsb_vals[3];
    bool u_outline = u_outline_thickness < 0.0;


    /* isotropic antialiasing */
    //vec2 dpdx = dFdx(uv);
    //vec2 dpdy = dFdy(uv);
    //float mag = length(vec2(length(dpdx), length(dpdy))) * GLYPHY_SQRT2_2;

    int endpoint_start_index = int(atlas_info[0]);
    int num_endpoints = int(atlas_info[1]);
    float start_side_side = atlas_info[2];
    float gsdist = SDF(endpoint_start_index, num_endpoints, start_side_side, uv);

    //gsdist -= u_boldness;
    //float sdist = gsdist / mag * u_contrast;

    #ifdef GLYPHY_DEBUG
    //outColor = DebugDraw(p, sdist, gsdist, glyph_info.nominal_size, u_atlas_tex, u_atlas_info, glyph_info.atlas_pos);
    //return;
    #endif

    vec4 color = vec4(0, 0, 0, 1);
    //if (u_outline) { sdist = abs(sdist) - u_outline_thickness * 0.5; }
    //if (sdist > 1.0f) { sdist = 1.0f; }//  discard; }
    //float alpha = smoothstep(-0.75, +0.75, -sdist);
    //if (u_gamma_adjust != 1.0f) { alpha = pow(alpha, 1.0 / u_gamma_adjust); }
    //color = vec4(color.rgb, color.a * alpha);
    //color = vec4(vec3(1.0 * float(ee.p.x >= 0.155762 && ee.p.x < 0.155762), 0, 0), 1);
    //color = vec4(vec3(float(endpoint_start_index) / float(num_endpoints) , 0, 0), 1);


    //Endpoint_t ee = endpoints[13];
    //color = vec4(vec3(ee.p.x, ee.p.y, ee.d), 1);
    //if (gsdist > 0) {// discard; }
    if (gsdist < 0.015) {
        color = vec4(1.0, 1.0, 1.0, 1.0);
    } else {
        discard; //color = vec4(1.0, 0.0, 0.0, 1.0);
    }

    //color = vec4(vec3(1.0f) * (color.a * alpha), 1.0f);
    //vec4 rgba = texture(u_atlas_tex, glyph_info.position);
    //vec3 x = vec3(0.0f);
    //if (isnan(sdist)) { x = vec3(1.0f); }
    //color = vec4(x, 1.0f);
    //color = vec4(rgba.xyz, 1.0f);//vec4(x, 1.0f);
    outColor = color;
}





#if 0
float Cross(const vec2 lhs, const vec2 rhs) { return lhs.x * rhs.y - lhs.y * rhs.x; }
vec2 Midpoint(const vec2 a, const vec2 b) { return (a + b) / 2.0; }
vec2 OrthoC(const vec2 v) { return {v.y, -v.x}; }
float cos2atan(float d) { return (1.0 - d*d) / (1.0 + d*d); } /* returns cos(2 * atan(d)) */
float sin2atan(float d) { return 2.0 * d / (1.0 + d*d); }     /* returns sin(2 * atan(d)) */


vec2 Center(const vec2 p0, const vec2 p1, float d) {
    vec2 pm = Midpoint()
    return pm + Ortho(pm - p0) * tan2atan(d);
}


bool Glyphy_ArcWedgeContains(const Arc_t a, const vec2 p) {
    float d2 = Glyphy_Tan2Atan(a.d);
    return dot(p - a.p0, (a.p1 - a.p0) * mat2(1,  d2, -d2, 1)) >= 0.0 &&
           dot(p - a.p1, (a.p1 - a.p0) * mat2(1, -d2,  d2, 1)) <= 0.0;
}


float Glyphy_ArcWedgeSignedDistShallow(const Arc_t a, const vec2 p) {
    vec2 v = normalize(a.p1 - a.p0);
    float line_d = dot(p - a.p0, Glyphy_Ortho(v));
    if (a.d == 0.0) { return line_d; }

    float d0 = dot((p - a.p0), v);
    if (d0 < 0.0) { return sign(line_d) * distance(p, a.p0); }
    float d1 = dot((a.p1 - p), v);
    if (d1 < 0.0) { return sign(line_d) * distance(p, a.p1); }
    float r = 2.0 * a.d * (d0 * d1) / (d0 + d1);
    if (r * line_d > 0.0) { return sign(line_d) * min(abs(line_d + r), min(distance(p, a.p0), distance(p, a.p1))); }
    return line_d + r;
}


float Glyphy_ArcWedgeSignedDist(const Arc_t a, const vec2 p) {
    if (abs(a.d) <= 0.03) { return Glyphy_ArcWedgeSignedDistShallow(a, p); }
    vec2 c = Glyphy_ArcCenter(a);
    return sign(a.d) * (distance(a.p0, c) - distance(p, c));
}


float Glyphy_ArcExtendedDist(const Arc_t a, const vec2 p) {
    /* Note: this doesn't handle points inside the wedge. */
    vec2 m = mix(a.p0, a.p1, .5);
    float d2 = Glyphy_Tan2Atan(a.d);
    if (dot(p - m, a.p1 - m) < 0.0) { return dot(p - a.p0, normalize((a.p1 - a.p0) * mat2(+d2, -1, +1, +d2))); }
    else                            { return dot(p - a.p1, normalize((a.p1 - a.p0) * mat2(-d2, -1, +1, -d2))); }
}


float Glyphy_SDF(int endpoint_start_index, int num_endpoints, float start_side_side, vec2 target) {
    float side = start_side_side;
    float min_dist = GLYPHY_INFINITY;
    Arc_t closest_arc;

    Endpoint_t endpoint = endpoints[endpoint_start_index];
    vec2 prev_endpoint = endpoint.p;
    for (int i = 1; i < num_endpoints; i++) {
        endpoint = endpoints[endpoint_start_index + i];
        Arc_t a = Arc_t(prev_endpoint, endpoint.p, endpoint.d);
        prev_endpoint = endpoint.p;
        if (IsInf(a.d)) { continue; }

        if (Glyphy_ArcWedgeContains(a, target)) {
            float sdist = Glyphy_ArcWedgeSignedDist(a, target);
            float udist = abs(sdist) * (1.0 - GLYPHY_EPSILON);
            if (udist <= min_dist) {
                min_dist = udist;
                side = sdist <= 0.0 ? -1.0 : +1.0;
            }
        } else {
            float udist = min(distance(target, a.p0), distance(target, a.p1));
            if (udist < min_dist - GLYPHY_EPSILON) {
            min_dist = udist;
            side = 0.0; /* unsure */
            closest_arc = a;
            } else if (side == 0.0 && udist - min_dist <= GLYPHY_EPSILON) {
            /* If this new distance is the same as the current minimum,
            * compare extended distances.  Take the sign from the arc
            * with larger extended distance. */
            float old_ext_dist = Glyphy_ArcExtendedDist(closest_arc, target);
            float new_ext_dist = Glyphy_ArcExtendedDist(a, target);
            float ext_dist = abs(new_ext_dist) <= abs(old_ext_dist) ? old_ext_dist : new_ext_dist;

#ifdef GLYPHY_SDF_PSEUDO_DISTANCE
            /* For emboldening and stuff: */
            min_dist = abs(ext_dist);
#endif
            side = sign(ext_dist);
            }
        }
    }

    if (side == 0.0) {
        // Technically speaking this should not happen, but it does.  So try to fix it.
        float ext_dist = Glyphy_ArcExtendedDist(closest_arc, target);
        side = sign(ext_dist);
    }

    return min_dist * side;
}
#endif
