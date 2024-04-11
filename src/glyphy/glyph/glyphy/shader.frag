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
 * Google Author(s): Behdad Esfahbod, Maysum Panju
 */


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
layout(set=0, binding=2) readonly buffer GlyphyStateBlock { vec4 gsb_vals; };
layout(set=0, binding=3) readonly buffer EndpointBuffer { Endpoint_t endpoints[]; };
layout(location=0) out vec4 outColor;


#define GLYPHY_MAX_NUM_ENDPOINTS 32
#define GLYPHY_MAX_D 0.5

#ifndef GLYPHY_INFINITY
#  define GLYPHY_INFINITY 1e9
#endif
#ifndef GLYPHY_EPSILON
#  define GLYPHY_EPSILON  1e-5
#endif
#define SQRT2_2 0.70710678118654757 /* 1 / sqrt(2.) */


struct glyphy_arc_t {
  vec2  p0;
  vec2  p1;
  float d;
};


#ifdef GLYPHY_BGRA
vec4 GlyphyRGBA(const vec4 v) { return v.bgra; }
#else
vec4 GlyphyRGBA(const vec4 v) { return v.rgba; }
#endif


bool glyphy_isinf(const float v) { return abs(v) >= GLYPHY_INFINITY * 0.5; }
bool glyphy_iszero(const float v) { return abs(v) <= GLYPHY_EPSILON * 2.0; }
float glyphy_tan2atan(const float d) { return 2.0 * d / (1.0 - d * d); } // returns tan(2 * atan(d))
vec2 glyphy_ortho(const vec2 v) { return vec2(-v.y, v.x); }


vec2 glyphy_arc_center(const glyphy_arc_t a) {
  return mix(a.p0, a.p1, 0.5) + glyphy_ortho(a.p1 - a.p0) / (2.0 * glyphy_tan2atan(a.d));
}


bool GlyphyArcWedgeContains(const glyphy_arc_t a, const vec2 p) {
  float d2 = glyphy_tan2atan(a.d);
  return dot(p - a.p0, (a.p1 - a.p0) * mat2(1,  d2, -d2, 1)) >= 0.0 &&
	       dot(p - a.p1, (a.p1 - a.p0) * mat2(1, -d2,  d2, 1)) <= 0.0;
}


float GlyphyArcWedgeSignedDistShallow(const glyphy_arc_t a, const vec2 p) {
  vec2 v = normalize(a.p1 - a.p0);
  float line_d = dot(p - a.p0, glyphy_ortho(v));
  if (a.d == 0.0) { return line_d; }

  float d0 = dot((p - a.p0), v);
  if (d0 < 0.0) { return sign(line_d) * distance(p, a.p0); }
  float d1 = dot((a.p1 - p), v);
  if (d1 < 0.0) { return sign(line_d) * distance(p, a.p1); }
  float r = 2.0 * a.d * (d0 * d1) / (d0 + d1);
  if (r * line_d > 0.0) { return sign(line_d) * min(abs(line_d + r), min(distance(p, a.p0), distance(p, a.p1))); }
  return line_d + r;
}


float GlyphyArcWedgeSignedDist(const glyphy_arc_t a, const vec2 p) {
  if (abs(a.d) <= 0.03) { return GlyphyArcWedgeSignedDistShallow(a, p); }
  vec2 c = glyphy_arc_center(a);
  return sign(a.d) * (distance(a.p0, c) - distance(p, c));
}


float GlyphyArcExtendedDist(const glyphy_arc_t a, const vec2 p) {
  /* Note: this doesn't handle points inside the wedge. */
  vec2 m = mix(a.p0, a.p1, 0.5);
  float d2 = glyphy_tan2atan(a.d);
  if (dot(p - m, a.p1 - m) < 0.0) { return dot(p - a.p0, normalize((a.p1 - a.p0) * mat2(+d2, -1, +1, +d2))); }
  else                            { return dot(p - a.p1, normalize((a.p1 - a.p0) * mat2(-d2, -1, +1, -d2))); }
}


int GlyphyArcListOffset(const vec2 p, const ivec2 nominal_size) {
  ivec2 cell = ivec2(clamp(floor(p), vec2(0.0, 0.0), vec2(nominal_size - 1)));
  return cell.y * nominal_size.x + cell.x;
}


float SDF(int endpoint_start_index, int num_endpoints, float start_side, vec2 target) {
  float side = start_side;
  float min_dist = GLYPHY_INFINITY;

  /* Short-circuits */
  if (num_endpoints == 0) {
    /* far-away cell */
    return GLYPHY_INFINITY * side;
  }

  glyphy_arc_t closest_arc;
  Endpoint_t endpoint = endpoints[endpoint_start_index];
  vec2 prev_p = endpoint.p;

  for (int i=1; i<GLYPHY_MAX_NUM_ENDPOINTS; i++) {
    if (i >= num_endpoints) { break; }

    endpoint = endpoints[endpoint_start_index + i];
    if (glyphy_isinf(endpoint.d)) {
      prev_p = endpoint.p;
      continue;
    }

    glyphy_arc_t a = glyphy_arc_t(prev_p, endpoint.p, endpoint.d);
    if (GlyphyArcWedgeContains(a, target)) {
      float sdist = GlyphyArcWedgeSignedDist(a, target);
      float udist = abs(sdist) * (1.0 - GLYPHY_EPSILON);
      if (udist <= min_dist) {
        min_dist = udist;
        side = sdist <= 0.0 ? -1.0 : +1.0;
      }
    } else {
      float udist = min(distance(target, a.p0), distance(target, a.p1));
      if (udist < min_dist - GLYPHY_EPSILON) {
        min_dist = udist;
        side = 0.0; // unsure
        closest_arc = a;
      } else if (side == 0.0 && udist - min_dist <= GLYPHY_EPSILON) {
        // If this new distance is the same as the current minimum,
        // compare extended distances.  Take the sign from the arc
        // with larger extended distance.
        float old_ext_dist = GlyphyArcExtendedDist(closest_arc, target);
        float new_ext_dist = GlyphyArcExtendedDist(a, target);
        float ext_dist = abs(new_ext_dist) <= abs(old_ext_dist) ? old_ext_dist : new_ext_dist;

        // For emboldening and stuff:
        min_dist = abs(ext_dist);
        side = sign(ext_dist);
      }
    }
    prev_p = endpoint.p;
  }

  if (side == 0.0) {
    // Technically speaking this should not happen, but it does.  So try to fix it.
    float ext_dist = GlyphyArcExtendedDist(closest_arc, target);
    side = ext_dist >= 0 ? +1.0 : -1.0;
  }

  return min_dist * side;
}


float antialias(float d) {
  return smoothstep(-0.75, +0.75, d);
}


void main() {
  /* isotropic antialiasing */
  vec2 dpdx = dFdx(uv);
  vec2 dpdy = dFdy(uv);
  float mag = length(vec2(length(dpdx), length(dpdy))) * SQRT2_2;

  float u_contrast = gsb_vals[0];
  float u_gamma_adjust = gsb_vals[1];
  float u_outline_thickness = gsb_vals[2];
  float u_boldness = gsb_vals[3];
  bool u_outline = u_outline_thickness <= 0.0;

  int endpoint_start_index = int(atlas_info[0] + 0.5);
  int num_endpoints = int(atlas_info[1] + 0.5);
  float start_side = atlas_info[2];
  float gsdist = SDF(endpoint_start_index, num_endpoints, start_side, uv);
  gsdist -= u_boldness;
  float sdist = gsdist / mag * u_contrast;
  
  vec4 color = vec4(1, 1, 1, 1);
  if (u_outline) { sdist = abs(sdist) - u_outline_thickness * 0.5; }
  if (sdist > 1.0) { discard; }
  float alpha = antialias(-sdist);
  if (u_gamma_adjust != 1.0) { alpha = pow(alpha, 1.0 / u_gamma_adjust); }
  color = vec4(color.rgb,color.a * alpha);

  outColor = color;
}
