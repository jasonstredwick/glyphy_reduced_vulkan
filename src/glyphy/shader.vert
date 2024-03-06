#version 460
#extension GL_EXT_scalar_block_layout : enable


struct ModelInfo {
    mat4 transform;
};


struct VertexData {
    uint model_data_index;
};


layout(location=0) in vec4 pos;
layout(location=1) in vec4 altas_info_in;
layout(location=2) in vec2 corners;
layout(location=3) in vec2 nominal_dims;

layout(scalar, set=0, binding=0) readonly buffer MVP { mat4 mvp; };
layout(scalar, set=0, binding=1) readonly buffer ModelTransformBuffer { ModelInfo model_info[]; };

layout(location=0) out vec4 atlas_info;
layout(location=1) out vec2 uv;
layout(location=2) out vec2 dims;


void main() {
    atlas_info = altas_info_in;
    uv = corners;
    dims = nominal_dims;
    gl_Position = mvp * vec4(pos.x, pos.y, pos.z, 1.0);
}
