#pragma once


#include <array>
#include <cstdint>
#include <limits>
#include <string_view>
#include <type_traits>
#include <vector>

#include "glyphy/glm.hpp"

#include "jms/vulkan/vulkan.hpp"
#include "jms/vulkan/graphics_pass.hpp"
#include "jms/vulkan/shader.hpp"


namespace glyphy::shader {


/***
 * Constants
 */
constexpr const float GLYPHY_EPSILON = 1e-5f;
constexpr const float GLYPHY_INFINITY = 1e9f;


/***
 * GLSL compatible structures.
 */
struct GlyphInfo {
    glm::vec4 dim{0.0f, 0.0f, 0.0f, 0.0f};
#if 0
            float u_contrast{1.0f};
            float u_gamma_adjust{1.0f};
            float u_outline_thickness{1.0f};
            //bool  u_outline{false};
            float u_boldness{0.0f};
            //bool  u_debug{false};
#endif
};


struct MVP {
    glm::mat4 mvp{1.0f};
};


struct ModelInfo {
    glm::mat4 transform{1.0f};
};


struct Vertex {
    glm::vec4 pos{0.0f, 0.0f, 0.0f, 0.0f};
};


struct VertexData {
    glm::vec4 atlas_info{0.0f, 0.0f, 0.0f, 0.0f};
    glm::vec2 corners{0.0f, 0.0f};
    glm::vec2 nominal_dims{0.0f, 0.0f};
};


struct TextVertexData {
    std::vector<Vertex> vertices{};
    std::vector<VertexData> vertex_data{};
    std::vector<uint32_t> indices{};
};


std::array<Vertex, 4> EncodeVertex(const glm::dvec2& position, const glm::dvec2& dims) {
    glm::vec4 p{static_cast<float>(position.x), 0.0f, static_cast<float>(position.y), 1.0f};
    float size_w = static_cast<float>(dims.x);
    float size_h = static_cast<float>(dims.y);
    return {
        Vertex{.pos={p + glm::vec4(0.0f,   10.0f, 0.0f,   0.0f)}},
        Vertex{.pos={p + glm::vec4(0.0f,   10.0f, size_h, 0.0f)}},
        Vertex{.pos={p + glm::vec4(size_w, 10.0f, 0.0f,   0.0f)}},
        Vertex{.pos={p + glm::vec4(size_w, 10.0f, size_h, 0.0f)}}
    };
}


std::array<VertexData, 4> EncodeVertexData(const size_t atlas_index, const size_t num_endpoints) {
    float ai = static_cast<float>(static_cast<uint32_t>(atlas_index));
    float num = static_cast<float>(static_cast<uint32_t>(num_endpoints));
    return {
        VertexData{.atlas_info={ai, num, 0.0f, 0.0f}, .corners=glm::vec2(0.0f, 0.0f)},
        VertexData{.atlas_info={ai, num, 0.0f, 0.0f}, .corners=glm::vec2(0.0f, 1.0f)},
        VertexData{.atlas_info={ai, num, 0.0f, 0.0f}, .corners=glm::vec2(1.0f, 0.0f)},
        VertexData{.atlas_info={ai, num, 0.0f, 0.0f}, .corners=glm::vec2(1.0f, 1.0f)}
    };
}


/***
 * Shader script management and data descriptions
 */
jms::vulkan::ShaderGroup CreateGroup(const std::string_view& frag_shader_path) {
    return jms::vulkan::ShaderGroup{
        .vertex_attribute_desc={
            {
                .location=0,
                .binding=0,
                .format=vk::Format::eR32G32B32A32Sfloat,
                .offset=offsetof(Vertex, pos)
            },
            {
                .location=1,
                .binding=1,
                .format=vk::Format::eR32G32B32A32Sfloat,
                .offset=offsetof(VertexData, atlas_info)
            },
            {
                .location=2,
                .binding=1,
                .format=vk::Format::eR32G32Sfloat,
                .offset=offsetof(VertexData, corners)
            },
            {
                .location=3,
                .binding=1,
                .format=vk::Format::eR32G32Sfloat,
                .offset=offsetof(VertexData, nominal_dims)
            }
        },

        .vertex_binding_desc={
            {   // Vertex
                .binding=0,
                .stride=sizeof(Vertex),
                .inputRate=vk::VertexInputRate::eVertex,
                .divisor=1
            },
            {   // VertexData
                .binding=1,
                .stride=sizeof(VertexData),
                .inputRate=vk::VertexInputRate::eVertex,
                .divisor=1
            }
        },

        .push_constant_ranges={},

        .set_layout_bindings={
            {
                {   // MVP
                    .binding=0,
                    .descriptorType=vk::DescriptorType::eStorageBuffer,
                    .descriptorCount=1,
                    .stageFlags=vk::ShaderStageFlagBits::eVertex,
                    .pImmutableSamplers=nullptr
                },
                {   // ModelData
                    .binding=1,
                    .descriptorType=vk::DescriptorType::eStorageBuffer,
                    .descriptorCount=1,
                    .stageFlags=vk::ShaderStageFlagBits::eVertex,
                    .pImmutableSamplers=nullptr
                },
                {   // GlyphInfo
                    .binding=2,
                    .descriptorType=vk::DescriptorType::eStorageBuffer,
                    .descriptorCount=1,
                    .stageFlags=vk::ShaderStageFlagBits::eFragment,
                    .pImmutableSamplers=nullptr
                },
                {   // Atlas
                    .binding=3,
                    .descriptorType=vk::DescriptorType::eStorageBuffer,
                    .descriptorCount=1,
                    .stageFlags=vk::ShaderStageFlagBits::eFragment,
                    .pImmutableSamplers=nullptr
                }
            }
        },

        // need to investigate vector initialization with aggregate Info initialization with forbidden copy constructor.
        .shader_infos={
            {
                .flags=vk::ShaderCreateFlagBitsEXT::eLinkStage,
                .stage=vk::ShaderStageFlagBits::eVertex,
                .next_stage=vk::ShaderStageFlagBits::eFragment,
                .code_type=vk::ShaderCodeTypeEXT::eSpirv,
                .code=jms::vulkan::Load(std::string{"shader.vert.spv"}),
                .entry_point_name=std::string{"main"},
                .set_info_indices={0}
            },
            {
                .flags=vk::ShaderCreateFlagBitsEXT::eLinkStage,
                .stage=vk::ShaderStageFlagBits::eFragment,
                .code_type=vk::ShaderCodeTypeEXT::eSpirv,
                .code=jms::vulkan::Load(std::string{frag_shader_path}),
                .entry_point_name=std::string{"main"},
                .set_info_indices={0}
            }
        }
    };
}


constexpr std::vector<size_t> ShaderBindIndices() { return {0, 1}; }


}
