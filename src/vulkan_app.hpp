#pragma once


#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <print>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "glyphy/glm.hpp"

#include "jms/graphics/projection.hpp"
#include "jms/utils/no_mutex.hpp"
#include "jms/vulkan/vulkan.hpp"
#include "jms/vulkan/camera.hpp"
#include "jms/vulkan/info.hpp"
#include "jms/vulkan/memory.hpp"
#include "jms/vulkan/memory_resource.hpp"
#include "jms/vulkan/state.hpp"
#include "jms/vulkan/utils.hpp"
#include "jms/vulkan/variants.hpp"
#include "jms/wsi/glfw.hpp"
#include "jms/wsi/glfw.cpp"
#include "jms/wsi/surface.hpp"

#include "jms/vulkan/scratch/commands.hpp"


#include <glm/gtx/string_cast.hpp>


#include "glyphy/shader.hpp"


constexpr const size_t WINDOW_WIDTH{1024};
constexpr const size_t WINDOW_HEIGHT{1024};


struct AppState {
    size_t window_width{WINDOW_WIDTH};
    size_t window_height{WINDOW_HEIGHT};

    std::string app_name{"GlyphyReduced_Vulkan"};
    std::string engine_name{"GlyphyReduced_Vulkan_ENGINE"};

    std::vector<std::string> instance_extensions{
        VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME
    };

    std::vector<std::string> instance_layers{
        std::string{"VK_LAYER_KHRONOS_synchronization2"},
        std::string{"VK_LAYER_KHRONOS_shader_object"}
    };

    std::vector<std::string> device_extensions{
        VK_EXT_EXTENDED_DYNAMIC_STATE_EXTENSION_NAME,
        VK_EXT_SHADER_OBJECT_EXTENSION_NAME,
        //VK_EXT_EXTENDED_DYNAMIC_STATE_2_EXTENSION_NAME,//VK_EXT_VERTEX_INPUT_DYNAMIC_STATE_EXTENSION_NAME,
        VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME,
        VK_KHR_CREATE_RENDERPASS_2_EXTENSION_NAME,
        VK_KHR_DEPTH_STENCIL_RESOLVE_EXTENSION_NAME,
        VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME,
        VK_KHR_MAINTENANCE2_EXTENSION_NAME,
        VK_KHR_MULTIVIEW_EXTENSION_NAME,
        VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

    std::vector<std::string> device_layers{};
    vk::PhysicalDeviceFeatures device_features{
        .depthClamp=true,
        .depthBounds=true
        //.shaderStorageBufferArrayDynamicIndexing=true
    };
    std::vector<jms::vulkan::DeviceCreateInfo2Variant> device_pnext_features{
        vk::PhysicalDeviceShaderObjectFeaturesEXT{.shaderObject=true},
        vk::PhysicalDeviceDynamicRenderingFeatures{.dynamicRendering=true}
    };

    std::vector<vk::MemoryPropertyFlags> memory_types{
        vk::MemoryPropertyFlagBits::eDeviceLocal,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    };

    jms::vulkan::GraphicsRenderingState default_graphics_rendering_state{
        .render_area{
            .extent{
                .width=static_cast<uint32_t>(WINDOW_WIDTH),
                .height=static_cast<uint32_t>(WINDOW_HEIGHT)
            }
        },

        .depth_attachment=vk::RenderingAttachmentInfo{
            .imageLayout=vk::ImageLayout::eDepthAttachmentOptimal,
            .loadOp=vk::AttachmentLoadOp::eClear,
            .storeOp=vk::AttachmentStoreOp::eStore,
            .clearValue=vk::ClearValue{.depthStencil={.depth=0.0f, .stencil=0}}
        },

        .viewports={{
            .x=0,
            .y=0,
            .width=static_cast<float>(WINDOW_WIDTH),
            .height=static_cast<float>(WINDOW_HEIGHT),
            .minDepth=1.0f,
            .maxDepth=0.0f
        }},

        .scissors={{
            .extent{
                .width=static_cast<uint32_t>(WINDOW_WIDTH),
                .height=static_cast<uint32_t>(WINDOW_HEIGHT)
            }
        }},

        .depth_test_enabled=true,
        .depth_clamp_enabled=true,
        .depth_compare_op=vk::CompareOp::eGreaterOrEqual,
        .depth_write_enabled=true
    };
};


struct DrawState {
    jms::vulkan::State& vulkan_state;
    jms::vulkan::GraphicsPass& graphics_pass;
    vk::Buffer vertex_buffer;
    vk::Buffer vertex_data_buffer;
    vk::Buffer index_buffer;
    uint32_t num_indices;
    std::vector<vk::DescriptorSet> descriptor_sets{};
    vk::Image depth_image;
    vk::ImageView depth_image_view;
};


/***
 * Forward declarations
 */
jms::vulkan::State CreateEnvironment(const AppState&, std::optional<jms::wsi::glfw::Window*> = std::nullopt);
void DrawFrame(DrawState& draw_state);
template <typename AtlasUnit_t>
void Run(const std::vector<AtlasUnit_t>& atlas_buffer_data,
         const glyphy::shader::TextVertexData& text_vertex_data,
         const glyphy::shader::GlyphInfo& glyph_state,
         const std::string_view& frag_shader_path);


/***
 * Implementations
 */
jms::vulkan::State CreateEnvironment(const AppState& app_state, std::optional<jms::wsi::glfw::Window*> window) {
    jms::vulkan::State vulkan_state{};

    vulkan_state.InitInstance(jms::vulkan::InstanceConfig{
        .app_name=app_state.app_name,
        .engine_name=app_state.engine_name,
        .layer_names=app_state.instance_layers,
        .extension_names=app_state.instance_extensions
    });

    vk::raii::PhysicalDevice& physical_device = vulkan_state.physical_devices.at(0);
    uint32_t queue_family_index = 0;
    std::vector<float> queue_priority{1.0f, 1.0f};

    vulkan_state.InitDevice(physical_device, jms::vulkan::DeviceConfig{
        .layer_names=app_state.device_layers,
        .extension_names=app_state.device_extensions,
        .features=app_state.device_features,
        .queue_family_index=queue_family_index,
        .queue_priority=queue_priority,
        .pnext_features=app_state.device_pnext_features
    });

    vk::raii::Device& device = vulkan_state.devices.at(0);

    vulkan_state.command_buffers.push_back(device.allocateCommandBuffers({
        .commandPool=*(vulkan_state.command_pools.at(0)),
        .level=vk::CommandBufferLevel::ePrimary,
        .commandBufferCount=2
    }));

    vulkan_state.memory_helper = {physical_device, device, app_state.memory_types};

    if (app_state.default_graphics_rendering_state.depth_attachment.has_value()) {
    }

    if (window.has_value()) {
        vulkan_state.surface = jms::wsi::glfw::CreateSurface(*window.value(), vulkan_state.instance);
        auto surface_render_info = jms::wsi::FromSurface(vulkan_state.surface,
                                                         physical_device,
                                                         static_cast<uint32_t>(app_state.window_width),
                                                         static_cast<uint32_t>(app_state.window_height));
        vulkan_state.InitSwapchain(device, vulkan_state.surface, surface_render_info);
    }

    return vulkan_state;
}


void DrawFrame(DrawState& draw_state) {
    auto& vulkan_state = draw_state.vulkan_state;
    auto& device = vulkan_state.devices.at(0);
    auto& image_available_semaphore = vulkan_state.semaphores.at(0);
    auto& render_finished_semaphore = vulkan_state.semaphores.at(1);
    auto& present_finished_semaphore = vulkan_state.semaphores.at(2);
    auto& in_flight_fence = vulkan_state.fences.at(0);
    auto& graphics_queue = vulkan_state.graphics_queue.at(0);
    auto& present_queue = vulkan_state.present_queue.at(0);
    auto& vs_command_buffers_0 = vulkan_state.command_buffers.at(0);
    auto& command_buffer_0 = vs_command_buffers_0.at(0);
    auto& command_buffer_1 = vs_command_buffers_0.at(0);
    auto& swapchain = vulkan_state.swapchain;
    auto& swapchain_image_views = vulkan_state.swapchain_image_views;

    vk::Result result = device.waitForFences({*in_flight_fence}, VK_TRUE, std::numeric_limits<uint64_t>::max());
    device.resetFences({*in_flight_fence});

    uint32_t swapchain_image_index = 0;
    std::tie(result, swapchain_image_index) = swapchain.acquireNextImage(std::numeric_limits<uint64_t>::max(),
                                                                         *image_available_semaphore);
    assert(result == vk::Result::eSuccess);
    assert(swapchain_image_index < swapchain_image_views.size());

    vk::Image depth_image = draw_state.depth_image;
    vk::ImageView depth_image_view = draw_state.depth_image_view;
    vk::Image target_image = swapchain.getImages().at(swapchain_image_index);
    vk::ImageView target_view = *swapchain_image_views.at(swapchain_image_index);
    vk::ImageMemoryBarrier image_barrier{
        .srcAccessMask=vk::AccessFlagBits::eColorAttachmentWrite,
        .dstAccessMask={},
        .oldLayout=vk::ImageLayout::eUndefined,
        .newLayout=vk::ImageLayout::ePresentSrcKHR,
        .srcQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED,
        .image=target_image,
        .subresourceRange=vk::ImageSubresourceRange{
            .aspectMask=vk::ImageAspectFlagBits::eColor,
            .baseMipLevel=0,
            .levelCount=1,
            .baseArrayLayer=0,
            .layerCount=1
        }
    };
    vk::ImageMemoryBarrier depth_barrier{
        .srcAccessMask={},
        .dstAccessMask=vk::AccessFlagBits::eDepthStencilAttachmentWrite,
        .oldLayout=vk::ImageLayout::eUndefined,
        .newLayout=vk::ImageLayout::eDepthStencilAttachmentOptimal,
        .srcQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED,
        .image=depth_image,
        .subresourceRange=vk::ImageSubresourceRange{
            .aspectMask=vk::ImageAspectFlagBits::eDepth,
            .baseMipLevel=0,
            .levelCount=1,
            .baseArrayLayer=0,
            .layerCount=1
        }
    };

    command_buffer_0.reset();
    command_buffer_0.begin({.pInheritanceInfo=nullptr});
    command_buffer_0.pipelineBarrier(vk::PipelineStageFlagBits::eEarlyFragmentTests | vk::PipelineStageFlagBits::eLateFragmentTests,
                                     vk::PipelineStageFlagBits::eEarlyFragmentTests | vk::PipelineStageFlagBits::eLateFragmentTests,
                                     vk::DependencyFlags{},
                                     {},
                                     {},
                                     {depth_barrier});
    draw_state.graphics_pass.ToCommands(
        command_buffer_0,
        {target_view},
        {},
        depth_image_view,
        draw_state.descriptor_sets,
        {},
        [a=0,
         b=std::vector<vk::Buffer>{draw_state.vertex_buffer, draw_state.vertex_data_buffer},
         c=std::vector<vk::DeviceSize>{0, 0}](auto& cb) {
            cb.bindVertexBuffers(a, b, c);
         },
        [&a=draw_state.index_buffer, b=0, c=vk::IndexType::eUint32](auto& cb) { cb.bindIndexBuffer(a, b, c); },
        [&a=draw_state.graphics_pass, &F=glyphy::shader::ShaderBindIndices](auto& cb) { a.BindShaders(cb, F()); },
        [&a=draw_state.num_indices, b=1, c=0, d=0, e=0](auto& cb) { cb.drawIndexed(a, b, c, d, e); }
    );
    command_buffer_0.pipelineBarrier(vk::PipelineStageFlagBits::eColorAttachmentOutput,
                                     vk::PipelineStageFlagBits::eBottomOfPipe,
                                     vk::DependencyFlags{},
                                     {},
                                     {},
                                     {image_barrier});
    command_buffer_0.end();

    std::vector<vk::Semaphore> wait_semaphores{*image_available_semaphore};
    std::vector<vk::Semaphore> render_semaphores{*render_finished_semaphore};
    std::vector<vk::Semaphore> present_semaphores{*present_finished_semaphore};
    std::vector<vk::PipelineStageFlags> dst_stage_mask{vk::PipelineStageFlagBits::eColorAttachmentOutput};
    std::vector<vk::CommandBuffer> command_buffers_0{*command_buffer_0};
    std::vector<vk::CommandBuffer> command_buffers_1{*command_buffer_1};

    graphics_queue.submit(std::array<vk::SubmitInfo, 1>{vk::SubmitInfo{
        .waitSemaphoreCount=static_cast<uint32_t>(wait_semaphores.size()),
        .pWaitSemaphores=jms::vulkan::VectorAsPtr(wait_semaphores),
        .pWaitDstStageMask=dst_stage_mask.data(),
        .commandBufferCount=static_cast<uint32_t>(command_buffers_0.size()),
        .pCommandBuffers=jms::vulkan::VectorAsPtr(command_buffers_0),
        .signalSemaphoreCount=static_cast<uint32_t>(present_semaphores.size()),
        .pSignalSemaphores=jms::vulkan::VectorAsPtr(present_semaphores)
    }}, *in_flight_fence);

    std::vector<vk::SwapchainKHR> swapchains{*swapchain};
    result = present_queue.presentKHR({
        .waitSemaphoreCount=static_cast<uint32_t>(present_semaphores.size()),
        .pWaitSemaphores=jms::vulkan::VectorAsPtr(present_semaphores),
        .swapchainCount=static_cast<uint32_t>(swapchains.size()),
        .pSwapchains=jms::vulkan::VectorAsPtr(swapchains),
        .pImageIndices=&swapchain_image_index,
        .pResults=nullptr
    });
}


template <typename AtlasUnit_t>
void Run(const std::vector<AtlasUnit_t>& atlas_buffer_data,
         const glyphy::shader::TextVertexData& text_vertex_data,
         const glyphy::shader::GlyphInfo& glyph_state,
         const std::string_view& frag_shader_path) {
    AppState app_state{};

    jms::wsi::glfw::Environment glfw_environment{};
    glfw_environment.EnableHIDPI();
    auto window = jms::wsi::glfw::Window::DefaultCreate(app_state.window_width, app_state.window_height);
    std::ranges::transform(jms::wsi::glfw::GetVulkanInstanceExtensions(),
                            std::back_inserter(app_state.instance_extensions),
                            [](auto& i) { return i; });

    jms::vulkan::State vulkan_state = CreateEnvironment(app_state, std::addressof(window));
    vk::raii::PhysicalDevice& physical_device = vulkan_state.physical_devices.at(0);
    vk::raii::Device& device = vulkan_state.devices.at(0);

    jms::vulkan::GraphicsPass graphics_pass{
        device, app_state.default_graphics_rendering_state, std::move(glyphy::shader::CreateGroup(frag_shader_path))};
    vk::raii::DescriptorPool descriptor_pool = graphics_pass.CreateDescriptorPool(device);
    std::vector<vk::DescriptorSet> descriptor_sets = graphics_pass.CreateDescriptorSets(device, descriptor_pool);
    vulkan_state.semaphores.push_back(device.createSemaphore({}));
    vulkan_state.semaphores.push_back(device.createSemaphore({}));
    vulkan_state.semaphores.push_back(device.createSemaphore({}));
    vulkan_state.fences.push_back(device.createFence({.flags=vk::FenceCreateFlagBits::eSignaled}));

    /***
     * Depth Buffer
     */
    uint32_t local_memory_type_index = vulkan_state.memory_helper.GetMemoryTypeIndex(0);
    uint32_t host_coherent_memory_type_index = vulkan_state.memory_helper.GetMemoryTypeIndex(1);
    vk::raii::Image depth_image = device.createImage({
        .flags={},
        .imageType=vk::ImageType::e2D,
        .format=vk::Format::eD32Sfloat,
        .extent={
            .width=static_cast<uint32_t>(app_state.window_width),
            .height=static_cast<uint32_t>(app_state.window_height),
            .depth=1
        },
        .mipLevels=1,
        .arrayLayers=1,
        .samples=vk::SampleCountFlagBits::e1,
        .tiling=vk::ImageTiling::eOptimal,
        .usage=vk::ImageUsageFlagBits::eDepthStencilAttachment,
        .sharingMode=vk::SharingMode::eExclusive,
        .initialLayout=vk::ImageLayout::eUndefined
    });
    vk::MemoryRequirements reqs = depth_image.getMemoryRequirements();
    if (!static_cast<bool>(reqs.memoryTypeBits & local_memory_type_index)) {
        throw std::runtime_error{"Cannot allocate resource with the given allocated device memory."};
    }
    vk::raii::DeviceMemory dev_mem = device.allocateMemory({
        .allocationSize=reqs.size,
        .memoryTypeIndex=local_memory_type_index
    });
    depth_image.bindMemory(*dev_mem, 0);
    vk::raii::ImageView depth_image_view = device.createImageView({
        .flags={},
        .image=*depth_image,
        .viewType=vk::ImageViewType::e2D,
        .format=vk::Format::eD32Sfloat,
        .components={
            .r{vk::ComponentSwizzle::eIdentity},
            .g{vk::ComponentSwizzle::eIdentity},
            .b{vk::ComponentSwizzle::eIdentity},
            .a{vk::ComponentSwizzle::eIdentity}
        },
        .subresourceRange={
            .aspectMask{vk::ImageAspectFlagBits::eDepth},
            .baseMipLevel{0},
            .levelCount{1},
            .baseArrayLayer{0},
            .layerCount{1}
        }
    });


    /***
     * Data Buffers
     */
    auto ToSize = []<typename T>(vk::DeviceSize quantity) -> vk::DeviceSize {
        vk::DeviceSize type_size = static_cast<vk::DeviceSize>(sizeof(T));
        vk::DeviceSize total = ((type_size / 64) + (type_size % 64 ? 1 : 0)) * 64;
        return total * quantity;
    };
    auto CreateBuffer = [&device](vk::DeviceSize num_bytes) -> vk::raii::Buffer {
        return device.createBuffer({
            .size=num_bytes,
            .usage=vk::BufferUsageFlagBits::eStorageBuffer,
            .sharingMode=vk::SharingMode::eExclusive
        });
    };
    vk::raii::Buffer vertex_buffer = device.createBuffer({
        .size=ToSize.operator()<glyphy::shader::Vertex>(65536),
        .usage=vk::BufferUsageFlagBits::eVertexBuffer,
        .sharingMode=vk::SharingMode::eExclusive
    });
    vk::raii::Buffer index_buffer = device.createBuffer({
        .size=(4 * 65536),
        .usage=vk::BufferUsageFlagBits::eIndexBuffer,
        .sharingMode=vk::SharingMode::eExclusive
    });
    vk::raii::Buffer mvp_buffer = CreateBuffer(256);//sizeof(glyphy::shader::MVP));
    vk::raii::Buffer gsb_buffer = CreateBuffer(256);//sizeof(glyphy::shader::GlyphInfo));
    vk::raii::Buffer vertex_data_buffer = CreateBuffer(ToSize.operator()<glyphy::shader::VertexData>(65536));
    vk::raii::Buffer model_data_buffer = CreateBuffer(ToSize.operator()<glyphy::shader::ModelInfo>(65536));
    vk::raii::Buffer atlas_buffer = CreateBuffer(ToSize.operator()<AtlasUnit_t>(65536));

    auto CreateAndBindMem = [&device, &host_coherent_memory_type_index](vk::raii::Buffer& buffer) -> vk::raii::DeviceMemory {
        auto reqs = buffer.getMemoryRequirements();
        if (!static_cast<bool>(reqs.memoryTypeBits & host_coherent_memory_type_index)) {
            throw std::runtime_error{"Cannot allocate resource with the given allocated device memory."};
        }
        vk::raii::DeviceMemory dev_mem = device.allocateMemory({
            .allocationSize=reqs.size,
            .memoryTypeIndex=host_coherent_memory_type_index
        });
        buffer.bindMemory(*dev_mem, 0);
        return dev_mem;
    };
    vk::raii::DeviceMemory vertex_mem = CreateAndBindMem(vertex_buffer);
    vk::raii::DeviceMemory index_mem = CreateAndBindMem(index_buffer);
    vk::raii::DeviceMemory mvp_mem = CreateAndBindMem(mvp_buffer);
    vk::raii::DeviceMemory gsb_mem = CreateAndBindMem(gsb_buffer);
    vk::raii::DeviceMemory vertex_data_mem = CreateAndBindMem(vertex_data_buffer);
    vk::raii::DeviceMemory model_data_mem = CreateAndBindMem(model_data_buffer);
    vk::raii::DeviceMemory atlas_mem = CreateAndBindMem(atlas_buffer);

    auto MapMem = []<typename T>(vk::raii::DeviceMemory& mem, size_t obj_size, vk::DeviceSize quantity) -> std::span<T> {
        vk::DeviceSize s = static_cast<vk::DeviceSize>(obj_size);
        vk::DeviceSize total_bytes = s * quantity;
        void* mapped_mem = mem.mapMemory(0, total_bytes);
        std::memset(mapped_mem, 0, total_bytes);
        T* t = static_cast<T*>(mapped_mem);
        return std::span{t, quantity};
    };
    auto vertices =    MapMem.operator()<glyphy::shader::Vertex>    (vertex_mem,      sizeof(glyphy::shader::Vertex),     65536);
    auto indices =     MapMem.operator()<uint32_t>                  (index_mem,       sizeof(uint32_t),                   65536);
    auto mvp_data =    MapMem.operator()<glyphy::shader::MVP>       (mvp_mem,         sizeof(glyphy::shader::MVP),        1);
    auto gsb_data =    MapMem.operator()<glyphy::shader::GlyphInfo> (gsb_mem,         sizeof(glyphy::shader::GlyphInfo),  1);
    auto vertex_data = MapMem.operator()<glyphy::shader::VertexData>(vertex_data_mem, sizeof(glyphy::shader::VertexData), 65536);
    auto model_data =  MapMem.operator()<glyphy::shader::ModelInfo> (model_data_mem,  sizeof(glyphy::shader::ModelInfo),  65536);
    auto atlas_data =  MapMem.operator()<AtlasUnit_t>               (atlas_mem,       sizeof(AtlasUnit_t),                65536);

    /***
     * World
     *
     * Z Y
     * |/
     * .--X
     */
    glm::mat4 world{1.0f};
    glm::mat4 projection = jms::Perspective_RH_OI(glm::radians(45.0f), 1.0f, 0.01f);
    /***
     * camera view
     *
     * Z Y
     * |/
     * .--X
     */
    // https://johannesugb.github.io/gpu-programming/setting-up-a-proper-vulkan-projection-matrix/
    glm::mat4 X{{1.0f,  0.0f,  0.0f, 0.0f},
                {0.0f,  0.0f, -1.0f, 0.0f},
                {0.0f,  1.0f,  0.0f, 0.0f},
                {0.0f,  0.0f,  0.0f, 1.0f}};
    projection = projection * glm::inverse(X);
    jms::vulkan::Camera camera{projection,
                               {50.0f, -120.0f, 0.0f},
                               {glm::radians(0.0f), glm::radians(0.0f), glm::radians(0.0f)}};
    mvp_data[0].mvp = camera.View();
    std::ranges::copy(text_vertex_data.vertices, vertices.begin());
    std::ranges::copy(text_vertex_data.vertex_data, vertex_data.begin());
    std::ranges::copy(text_vertex_data.indices, indices.begin());
    // begin tmp
    model_data[0] = glyphy::shader::ModelInfo{};
    gsb_data[0] = glyph_state;
    // end tmp
    // update atlas image
    // msvc doesn't currently allow std::span with std::ranges::copy
    std::copy(atlas_buffer_data.begin(), atlas_buffer_data.end(), atlas_data.begin());

    auto NumBytes = []<typename T>(const T& t) -> vk::DeviceSize {
        return static_cast<vk::DeviceSize>(t.size() * sizeof(typename T::value_type));
    };
    graphics_pass.UpdateDescriptorSets(
        device, descriptor_sets, 0,
        {
            { 0, {.buffer=*mvp_buffer,         .offset=0, .range=static_cast<vk::DeviceSize>(sizeof(glyphy::shader::MVP))} },
            { 1, {.buffer=*model_data_buffer,  .offset=0, .range=NumBytes(model_data)} },
            { 2, {.buffer=*gsb_buffer,         .offset=0, .range=static_cast<vk::DeviceSize>(sizeof(glyphy::shader::GlyphInfo))} },
            { 3, {.buffer=*atlas_buffer,       .offset=0, .range=NumBytes(atlas_data)} }
        },
        {},
        {}
    );

    /***
     * Update draw state
     */
    DrawState draw_state{
        .vulkan_state=vulkan_state,
        .graphics_pass=graphics_pass,
        .vertex_buffer=*vertex_buffer,
        .vertex_data_buffer=*vertex_data_buffer,
        .index_buffer=*index_buffer,
        .num_indices=static_cast<uint32_t>(indices.size()),
        .descriptor_sets=descriptor_sets,
        .depth_image=*depth_image,
        .depth_image_view=*depth_image_view
    };

    bool not_done = true;
    int ix = 500;
    if (glfwRawMouseMotionSupported()) {
        glfwSetInputMode(window.get(), GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
        glfwSetInputMode(window.get(), GLFW_STICKY_MOUSE_BUTTONS, GLFW_TRUE);
    }
    double xpos = 0.0;
    double ypos = 0.0;
    glfwGetCursorPos(window.get(), &xpos, &ypos);
    bool inverse_pitch = true;
    float move_speed = 0.01;
    float pitch_speed = 0.01;
    float yaw_speed = 0.01;
    float roll_speed = 0.01;

    while (not_done) {
        glfwPollEvents();
        if (glfwGetKey(window.get(), GLFW_KEY_Q) == GLFW_PRESS) {
            not_done = false;
            continue;
        }

        double prev_xpos = xpos;
        double prev_ypos = ypos;
        glfwGetCursorPos(window.get(), &xpos, &ypos);

        glm::vec3 pos_delta{0.0f};
        glm::vec3 rot_delta{0.0f};

        if (glfwRawMouseMotionSupported()) {
            int lmb_pressed = glfwGetMouseButton(window.get(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            int rmb_pressed = glfwGetMouseButton(window.get(), GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            if (lmb_pressed || rmb_pressed) {
                glfwSetInputMode(window.get(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                float delta_xpos = static_cast<float>(xpos - prev_xpos);
                float delta_ypos = static_cast<float>(ypos - prev_ypos);
                rot_delta.x += delta_ypos * pitch_speed * (inverse_pitch ? 1.0 : -1.0); // pitch
                rot_delta.z += delta_xpos * yaw_speed; // yaw
            } else {
                glfwSetInputMode(window.get(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }

        if (glfwGetKey(window.get(), GLFW_KEY_W) == GLFW_PRESS) { pos_delta.y += move_speed; }
        if (glfwGetKey(window.get(), GLFW_KEY_S) == GLFW_PRESS) { pos_delta.y -= move_speed; }
        if (glfwGetKey(window.get(), GLFW_KEY_A) == GLFW_PRESS) { pos_delta.x -= move_speed; }
        if (glfwGetKey(window.get(), GLFW_KEY_D) == GLFW_PRESS) { pos_delta.x += move_speed; }
        if (glfwGetKey(window.get(), GLFW_KEY_Z) == GLFW_PRESS) { pos_delta.z -= move_speed; }
        if (glfwGetKey(window.get(), GLFW_KEY_C) == GLFW_PRESS) { pos_delta.z += move_speed; }
        if (glfwGetKey(window.get(), GLFW_KEY_1) == GLFW_PRESS) { rot_delta.x += glm::radians(pitch_speed); }
        if (glfwGetKey(window.get(), GLFW_KEY_2) == GLFW_PRESS) { rot_delta.y += glm::radians(roll_speed); }
        if (glfwGetKey(window.get(), GLFW_KEY_3) == GLFW_PRESS) { rot_delta.z += glm::radians(yaw_speed); }
        if (glfwGetKey(window.get(), GLFW_KEY_4) == GLFW_PRESS) { rot_delta.x -= glm::radians(pitch_speed); }
        if (glfwGetKey(window.get(), GLFW_KEY_5) == GLFW_PRESS) { rot_delta.y -= glm::radians(roll_speed); }
        if (glfwGetKey(window.get(), GLFW_KEY_6) == GLFW_PRESS) { rot_delta.z -= glm::radians(yaw_speed); }
        camera.Update(pos_delta, rot_delta);
        mvp_data[0].mvp = camera.View();

        DrawFrame(draw_state);
        vulkan_state.devices.at(0).waitIdle();
    }
}
