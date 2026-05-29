/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/scan_matching/vulkan_correlative_scan_matcher_2d.h"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>

#include "cartographer/mapping/internal/2d/scan_matching/score_candidates_spv.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

constexpr uint32_t kLocalSizeX = 64;

bool EnvFlagEnabled(const char* name) {
  const char* value = std::getenv(name);
  if (value == nullptr) return false;
  return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0 ||
         std::strcmp(value, "TRUE") == 0 || std::strcmp(value, "on") == 0 ||
         std::strcmp(value, "ON") == 0;
}

bool VkOk(const VkResult result, const char* operation) {
  if (result == VK_SUCCESS) return true;
  LOG(WARNING) << "Vulkan " << operation << " failed with VkResult "
               << result;
  return false;
}

struct GpuPoint {
  int32_t x;
  int32_t y;
};

struct GpuCandidate {
  uint32_t scan_start;
  uint32_t scan_count;
  int32_t x_index_offset;
  int32_t y_index_offset;
  float x;
  float y;
  float orientation;
};

struct GpuParams {
  uint32_t num_candidates;
  uint32_t grid_width;
  uint32_t grid_height;
  float translation_delta_cost_weight;
  float rotation_delta_cost_weight;
};

static_assert(sizeof(GpuPoint) == 8, "GpuPoint layout must match GLSL.");
static_assert(sizeof(GpuCandidate) == 28,
              "GpuCandidate layout must match GLSL.");
static_assert(sizeof(GpuParams) == 20, "GpuParams layout must match GLSL.");

class VulkanCandidateScorer {
  struct Buffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceSize size = 0;
  };

 public:
  VulkanCandidateScorer() { initialized_ = Initialize(); }

  ~VulkanCandidateScorer() {
    if (device_ != VK_NULL_HANDLE) {
      vkDeviceWaitIdle(device_);
      if (pipeline_ != VK_NULL_HANDLE) vkDestroyPipeline(device_, pipeline_, nullptr);
      if (pipeline_layout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
      }
      if (descriptor_set_layout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, descriptor_set_layout_, nullptr);
      }
      if (command_pool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, command_pool_, nullptr);
      }
      vkDestroyDevice(device_, nullptr);
    }
    if (instance_ != VK_NULL_HANDLE) vkDestroyInstance(instance_, nullptr);
  }

  bool Score(const ProbabilityGrid& probability_grid,
             const std::vector<DiscreteScan2D>& discrete_scans,
             const double translation_delta_cost_weight,
             const double rotation_delta_cost_weight,
             std::vector<Candidate2D>* const candidates) {
    if (!initialized_ || candidates == nullptr || candidates->empty()) {
      return false;
    }

    const MapLimits& limits = probability_grid.limits();
    const uint32_t grid_width = limits.cell_limits().num_x_cells;
    const uint32_t grid_height = limits.cell_limits().num_y_cells;
    if (grid_width == 0 || grid_height == 0) return false;

    const std::vector<uint16>& raw_cells =
        probability_grid.correspondence_cost_cells_for_scan_matching();
    if (raw_cells.size() != static_cast<size_t>(grid_width) * grid_height) {
      return false;
    }

    std::vector<uint32_t> grid_cells;
    grid_cells.reserve(raw_cells.size());
    for (const uint16 cell : raw_cells) {
      grid_cells.push_back(cell);
    }

    std::vector<GpuPoint> points;
    std::vector<uint32_t> scan_starts;
    scan_starts.reserve(discrete_scans.size());
    for (const DiscreteScan2D& scan : discrete_scans) {
      scan_starts.push_back(points.size());
      for (const Eigen::Array2i& point : scan) {
        points.push_back(GpuPoint{point.x(), point.y()});
      }
    }
    if (points.empty()) return false;

    std::vector<GpuCandidate> gpu_candidates;
    gpu_candidates.reserve(candidates->size());
    for (const Candidate2D& candidate : *candidates) {
      if (candidate.scan_index < 0 ||
          candidate.scan_index >= static_cast<int>(discrete_scans.size())) {
        return false;
      }
      const DiscreteScan2D& scan = discrete_scans[candidate.scan_index];
      if (scan.empty()) return false;
      gpu_candidates.push_back(GpuCandidate{
          scan_starts[candidate.scan_index],
          static_cast<uint32_t>(scan.size()),
          candidate.x_index_offset,
          candidate.y_index_offset,
          static_cast<float>(candidate.x),
          static_cast<float>(candidate.y),
          static_cast<float>(candidate.orientation),
      });
    }

    const GpuParams params{
        static_cast<uint32_t>(gpu_candidates.size()),
        grid_width,
        grid_height,
        static_cast<float>(translation_delta_cost_weight),
        static_cast<float>(rotation_delta_cost_weight),
    };
    std::vector<float> scores(candidates->size(), 0.f);

    Buffer grid_buffer;
    Buffer points_buffer;
    Buffer candidates_buffer;
    Buffer scores_buffer;
    Buffer params_buffer;
    if (!CreateAndUpload(grid_cells, &grid_buffer) ||
        !CreateAndUpload(points, &points_buffer) ||
        !CreateAndUpload(gpu_candidates, &candidates_buffer) ||
        !CreateAndUpload(scores, &scores_buffer) ||
        !CreateAndUploadOne(params, &params_buffer)) {
      DestroyBuffer(&grid_buffer);
      DestroyBuffer(&points_buffer);
      DestroyBuffer(&candidates_buffer);
      DestroyBuffer(&scores_buffer);
      DestroyBuffer(&params_buffer);
      return false;
    }

    bool ok = Dispatch(grid_buffer, points_buffer, candidates_buffer,
                       scores_buffer, params_buffer, scores.size());
    if (ok) {
      ok &= Download(scores_buffer, &scores);
    }

    DestroyBuffer(&grid_buffer);
    DestroyBuffer(&points_buffer);
    DestroyBuffer(&candidates_buffer);
    DestroyBuffer(&scores_buffer);
    DestroyBuffer(&params_buffer);

    if (!ok) return false;
    for (size_t i = 0; i < scores.size(); ++i) {
      (*candidates)[i].score = scores[i];
    }
    return true;
  }

 private:
  bool Initialize() {
    VkApplicationInfo app_info{};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pApplicationName = "cartographer_vulkan_scan_matcher";
    app_info.apiVersion = VK_API_VERSION_1_1;

    VkInstanceCreateInfo instance_info{};
    instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instance_info.pApplicationInfo = &app_info;
    if (!VkOk(vkCreateInstance(&instance_info, nullptr, &instance_),
              "vkCreateInstance")) {
      return false;
    }

    if (!PickPhysicalDevice()) return false;

    const float queue_priority = 1.f;
    VkDeviceQueueCreateInfo queue_info{};
    queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue_info.queueFamilyIndex = queue_family_index_;
    queue_info.queueCount = 1;
    queue_info.pQueuePriorities = &queue_priority;

    VkDeviceCreateInfo device_info{};
    device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    device_info.queueCreateInfoCount = 1;
    device_info.pQueueCreateInfos = &queue_info;
    if (!VkOk(vkCreateDevice(physical_device_, &device_info, nullptr, &device_),
              "vkCreateDevice")) {
      return false;
    }
    vkGetDeviceQueue(device_, queue_family_index_, 0, &queue_);

    VkCommandPoolCreateInfo command_pool_info{};
    command_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    command_pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    command_pool_info.queueFamilyIndex = queue_family_index_;
    if (!VkOk(vkCreateCommandPool(device_, &command_pool_info, nullptr,
                                  &command_pool_),
              "vkCreateCommandPool")) {
      return false;
    }

    return CreatePipeline();
  }

  bool PickPhysicalDevice() {
    uint32_t device_count = 0;
    if (!VkOk(vkEnumeratePhysicalDevices(instance_, &device_count, nullptr),
              "vkEnumeratePhysicalDevices") ||
        device_count == 0) {
      return false;
    }
    std::vector<VkPhysicalDevice> devices(device_count);
    if (!VkOk(vkEnumeratePhysicalDevices(instance_, &device_count,
                                         devices.data()),
              "vkEnumeratePhysicalDevices")) {
      return false;
    }

    for (const VkPhysicalDevice device : devices) {
      VkPhysicalDeviceProperties properties{};
      vkGetPhysicalDeviceProperties(device, &properties);
      if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU) continue;
      if (PickQueueFamily(device)) {
        physical_device_ = device;
        LOG(INFO) << "Using Vulkan device for scan matching: "
                  << properties.deviceName;
        return true;
      }
    }
    for (const VkPhysicalDevice device : devices) {
      if (PickQueueFamily(device)) {
        physical_device_ = device;
        return true;
      }
    }
    LOG(WARNING) << "No Vulkan compute queue found.";
    return false;
  }

  bool PickQueueFamily(const VkPhysicalDevice device) {
    uint32_t queue_family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queue_family_count,
                                             nullptr);
    std::vector<VkQueueFamilyProperties> queue_families(queue_family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(
        device, &queue_family_count, queue_families.data());
    for (uint32_t i = 0; i < queue_family_count; ++i) {
      if (queue_families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
        queue_family_index_ = i;
        return true;
      }
    }
    return false;
  }

  bool CreatePipeline() {
    std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
    for (uint32_t i = 0; i < bindings.size(); ++i) {
      bindings[i].binding = i;
      bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      bindings[i].descriptorCount = 1;
      bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layout_info.bindingCount = bindings.size();
    layout_info.pBindings = bindings.data();
    if (!VkOk(vkCreateDescriptorSetLayout(device_, &layout_info, nullptr,
                                          &descriptor_set_layout_),
              "vkCreateDescriptorSetLayout")) {
      return false;
    }

    VkPipelineLayoutCreateInfo pipeline_layout_info{};
    pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_layout_info.setLayoutCount = 1;
    pipeline_layout_info.pSetLayouts = &descriptor_set_layout_;
    if (!VkOk(vkCreatePipelineLayout(device_, &pipeline_layout_info, nullptr,
                                     &pipeline_layout_),
              "vkCreatePipelineLayout")) {
      return false;
    }

    VkShaderModuleCreateInfo shader_info{};
    shader_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    shader_info.codeSize = vulkan_shader::kScoreCandidatesShaderSpv_len;
    shader_info.pCode = reinterpret_cast<const uint32_t*>(
        vulkan_shader::kScoreCandidatesShaderSpv);
    VkShaderModule shader_module = VK_NULL_HANDLE;
    if (!VkOk(vkCreateShaderModule(device_, &shader_info, nullptr,
                                   &shader_module),
              "vkCreateShaderModule")) {
      return false;
    }

    VkPipelineShaderStageCreateInfo stage_info{};
    stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage_info.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage_info.module = shader_module;
    stage_info.pName = "main";

    VkComputePipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_info.stage = stage_info;
    pipeline_info.layout = pipeline_layout_;
    const VkResult result = vkCreateComputePipelines(
        device_, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &pipeline_);
    vkDestroyShaderModule(device_, shader_module, nullptr);
    return VkOk(result, "vkCreateComputePipelines");
  }

  uint32_t FindMemoryType(const uint32_t type_filter,
                          const VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memory_properties{};
    vkGetPhysicalDeviceMemoryProperties(physical_device_, &memory_properties);
    for (uint32_t i = 0; i < memory_properties.memoryTypeCount; ++i) {
      if ((type_filter & (1u << i)) &&
          (memory_properties.memoryTypes[i].propertyFlags & properties) ==
              properties) {
        return i;
      }
    }
    return std::numeric_limits<uint32_t>::max();
  }

  bool CreateBuffer(const VkDeviceSize size, Buffer* const buffer) {
    CHECK(buffer != nullptr);
    if (size == 0) return false;
    buffer->size = size;

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = size;
    buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (!VkOk(vkCreateBuffer(device_, &buffer_info, nullptr, &buffer->buffer),
              "vkCreateBuffer")) {
      return false;
    }

    VkMemoryRequirements memory_requirements{};
    vkGetBufferMemoryRequirements(device_, buffer->buffer, &memory_requirements);
    const uint32_t memory_type = FindMemoryType(
        memory_requirements.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (memory_type == std::numeric_limits<uint32_t>::max()) {
      LOG(WARNING) << "No host-visible Vulkan memory type found.";
      return false;
    }

    VkMemoryAllocateInfo allocation_info{};
    allocation_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocation_info.allocationSize = memory_requirements.size;
    allocation_info.memoryTypeIndex = memory_type;
    if (!VkOk(vkAllocateMemory(device_, &allocation_info, nullptr,
                               &buffer->memory),
              "vkAllocateMemory")) {
      return false;
    }
    if (!VkOk(vkBindBufferMemory(device_, buffer->buffer, buffer->memory, 0),
              "vkBindBufferMemory")) {
      return false;
    }
    return true;
  }

  template <typename T>
  bool CreateAndUpload(const std::vector<T>& data, Buffer* const buffer) {
    if (!CreateBuffer(sizeof(T) * data.size(), buffer)) return false;
    void* mapped = nullptr;
    if (!VkOk(vkMapMemory(device_, buffer->memory, 0, buffer->size, 0, &mapped),
              "vkMapMemory")) {
      return false;
    }
    std::memcpy(mapped, data.data(), static_cast<size_t>(buffer->size));
    vkUnmapMemory(device_, buffer->memory);
    return true;
  }

  template <typename T>
  bool CreateAndUploadOne(const T& data, Buffer* const buffer) {
    if (!CreateBuffer(sizeof(T), buffer)) return false;
    void* mapped = nullptr;
    if (!VkOk(vkMapMemory(device_, buffer->memory, 0, buffer->size, 0, &mapped),
              "vkMapMemory")) {
      return false;
    }
    std::memcpy(mapped, &data, sizeof(T));
    vkUnmapMemory(device_, buffer->memory);
    return true;
  }

  bool Download(const Buffer& buffer, std::vector<float>* const scores) {
    void* mapped = nullptr;
    if (!VkOk(vkMapMemory(device_, buffer.memory, 0, buffer.size, 0, &mapped),
              "vkMapMemory")) {
      return false;
    }
    std::memcpy(scores->data(), mapped, static_cast<size_t>(buffer.size));
    vkUnmapMemory(device_, buffer.memory);
    return true;
  }

  void DestroyBuffer(Buffer* const buffer) {
    if (buffer == nullptr) return;
    if (buffer->buffer != VK_NULL_HANDLE) {
      vkDestroyBuffer(device_, buffer->buffer, nullptr);
    }
    if (buffer->memory != VK_NULL_HANDLE) {
      vkFreeMemory(device_, buffer->memory, nullptr);
    }
    *buffer = Buffer{};
  }

  bool Dispatch(const Buffer& grid_buffer, const Buffer& points_buffer,
                const Buffer& candidates_buffer, const Buffer& scores_buffer,
                const Buffer& params_buffer, const size_t num_scores) {
    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    pool_size.descriptorCount = 5;

    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.maxSets = 1;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
    if (!VkOk(vkCreateDescriptorPool(device_, &pool_info, nullptr,
                                     &descriptor_pool),
              "vkCreateDescriptorPool")) {
      return false;
    }

    VkDescriptorSetAllocateInfo set_info{};
    set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    set_info.descriptorPool = descriptor_pool;
    set_info.descriptorSetCount = 1;
    set_info.pSetLayouts = &descriptor_set_layout_;
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
    if (!VkOk(vkAllocateDescriptorSets(device_, &set_info, &descriptor_set),
              "vkAllocateDescriptorSets")) {
      vkDestroyDescriptorPool(device_, descriptor_pool, nullptr);
      return false;
    }

    std::array<VkDescriptorBufferInfo, 5> buffer_infos{{
        {grid_buffer.buffer, 0, grid_buffer.size},
        {points_buffer.buffer, 0, points_buffer.size},
        {candidates_buffer.buffer, 0, candidates_buffer.size},
        {scores_buffer.buffer, 0, scores_buffer.size},
        {params_buffer.buffer, 0, params_buffer.size},
    }};
    std::array<VkWriteDescriptorSet, 5> writes{};
    for (uint32_t i = 0; i < writes.size(); ++i) {
      writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      writes[i].dstSet = descriptor_set;
      writes[i].dstBinding = i;
      writes[i].descriptorCount = 1;
      writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      writes[i].pBufferInfo = &buffer_infos[i];
    }
    vkUpdateDescriptorSets(device_, writes.size(), writes.data(), 0, nullptr);

    VkCommandBufferAllocateInfo command_buffer_info{};
    command_buffer_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    command_buffer_info.commandPool = command_pool_;
    command_buffer_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_buffer_info.commandBufferCount = 1;
    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    if (!VkOk(vkAllocateCommandBuffers(device_, &command_buffer_info,
                                       &command_buffer),
              "vkAllocateCommandBuffers")) {
      vkDestroyDescriptorPool(device_, descriptor_pool, nullptr);
      return false;
    }

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    bool ok = VkOk(vkBeginCommandBuffer(command_buffer, &begin_info),
                   "vkBeginCommandBuffer");
    if (ok) {
      vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE,
                        pipeline_);
      vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE,
                              pipeline_layout_, 0, 1, &descriptor_set, 0,
                              nullptr);
      vkCmdDispatch(command_buffer,
                    static_cast<uint32_t>((num_scores + kLocalSizeX - 1) /
                                          kLocalSizeX),
                    1, 1);
      ok = VkOk(vkEndCommandBuffer(command_buffer), "vkEndCommandBuffer");
    }

    VkFence fence = VK_NULL_HANDLE;
    if (ok) {
      VkFenceCreateInfo fence_info{};
      fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
      ok = VkOk(vkCreateFence(device_, &fence_info, nullptr, &fence),
                "vkCreateFence");
    }
    if (ok) {
      VkSubmitInfo submit_info{};
      submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submit_info.commandBufferCount = 1;
      submit_info.pCommandBuffers = &command_buffer;
      ok = VkOk(vkQueueSubmit(queue_, 1, &submit_info, fence),
                "vkQueueSubmit");
    }
    if (ok) {
      ok = VkOk(vkWaitForFences(device_, 1, &fence, VK_TRUE,
                                std::numeric_limits<uint64_t>::max()),
                "vkWaitForFences");
    }

    if (fence != VK_NULL_HANDLE) vkDestroyFence(device_, fence, nullptr);
    vkFreeCommandBuffers(device_, command_pool_, 1, &command_buffer);
    vkDestroyDescriptorPool(device_, descriptor_pool, nullptr);
    return ok;
  }

  bool initialized_ = false;
  VkInstance instance_ = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
  VkDevice device_ = VK_NULL_HANDLE;
  VkQueue queue_ = VK_NULL_HANDLE;
  uint32_t queue_family_index_ = 0;
  VkCommandPool command_pool_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout descriptor_set_layout_ = VK_NULL_HANDLE;
  VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
  VkPipeline pipeline_ = VK_NULL_HANDLE;
};

}  // namespace

bool ScoreCandidatesWithVulkan(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const double translation_delta_cost_weight,
    const double rotation_delta_cost_weight,
    std::vector<Candidate2D>* const candidates) {
  if (!EnvFlagEnabled("CARTOGRAPHER_VULKAN_CORRELATIVE_SCAN_MATCHER")) {
    return false;
  }
  static VulkanCandidateScorer* scorer = new VulkanCandidateScorer();
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  return scorer->Score(probability_grid, discrete_scans,
                       translation_delta_cost_weight,
                       rotation_delta_cost_weight, candidates);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
