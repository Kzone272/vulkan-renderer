#pragma once

#include "vulkan-include.h"

struct VulkanState;

vk::CommandBuffer beginSingleTimeCommands(const VulkanState& vs);
void endSingleTimeCommands(const VulkanState& vs, vk::CommandBuffer cmd_buf);
