#pragma once


#if 1

#include "jms/external/glm.hpp"

#else

#define GLM_FORCE_CXX2A
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_FORCE_RADIANS
#define GLM_FORCE_SIZE_T_LENGTH
#include <glm/glm.hpp>

#endif
