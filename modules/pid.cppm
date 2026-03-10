/**
 * @file pid.cppm
 * @brief PID
 */

module;

#include <cstdint>
#include <cmath>

#define EMDEVIF_MODULE_INTERFACE_UNIT

export module rmdev.control_algorithm.pid;

import rmdev.math;
import emdevif.core.concepts;

#ifdef __clang__
    #pragma clang diagnostic ignored "-Winclude-angled-in-module-purview"
#endif

#include "rmdev/control_algorithm/pid.hpp"
