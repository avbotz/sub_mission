/*
 *  @file util.hpp
 *  @brief Various helper functions to simplify mission code.
 */

#ifndef MISSION_UTIL_HPP
#define MISSION_UTIL_HPP

#include <cmath>
#include <chrono>
#include "sub_mission/client.hpp"
#include "sub_vision/config.hpp"
#include "sub_control_interfaces/msg/state.hpp"

// Simplify so don't have to type entire path each time
using sub_control_interfaces::msg::State;

State create_state(float, float, float, float, float, float);
std::string state_to_text(State);
float seconds_since_start(std::chrono::time_point<std::chrono::high_resolution_clock>);

#endif