/** @file functions.hpp
 *  @brief Function definitions for each task process during competition.
 *
 *  @author David Zhang
 */
#ifndef MISSION_FUNCTIONS_HPP
#define MISSION_FUNCTIONS_HPP 

#include "rclcpp/rclcpp.hpp"
#include "sub_vision_interfaces/srv/vision.h"

void vision_test();
void pid_tuning_sequence();
void gate();
void gate_extra();
void gate_extra_vision();
void bins();
void target();
void octagon();
void gate_debug();

#endif
