/** @file functions.hpp
 *  @brief Function definitions for each task process during competition.
 *
 *  @author David Zhang
 */
#ifndef MISSION_FUNCTIONS_HPP
#define MISSION_FUNCTIONS_HPP 

#include <ros/ros.h>
#include <vision/Vision.h>

void vision_test();
void gate();
void gate_extra();
void gate_extra_vision();
void bins();
void target();
void octagon();
void gate_debug();

#endif
