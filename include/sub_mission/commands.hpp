/** @file commands.hpp
 *  @brief Function definitions for generic competition actions.
 *
 *  @author David Zhang
 *  @author Suhas Nagar
 */
#ifndef MISSION_COMMANDS_HPP
#define MISSION_COMMANDS_HPP

#include <tuple>
#include <ros/ros.h>
#include <vision/Vision.h>
#include "control/state.hpp"
#include "vision/observation.hpp"

typedef std::pair<float, float> Coordinate;

float angleDifference(float, float);
float angleAdd(float, float);
float align(int, Task, int);
float distance(int, Task, int);
Coordinate downAlign(int, Task, int);
void setForward(float);
void setHorizontal(float);
void setForwardAtDepth(float, float);
void setHorizontalAtDepth(float, float);
void setAngle(float);
void addAngle(float);
bool isValidAngle(float);
void setCoordinate(Coordinate);
void addCoordinate(Coordinate);
bool isValidOffsetCoordinate(Coordinate);
bool isValidCoordinate(Coordinate);
void move(const State &); 
float distAndAlign(int, Task, int, float);

#endif
