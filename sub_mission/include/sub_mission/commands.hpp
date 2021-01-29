/** @file commands.hpp
 *  @brief Function definitions for generic competition actions.
 *
 *  @author David Zhang
 *  @author Suhas Nagar
 */
#ifndef MISSION_COMMANDS_HPP
#define MISSION_COMMANDS_HPP

#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "sub_vision_interfaces/srv/vision.h"
#include "sub_control_interfaces/msg/state.hpp"
#include "sub_vision/observation.hpp"

using namespace std::chrono_literals;
using namespace sub_control_interfaces::msg;

typedef std::pair<float, float> Coordinate;

float angleDifference(float, float);
float angleAdd(float, float);
float align(int, Task, int);
float relativeAlign(int, Task, int);
float distance(int, Task, int);
float distAndAlign(int, Task, int);
bool isValidDistance(float);
State forwardAlign(float, Task, int, float);
Coordinate downAlign(int, Task, int);
bool frontContinuousAlign(float, Task, int, float);
bool frontContinuousValign(float, Task, int, float);
bool frontContinuousHalign(float, Task, int, float);
bool frontConsecutiveAngleAlign(int, Task, int);
bool downContinuousAlign(float, Task, int, float);
bool testObjIsCentered(int, Task, int, float);
bool objIsCentered(float, float, int, float);
bool isValigned(float, int, float);
bool isHaligned(float, int, float);
void setForward(float);
void setHorizontal(float);
void setForwardAtDepth(float, float);
void setHorizontalAtDepth(float, float);
void setAngle(float);
void addAngle(float);
void continuousSpin(int);
bool isValidAngle(float);
void setCoordinate(Coordinate);
void addCoordinate(Coordinate);
bool isValidCoordinate(Coordinate);
bool isValidOffsetCoordinate(Coordinate);
void warmupInference(int, Task, int);
void disableAltitudeControl();
void move(const State &); 

#endif