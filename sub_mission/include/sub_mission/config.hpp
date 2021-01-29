/** @file sub_mission/include/mission/config.hpp
 *  @brief Configuration and constants for mission.
 *
 *  @author David Zhang
 *  @author Craig Wang
 */
#ifndef MISSION_CONFIG_HPP
#define MISSION_CONFIG_HPP

const bool SIM = false;

const bool POOL_BC = true;
const bool GATE_LEFT = false;

// Down camera is 19 cm left of the DVL
const float DOWN_CAM_OFFSET = -0.19;

// Dropper center is 0 cm in front of the down camera's center
const float DROPPER_X_OFFSET = 0.;

// Dropper center is 7.5 cm to the left of the down camera's center
const float DROPPER_Y_OFFSET = -0.075;

// 2 balls; one is 3 cm in front of midline, other is 3 cm behind midline
const float BALL_OFFSET[2] = { 0.03, -0.03 };

// Constants to represent each DOF
const int X = 0;
const int Y = 1;
const int Z = 2;
const int YAW = 3;
const int PITCH = 4;
const int ROLL = 5;
const int N = 6;

// Conversions between degrees and radians
const float D2R = M_PI/180.;
const float R2D = 180./M_PI;

#endif
