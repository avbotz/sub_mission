#include <cmath>
#include "sub_mission/client.hpp"
#include "sub_control_interfaces/msg/state.hpp"

using namespace sub_control_interfaces::msg;

// Constants to represent each DOF
const int X = 0;
const int Y = 1;
const int Z = 2;
const int YAW = 3;
const int PITCH = 4;
const int ROLL = 5;
const int N = 6;

State create_state(float, float, float, float, float, float);
std::string state_to_text(State);
void sleep(float);