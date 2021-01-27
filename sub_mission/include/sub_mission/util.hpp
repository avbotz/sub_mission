#include <cmath>
#include "sub_mission/client.hpp"
#include "sub_control_interfaces/msg/state.hpp"

using namespace sub_control_interfaces::msg;

State create_state(float, float, float, float, float, float);
void sleep(float);