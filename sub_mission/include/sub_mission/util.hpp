#include <cmath>
#include "sub_mission/client.hpp"
#include "sub_control_interfaces/msg/state.hpp"

using namespace sub_control_interfaces::msg;

State create_state(float x, float y, float z, float yaw, float pitch, float roll);

float angle_difference(float a1, float a2);

void move_sync(std::shared_ptr<rclcpp::Node> node, State state);
