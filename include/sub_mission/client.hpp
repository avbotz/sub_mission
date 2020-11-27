/** @file client.hpp
 *  @brief Wrapper namespaces for using ROS clients.
 *
 *  @author David Zhang
 */
#ifndef MISSION_CLIENT_HPP
#define MISSION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/control_alive.hpp"
#include "sub_control_interfaces/control_state.hpp"
#include "sub_control_interfaces/control_depth.hpp"
#include "sub_control_interfaces/control_write.hpp"
#include "sub_control_interfaces/control_write_state.hpp"
#include "sub_control_interfaces/control_depth.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

// #include "vision/Vision.h"
// #include "control/state.hpp"
// #include "vision/observation.hpp"

// namespace vision_client
// {
// 	extern ros::ServiceClient client;
//
// 	Observation vision(Task, int);
// };

struct State {
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
}

namespace control_client
{
	extern rclcpp::Client<sub_control_interfaces::srv::ControlAlive> alive_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlState> state_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlDepth> depth_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlWrite> write_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlWriteState> write_state_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlWriteDepth> write_depth_client;

	bool alive();
	State state();
	float depth();
	void write(std::string);
	void writeState(const State&);
	void writeDepth(float);
};

#endif
