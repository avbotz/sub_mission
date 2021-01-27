/** @file client.hpp
 *  @brief Wrapper namespaces for using ROS clients.
 *
 *  @author David Zhang
 */
#ifndef MISSION_CLIENT_HPP
#define MISSION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sub_control_interfaces/srv/control_alive.hpp"
#include "sub_control_interfaces/srv/control_state.hpp"
#include "sub_control_interfaces/srv/control_depth.hpp"
#include "sub_control_interfaces/srv/control_write.hpp"
#include "sub_control_interfaces/srv/control_write_state.hpp"
#include "sub_control_interfaces/srv/control_write_depth.hpp"
#include "sub_control_interfaces/msg/state.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace sub_control_interfaces::msg;

// #include "vision/Vision.h"
// #include "control/state.hpp"
// #include "vision/observation.hpp"

// namespace vision_client
// {
// 	extern ros::ServiceClient client;
//
// 	Observation vision(Task, int);
// };

namespace control_client
{
    extern std::shared_ptr<rclcpp::Node> node;

	extern rclcpp::Client<sub_control_interfaces::srv::ControlAlive>::SharedPtr alive_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlState>::SharedPtr state_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlDepth>::SharedPtr depth_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlWrite>::SharedPtr write_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlWriteState>::SharedPtr write_state_client;
	extern rclcpp::Client<sub_control_interfaces::srv::ControlWriteDepth>::SharedPtr write_depth_client;

	bool alive();
	State state();
	float depth();
	void write(std::string);
	void write_state(const State&);
	void write_depth(float);

	void init_clients(std::shared_ptr<rclcpp::Node> new_node);
}

#endif
