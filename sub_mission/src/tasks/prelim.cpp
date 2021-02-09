#include <iostream>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/msg/state.hpp"
#include "sub_mission/util.hpp"
#include "sub_mission/client.hpp"

using namespace std::chrono_literals;
using namespace sub_control_interfaces::msg;

int main(int argc, char** argv)
{
    // Init node
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Init node");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("prelim_node");

    // Init control clients
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Init control clients");
    control_client::init_clients(node);

    // Wait for kill switch
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Wait for kill");
    while (rclcpp::ok() && !control_client::alive())
    {
        RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Sub is not alive yet, waiting...");
        std::this_thread::sleep_for(500ms);
    }

    // Turn on thrusters
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Set power to 0.2");
    control_client::write("p 0.2\n");

    // Run prelim
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting prelim!");
    move(create_state(-3, -1, 0, 0, 0, 0));
    move(create_state(3, 1, 0, 0, 0, 0));
    move(create_state(1, 0, 0, 0, 0, 0));
    move(create_state(-1, 0, 0, 0, 0, 0));

    return 0;
}
