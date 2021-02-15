#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/msg/state.hpp"
#include "sub_mission/functions.hpp"
#include "sub_mission/commands.hpp"
#include "sub_mission/client.hpp"
#include "sub_mission/util.hpp"

using namespace std::chrono_literals;
using namespace sub_control_interfaces::msg;

int main(int argc, char** argv)
{
    // Init node
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Init node");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_node");

    // Init control and vision clients
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Init control and vision clients");
    control_client::init_clients(node);
    vision_client::init_clients(node);

    // Wait for kill switch
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Wait for kill");
    while (rclcpp::ok() && !control_client::alive())
    {
        RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Sub is not alive yet, waiting...");
        std::this_thread::sleep_for(0.5s);
    }

    // Turn on thrusters
    RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Set power");
    control_client::set_power(0.2);

    // Run mission functions.

    /* Supported in sim */
    gate_extra_vision();
    bins();
    target();
    octagon();
    /* ////////////////////////////////////// */

    // gate();
    // gate_extra();
    // gate_debug();
    // gate_extra_vision();
    // bins();
    // target();
    // octagon(); 
    // vision_test();
    // pid_tuning_sequence();

    return 0;
}

