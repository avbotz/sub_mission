#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/msg/state.hpp"
#include "sub_mission/functions.hpp"
#include "sub_mission/commands.hpp"
#include "sub_mission/client.hpp"
#include "sub_mission/util.hpp"

using namespace std::chrono_literals;
using namespace sub_control_interfaces::msg;


bool SIM;

int main(int argc, char** argv)
{
    // Init node
    std::cout << "Init node" << std::endl;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_node");

    // Check if user has turned sim on, default is false
    node->declare_parameter("SIM", false);
    node->get_parameter("SIM", SIM);
    std::cout << "Is simulation on: " << std::boolalpha << SIM << std::endl;

    // Init control and vision clients
    std::cout << "Init control and vision clients" << std::endl;
    control_client::init_clients(node);
    vision_client::init_clients(node);

    // Wait for kill switch
    std::cout << "Wait for kill" << std::endl;
    while (rclcpp::ok() && !control_client::alive())
    {
        std::cout << "Sub is not alive yet, waiting..." << std::endl;
        std::this_thread::sleep_for(0.5s);
    }

    // Turn on thrusters
    std::cout << "Set power" << std::endl;
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

