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
    std::cout << "init node" << std::endl;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_node");

    // Init control client
    std::cout << "init control clients" << std::endl;
    control_client::init_clients(node);

    // Wait a second for sub to come up
    std::this_thread::sleep_for(2s);

    std::cout << "set power to 0.2" << std::endl;
    control_client::write("p 0.2\n");

    // Wait a second for sub to come up
    std::this_thread::sleep_for(3s);

    // Wait for kill switch
    std::cout << "wait for kill" << std::endl;
    while (rclcpp::ok() && !control_client::alive())
    {
        std::cout << "Sub is not yet alive, waiting..." << std::endl;
        std::this_thread::sleep_for(500ms);
    }

    // Run prelim
    std::cout << "Started prelim!" << std::endl;
    move(create_state(-3, -1, 0, 0, 0, 0));
    move(create_state(3, 1, 0, 0, 0, 0));
    move(create_state(1, 0, 0, 0, 0, 0));
    move(create_state(-1, 0, 0, 0, 0, 0));

    return 0;
}
