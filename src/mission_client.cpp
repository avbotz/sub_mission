/* Main mission node code.
 * Creates necessary connections to control_node
 * and vision_node and executes the necessary function.
 */
#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/srv/control_alive.hpp"
#include "sub_control_interfaces/srv/control_depth.hpp"
#include "sub_control_interfaces/srv/control_state.hpp"
#include "sub_control_interfaces/srv/control_write.hpp"
#include "sub_control_interfaces/srv/control_write_depth.hpp"
#include "sub_control_interfaces/srv/control_write_state.hpp"

#include "mission/functions.hpp"
#include "mission/client.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv, "mission_node");
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mission_node");

    control_client::alive_client =
        node->create_client<sub_control_interfaces::srv::ControlAlive>("control_alive");
    control_client::state_client =
        node->create_client<sub_control_interfaces::srv::ControlState>("control_state");
    control_client::depth_client =
        node->create_client<sub_control_interfaces::srv::ControlAlive>("control_depth");
    control_client::write_client =
        node->create_client<sub_control_interfaces::srv::ControlWrite>("control_write");
    control_client::write_state_client =
        node->create_client<sub_control_interfaces::srv::ControlWriteState>("control_write_state");
    control_client::write_depth_client =
        node->create_client<sub_control_interfaces::srv::ControlWriteDepth>("control_write_depth");
}
