/* Main mission node code.
 * Creates necessary connections to control_node
 * and vision_node and executes the necessary function.
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sub_control_interfaces/srv/control_alive.hpp"
#include "sub_control_interfaces/srv/control_depth.hpp"
#include "sub_control_interfaces/srv/control_state.hpp"
#include "sub_control_interfaces/srv/control_write.hpp"
#include "sub_control_interfaces/srv/control_write_depth.hpp"
#include "sub_control_interfaces/srv/control_write_state.hpp"
#include "sub_control_interfaces/msg/state.hpp"

#include "sub_mission/client.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

// Alias namespace here just because it's really long to type
namespace control = sub_control_interfaces::srv;

namespace control_client
{
	std::shared_ptr<rclcpp::Node> node;

    rclcpp::Client<control::ControlAlive>::SharedPtr alive_client;
    rclcpp::Client<control::ControlState>::SharedPtr state_client;
    rclcpp::Client<control::ControlWrite>::SharedPtr write_client;
    rclcpp::Client<control::ControlDepth>::SharedPtr depth_client;
    rclcpp::Client<control::ControlWriteState>::SharedPtr write_state_client;
    rclcpp::Client<control::ControlWriteDepth>::SharedPtr write_depth_client;

    bool alive()
    {
        auto request = std::make_shared<control::ControlAlive::Request>();
        auto result = alive_client->async_send_request(request);

        rclcpp::spin_until_future_complete(node, result);

        return result.get()->data;
    }

    State state()
    {
        auto request = std::make_shared<control::ControlState::Request>();
        auto result = state_client->async_send_request(request);

        rclcpp::spin_until_future_complete(node, result);

        State state;
        state.x = result.get()->x;
        state.y = result.get()->y;
        state.z = result.get()->z;
        state.yaw = result.get()->yaw;
        state.pitch = result.get()->pitch;
        state.roll = result.get()->roll;
        return state;
    }

    float depth()
    {
        auto request = std::make_shared<control::ControlDepth::Request>();
        auto result = depth_client->async_send_request(request);

        rclcpp::spin_until_future_complete(node, result);

        return result.get()->depth;
    }

    void write(std::string input)
    {
        auto request = std::make_shared<control::ControlWrite::Request>();
        request->data = input;

        auto result = write_client->async_send_request(request);
    }

    void write_state(const State &state)
    {
        auto request = std::make_shared<control::ControlWriteState::Request>();
        request->x = state.x;
        request->y = state.y;
        request->z = state.z;
        request->yaw = state.yaw;
        request->pitch = state.pitch;
        request->roll = state.roll;
        auto result = write_state_client->async_send_request(request);
    }

    void write_depth(float dist)
    {
        auto request = std::make_shared<control::ControlWriteDepth::Request>();
        request->dist = dist;

        auto result = write_depth_client->async_send_request(request);
    }
    void init_clients(std::shared_ptr<rclcpp::Node> new_node)
    {
        // TODO: Add a vision subscriber as well
        // TODO: Eventually, pub/sub based API/architecture for this?
        node = new_node;

        control_client::alive_client =
            node->create_client<sub_control_interfaces::srv::ControlAlive>("control_alive");
        control_client::state_client =
            node->create_client<sub_control_interfaces::srv::ControlState>("control_state");
        control_client::depth_client =
            node->create_client<sub_control_interfaces::srv::ControlDepth>("control_depth");
        control_client::write_client =
            node->create_client<sub_control_interfaces::srv::ControlWrite>("control_write");
        control_client::write_state_client =
            node->create_client<sub_control_interfaces::srv::ControlWriteState>("control_write_state");
        control_client::write_depth_client =
            node->create_client<sub_control_interfaces::srv::ControlWriteDepth>("control_write_depth");
    }
}

