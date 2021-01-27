/**
 * @file mission_controller.cpp
 * @brief mission controller for prelim.
 * @author Vincent Wang
 */
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "sub_mission_interfaces/srv/current_target_state.hpp"
#include "sub_mission_interfaces/srv/set_current_target_state.hpp"
#include "sub_mission_interfaces/srv/queue_state.hpp"
#include "sub_mission_interfaces/srv/get_states_len.hpp"

#include "sub_mission/client.hpp"

using namespace sub_mission_interfaces::srv;
using namespace std::chrono_literals;
using lifecycle_msgs::msg::Transition;

rclcpp::Client<CurrentTargetState>::SharedPtr get_target_state_client;
rclcpp::Client<SetCurrentTargetState>::SharedPtr set_target_state_client;
rclcpp::Client<GetStatesLen>::SharedPtr get_states_len_client;
rclcpp::Client<QueueState>::SharedPtr queue_states_client;
rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_node_state_client;
rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_node_state_client;

bool transition(std::shared_ptr<rclcpp::Node> node, uint8_t transition)
{
    // If there's no more states, deactivate
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = change_node_state_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result);
    return result.get()->success;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_controller_service");

    // init control client
    control_client::init_clients(node);

    // Init mission service clients
    get_target_state_client = node->create_client<CurrentTargetState>(
            "prelim_node/current_target_state_service"
    );
    set_target_state_client = node->create_client<SetCurrentTargetState>(
            "prelim_node/set_current_target_state_service"
    );
    get_states_len_client = node->create_client<GetStatesLen>(
            "prelim_node/set_current_target_state_service"
    );
    queue_states_client = node->create_client<QueueState>(
            "prelim_node/set_current_target_state_service"
    );

    get_node_state_client = node->create_client<lifecycle_msgs::srv::GetState>(
            "prelim_node/get_state"
    );
    change_node_state_client = node->create_client<lifecycle_msgs::srv::ChangeState>(
            "prelim_node/change_state"
    );

    // configure node
    while (!transition(node, Transition::TRANSITION_CONFIGURE))
    {
        // Try and configure every 500ms if unsuccessful
        std::cerr << "Configuring prelim node, unsuccessful, trying again in 500ms..."
        << std::endl;

        rclcpp::sleep_for(500ms);
    };

    auto last_alive = control_client::alive();
    while (rclcpp::ok())
    {
        // Get current state
        auto request = std::make_shared<GetStatesLen::Request>();
        auto result = get_states_len_client->async_send_request(request);
        rclcpp::spin_until_future_complete(node, result);

        auto alive = control_client::alive();
        if (last_alive && !alive)
        {
            // sub is no longer alive, deactivate
            transition(node, Transition::TRANSITION_DEACTIVATE);
            last_alive = alive;
        } else if (!last_alive && alive)
        {
            // sub is alive, activate
            transition(node, Transition::TRANSITION_ACTIVATE);
        }

        last_alive = alive;

        if (result.get()->len == 0)
        {
            // no more state, deactivate
            transition(node, Transition::TRANSITION_DEACTIVATE);
        }

        rclcpp::sleep_for(500ms);
        rclcpp::spin_some(node);
    }
    return 0;
}
