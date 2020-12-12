/** @file prelim.cpp
 *  @brief Lifecycle node to run prelim task.
 *
 *  @author Vincent Wang
 */
#include <cmath>
#include <functional>
#include <chrono>
#include <iostream>
#include <queue>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "sub_mission_interfaces/srv/current_target_state.hpp"
#include "sub_mission_interfaces/srv/set_current_target_state.hpp"
#include "sub_mission_interfaces/srv/get_states_len.hpp"
#include "sub_mission_interfaces/srv/queue_state.hpp"
#include "sub_mission/util.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rcutils/logging_macros.h"

#include "sub_mission/client.hpp"

// TODO: May not be needed - this likely won't be included elsewhere
// Instead, we'll communicate via service calls
// #include "sub_mission/tasks/prelim.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace rclcpp_lifecycle::node_interfaces;
using namespace sub_mission_interfaces::srv;

/** Prelim, implemented as a lifecycle node
 * ROS2 lifecycle nodes are a special type of node
 * that allows the system to react to "state changes"
 * in the node (e.g. error, activation/deactivation).
 *
 * Here, we're using it to implement the prelim task, which
 * is basically just moving the sub around a bit.
 *
 * The way this is implemented is basically:
 * - The node will configure itself (init clients if needed)
 *   - States will be queued in a list
 * - Wait for activation signal from master
 *   - On activate, it will start a control loop
 *   - If one desired state is reached, it will move on to the next
 *   - When the task is finished, node will shutdown
 * - Node may be deactivated at any time by external command
 *   - This has the effect of disabling control loop and issuing a
 *     hold-in-place command, effectively pausing the mission in place
 * - When state queue is empty, shutdown the node (finished)
 *
 * Note that this is SIGNIFICANTLY more complex than the original prelim
 * code (which basically just moved the sub procedurally). This more
 * complex layout is not necessarily NEEDED for this specific task, but
 * is sort of an experiment in how we could implement an architecturally
 * clean and impressive prelim system.
 *
 * TODO: Evaluate whether this is too complex
 */
class PrelimNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /**
     * PrelimNode constructor
     * @param intra_comms whether to use intra-process communication.
     *                    This mode is used when high-throughput communication
     *                    between nodes is needed. Puts nodes in the same process.
     *                    Results in some limitations (same process).
     */
    explicit PrelimNode(bool intra_comms = false)
    : rclcpp_lifecycle::LifecycleNode("prelim_node",
            rclcpp::NodeOptions().use_intra_process_comms(intra_comms))
    { }

    /**
     * Control loop, which handles actually moving the sub.
     * Basically this has a queue of sub states to move to
     * in order, and checks on execution whether to move on to
     * next state.
     * The advantage of implementing it like this is we can
     * pause the loop (and therefore the sub)
     * whenever the lifecycle node deactivates.
     *
     * However, for prelim itself, this is probably overkill,
     * and more a proof of concept to get people familiarized with
     * lifecycle nodes and their usefulness for complex tasks rather
     * than being completely practical.
     */
    void control_loop()
    {
        // Get states
        State target_state = this->current_target_state;
        State current_state = control_client::state();

        // If states are not yet reached, end early
        if (std::fabs(target_state.x - current_state.x) > 1.)
            return;
        if (std::fabs(target_state.y - current_state.y) > 1.)
            return;
        if (angle_difference(target_state.yaw, current_state.yaw))
            return;

        this->current_target_state = this->state_queue.front();
        this->state_queue.pop();
        control_client::write_state(this->current_target_state);
    }

    /**
     * current_target_state_srv_ callback, used to respond to
     * CurrentTargetState service request
     *
     * Gets target state
     */
    void current_target_state_srv_(const std::shared_ptr<CurrentTargetState::Request>,
                                   std::shared_ptr<CurrentTargetState::Response> response)
    {
        response->x = this->current_target_state.x;
        response->y = this->current_target_state.y;
        response->z = this->current_target_state.z;
        response->yaw = this->current_target_state.yaw;
        response->pitch = this->current_target_state.pitch;
        response->roll = this->current_target_state.roll;
    }

    /**
     * set_current_target_state_srv_ callback, used to respond to
     * SetCurrentTargetState service request
     *
     * Sets current target state
     */
    void set_current_target_state_srv_(const std::shared_ptr<SetCurrentTargetState::Request> request,
                                       std::shared_ptr<SetCurrentTargetState::Response>)
    {
        State state;
        state.x = request->x;
        state.y = request->y;
        state.z = request->z;
        state.yaw = request->yaw;
        state.pitch = request->pitch;
        state.roll = request->roll;

        this->set_current_target_state(state);
    }

    /**
     * queue_state_srv_ callback, used to respond to
     * QueueState service request
     *
     * Enqueues a state (pushes state onto end of queue)
     */
    void queue_state_srv_(const std::shared_ptr<QueueState::Request> request,
                          std::shared_ptr<QueueState::Response>)
    {
        State state;
        state.x = request->x;
        state.y = request->y;
        state.z = request->z;
        state.yaw = request->yaw;
        state.pitch = request->pitch;
        state.roll = request->roll;

        this->state_queue.push(state);
    }

    /**
     * get_states_len_srv_ callback, used to respond to
     * GetStatesLen service request
     *
     * Returns length of state queue
     */
    void get_states_len_srv_(const std::shared_ptr<GetStatesLen::Request>,
                          std::shared_ptr<GetStatesLen::Response> response)
    {
        response->len = this->state_queue.size();
    }

    /**
     * on_configure callback, called when node transitions to CONFIGURED
     * state. This basically sets everything up for us.
     *
     * We return one of SUCCESS, ERROR, FAILURE, which will transition us
     * to the respective state (handled by node master).
     */
    LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
        // Assuming clients are already alive
        // If not, then error
        if (!control_client::alive_client ||
            !control_client::write_client ||
            !control_client::state_client ||
            !control_client::depth_client ||
            !control_client::write_state_client ||
            !control_client::write_depth_client)
        {
            RCLCPP_ERROR(this->get_logger(), "some control clients not properly initialized! Prelim will not run.");
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        // Init services
        current_target_state_service =
            this->create_service<CurrentTargetState>("current_target_state_service",
                    std::bind(&PrelimNode::current_target_state_srv_, this, _1, _2));
        set_current_target_state_service =
            this->create_service<SetCurrentTargetState>("set_current_target_state_service",
                    std::bind(&PrelimNode::set_current_target_state_srv_, this, _1, _2));
        get_states_len_service =
            this->create_service<GetStatesLen>("get_states_len_service",
                    std::bind(&PrelimNode::get_states_len_srv_, this, _1, _2));
        queue_state_service =
            this->create_service<QueueState>("queue_state_service",
                    std::bind(&PrelimNode::queue_state_srv_, this, _1, _2));

        // Initialize and start control loop
        // Loop itself will only execute if state = ACTIVE, so no
        // problem if we start it here
        // 1s <- std::chrono-literals
        control_timer = this->create_wall_timer(500ms,
                std::bind(&PrelimNode::control_loop, this));

        // Enqueue states
        // The flexibility of our approach here shows in how
        // we could basically dynamically insert target states
        // however we wanted
        // However, we don't need that functionality (yet), so hardcoding it is!
        this->state_queue.push(State(-3, -1, 0, 0, 0, 0));
        this->state_queue.push(State(3, 1, 0, 0, 0, 0));
        this->state_queue.push(State(1, 0, 0, 0, 0, 0));
        this->state_queue.push(State(-1, 0, 0, 0, 0, 0));

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }


    /**
     * on_activate callback, called when node transitions to
     * ACTIVE state. This (re)starts the mission by setting
     * target state to first enqueued state.
     */
    LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &)
    {
        this->set_current_target_state(this->state_queue.front());

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * on_deactivate callback, called when node transitions to
     * INACTIVE state. This pauses the mission by setting
     * target state to current state. (control loop still runs,
     * but as the sub is holding position (i.e. not reaching next target state)
     * it will not do anything until reactivated).
     */
    LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
    {
        this->current_target_state = control_client::state();
        control_client::write_state(this->current_target_state);

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * on_cleanup callback, called when node enters cleaningup state.
     * This transition is from INACTIVE stated, and
     * leads to either UNCONFIGURED on success, or stays INACTIVE on failure.
     * Here we release our shared pointer to the timer and clear states.
     */
    LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        this->control_timer.reset(); // release timer shared ptr
        std::queue<State>().swap(this->state_queue); // clear queue
        this->current_target_state = State(0, 0, 0, 0, 0, 0); // clear current target state

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * on_shutdown callback, called when node shuts down.
     * Here we release our shared pointer to the timer and clear states.
     */
    LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &)
    {
        this->control_timer.reset(); // release timer shared ptr
        std::queue<State>().swap(this->state_queue); // clear queue
        this->current_target_state = State(0, 0, 0, 0, 0, 0); // clear current target state

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * Get reference to state queue.
     */
    std::queue<State>& get_state_queue()
    {
        return this->state_queue;
    }

    /**
     * Get current target state.
     */
    State get_current_target_state()
    {
        return this->current_target_state;
    }

    /**
     * Set current target state.
     *
     * @param newstate New state to set.
     */
    void set_current_target_state(State newstate)
    {
        this->current_target_state = newstate;
        control_client::write_state(this->current_target_state);
    }


private:
        // Declare wall timer for control loop callback
        std::shared_ptr<rclcpp::TimerBase> control_timer;

        // Declare service servers
        rclcpp::Service<CurrentTargetState>::SharedPtr current_target_state_service;
        rclcpp::Service<SetCurrentTargetState>::SharedPtr set_current_target_state_service;
        rclcpp::Service<GetStatesLen>::SharedPtr get_states_len_service;
        rclcpp::Service<QueueState>::SharedPtr queue_state_service;

        // Queue containing states to move to
        std::queue<State> state_queue;

        // Current state target
        State current_target_state;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<PrelimNode> prelim_node =
        std::make_shared<PrelimNode>();
    control_client::init_clients(prelim_node->get_node_base_interface());

    std::cout << "Starting prelim!" << "\n";

    exe.add_node(prelim_node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
