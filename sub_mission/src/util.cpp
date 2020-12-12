#include <cmath>
#include <chrono>
#include "sub_control_interfaces/msg/state.hpp"
#include "sub_mission/client.hpp"

#include "sub_mission/util.hpp"

using namespace std::chrono_literals;
using namespace sub_control_interfaces::msg;

// Convenience function to create State
// mirrors how old Porpoise syntax worked
State create_state(float x, float y, float z, float yaw, float pitch, float roll)
{
    State state;
    state.x = x;
    state.y = y;
    state.z = z;
    state.yaw = yaw;
    state.pitch = pitch;
    state.roll = roll;

    return state;
}

/**
 * angle_difference calculates normalized
 * angle differences in degrees, in the range [-180, 180]
 * (e.g. for angles 1 and 179 it will return 2 degrees, not 178).
 *
 * @param a1 first angle
 * @param a2 second angle
 */
float angle_difference(float a1, float a2)
{
    float diff = a1 - a2; // Get raw diff
    if (std::fabs(diff) > 180.)
    {
        if (a1 < a2)
            a1 += 360.;
        else
            a2 += 360.;
    }

    return diff;
}

/**
 * Cheap, synchronous and blocking move command
 * will basically write a state command
 * and then block until it gets there
 * would rather not use it if possible, but for a
 * cheap solution here
 *
 * For an async solution, simply directly use control_client::write_state
 */
void move_sync(std::shared_ptr<rclcpp::Node> node, State target_state)
{
    std::cout << "target " << target_state.x << " " << target_state.y  << " " << target_state.z << std::endl;
    control_client::write_state(target_state);
    std::this_thread::sleep_for(500ms);

    while (rclcpp::ok())
    {
        // Get current sub state
        State current_state = control_client::state();

        // If state is reached, stop blocking
        if (std::fabs(target_state.x - current_state.x) > 1) {}
        else if (std::fabs(target_state.y - current_state.y) > 1) {}
        else if (angle_difference(target_state.yaw, current_state.yaw) > 1) {}
        else { break; }

        // Otherwise, sleep 0.5s
        // TODO: Perhaps overhaul sub to use pub/sub telemetry?
        std::this_thread::sleep_for(500ms);
    }

    std::cout << "State reached!" << std::endl;
}
