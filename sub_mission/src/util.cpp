#include <cmath>
#include <chrono>
#include "sub_control_interfaces/msg/state.hpp"
#include "sub_mission/client.hpp"

#include "sub_mission/util.hpp"

using namespace std::chrono_literals;
using namespace sub_control_interfaces::msg;

State create_state(float x, float y, float z, float yaw, float pitch, float roll)
{
    /* Convenience function to create State. Mirros how old Porpoise syntax worked. */
    State state;
    state.x = x;
    state.y = y;
    state.z = z;
    state.yaw = yaw;
    state.pitch = pitch;
    state.roll = roll;

    return state;
}

std::string state_to_text(State state)
{
    /* Display State's information as a string */
    std::ostringstream os;
    os.precision(2);
    os << std::fixed;
    os << "(";
    os << state.x << " " << state.y << " " << state.z << " "
        << state.yaw << " " << state.pitch << " " << state.roll;
    os << ")";
    return os.str();
}

void sleep(float seconds)
{
    /* Sleep for specified amount of seconds */
    int nanoseconds = (int) seconds * 1000000000.;
    rclcpp::sleep_for(std::chrono::nanoseconds(nanoseconds));
}