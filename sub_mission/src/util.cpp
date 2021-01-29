/*
 * Various helper functions to simplify mission code 
 */
#include "sub_mission/util.hpp"

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

float seconds_since_start(std::chrono::time_point<std::chrono::high_resolution_clock> start_time)
{
    /* 
     * Input is the output of std::chrono::high_resolution_clock::now()
     * Use this function to calculate time elapsed since the start time 
     * from your instance of std::chrono::high_resolution_clock::now()
     */
    auto stop_time = std::chrono::high_resolution_clock::now();
    float nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_time - start_time).count();
    float seconds = nanoseconds / 1000000000.;
    return seconds;
}