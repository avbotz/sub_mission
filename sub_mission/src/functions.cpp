/** @file functions.cpp
 *
 *  @brief Functions for each task process during competition.
 *
 *  @author David Zhang
 *  @author Suhas Nagar
 *  @author Craig Wang
 */
#include <vector>
#include "sub_mission/functions.hpp"
#include "sub_mission/commands.hpp"
#include "sub_mission/client.hpp"
#include "sub_mission/config.hpp"
#include "sub_mission/euler.hpp"
#include "sub_mission/util.hpp"
#include "sub_vision/observation.hpp"

using namespace std::chrono_literals;

void vision_test()
{
	control_client::write_depth(2.);
	std::cout << "DIVED" << std::endl;
	downAlign(3, Task::BINS_ML, DOWN);
	std::cout << "DONE" << std::endl;
}

void pid_tuning_sequence()
{
	/* 
	 * Sequence of movements to tune pid gains for each degree of freedom.
	 * Make sure you have enough space in the pool or edit the movements accordingly.
	 */
	control_client::write_depth(1.);
	std::this_thread::sleep_for(10s);

	while (rclcpp::ok())
	{
		// Basically moves in a square
		std::cout << "Testing X" << std::endl;
		move(create_state(2., 0., 0., 0., 0., 0.));
		std::this_thread::sleep_for(8s);

		std::cout << "Testing Y" << std::endl;
		move(create_state(2., 2., 0., 0., 0., 0.));
		std::this_thread::sleep_for(8s);

		std::cout << "Testing Z" << std::endl;
		disableAltitudeControl();
		move(create_state(2., 2., 1., 0., 0., 0.));
		std::this_thread::sleep_for(8s);

		std::cout << "Testing YAW" << std::endl;
		move(create_state(2., 2., 1., 180., 0., 0.));
		move(create_state(2., 2., 1., 90., 0., 0.));
		move(create_state(2., 2., 1., 0., 0., 0.));
		move(create_state(2., 2., 1., -180., 0., 0.));
		move(create_state(2., 2., 1., -90., 0., 0.));
		move(create_state(2., 2., 1., 0., 0., 0.));
		std::this_thread::sleep_for(8s);

		std::cout << "Testing ALTITUDE" << std::endl;
		control_client::write_depth(1.);
		std::this_thread::sleep_for(12s);

		std::cout << "Returning to start" << std::endl;
		move(create_state(0., 0., 0., 0., 0., 0.));
		std::this_thread::sleep_for(8s);
	}
}

void gate()
{
	std::cout << "Beginning GATE function." << std::endl;

	std::cout << "Set initial state." << std::endl;
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	initial.z = 1.25;
	
	std::cout << "State @ " << state_to_text(initial) << std::endl;
	move(initial);
	std::this_thread::sleep_for(3s);

	std::cout << "Turn towards gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	float dist = distAndAlign(5, Task::GATE_ML, FRONT);

	/* Different from porpoise */
	if (dist == 0)
	{
		std::cout << "Failed to locate gate" << std::endl;
		setForward(4.0);
	}
	std::cout << "Turn towards gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	dist = distAndAlign(5, Task::GATE_ML, FRONT);
	/* Different from porpoise */
	
	// std::cout << "Angle @ " << angle << std::endl;
	// if (angle > -180. && angle < 180.) setAngle(angle);
	std::this_thread::sleep_for(1s);

	std::cout << "Go through gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	setForward(dist);
	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;
}

void gate_extra()
{
	std::cout << "Beginning GATE_EXTRA function." << std::endl;
	
	std::cout << "Set initial depth." << std::endl;
	control_client::write_depth(2.8);
	std::this_thread::sleep_for(6s);

	std::cout << "Go towards gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	setCoordinate(std::make_pair(6.5, 0));
	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;
	
	std::cout << "Set 40/60 depth." << std::endl;
	control_client::write_depth(3.4);
	std::this_thread::sleep_for(6s);
	
	std::cout << "Go through gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	setCoordinate(std::make_pair(13., 0));
	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;

	std::cout << "Spin in gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	for (int i = 0; i < 8; i++)
	{
		State temp = control_client::state();
		temp.yaw = angleAdd(temp.yaw, 90.);
		move(temp);
		std::cout << "Spin " << i+1 << " @ " << state_to_text(control_client::state()) << std::endl;
	}
	std::cout << "New State @ " << state_to_text(control_client::state()) << std::endl;
	
	std::cout << "Return to original angle." << std::endl;
	if (POOL_BC)
		setAngle(6.);
	if (!POOL_BC) 
		setAngle(-6.);
	std::this_thread::sleep_for(4s);

	std::cout << "Continue forward through gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	setForward(3.);
	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;
}

void gate_extra_vision()
{
	std::cout << "Beginning GATE_EXTRA_VISION function." << std::endl;
	
	std::cout << "Set initial depth." << std::endl;
	control_client::write_depth(2.7);
	std::this_thread::sleep_for(6s);

	std::cout << "Align and pass through gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	State desired = forwardAlign(15., Task::GATE_ML, FRONT, 120.);
	control_client::write_state(desired);
	std::this_thread::sleep_for(10s);

	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;

	std::cout << "Spin 2 meters after gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	for (int i = 0; i < 8; i++)
	{
		State temp = control_client::state();
		temp.yaw = angleAdd(temp.yaw, 90.);
		move(temp);
		std::cout << "Spin " << i+1 << " @ " << state_to_text(control_client::state()) << std::endl;
	}
	// continuousSpin(8);

	std::cout << "New State @ " << state_to_text(control_client::state()) << std::endl;
	
	std::cout << "Return to original angle." << std::endl;
	if (POOL_BC)
		setAngle(6.);
	if (!POOL_BC) 
		setAngle (-6.);
	std::this_thread::sleep_for(4s);

	std::cout << "Continue forward through gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	setForward(1.5);
	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;
}

void bins()
{
	std::cout << "Beginning BINS function." << std::endl;
	control_client::write_depth(2.);

	/*
	 * Used to navigate between a few meters in front of gate and bins.
	 */

	// Increased angle because sub crashed into the wall in sim
	std::cout << "Go towards bins, avoid target." << std::endl;
	if (POOL_BC) 
		setAngle(16.);
	if (!POOL_BC) 
		setAngle(-16.5);
	std::this_thread::sleep_for(6s);
	setForwardAtDepth(12.5, 1.);
	std::this_thread::sleep_for(3s);
	
	std::cout << "Set initial depth." << std::endl;
	control_client::write_depth(3.5);

//	std::cout << "Look for bins in pool." << std::endl;
	std::vector<Coordinate> search_offsets;
	// Use for pool B/C.
	/*
	search_offsets.push_back(std::make_pair(0., 0.));
	search_offsets.push_back(std::make_pair(-0.5, -0.5));
	search_offsets.push_back(std::make_pair(0.5, 0.5));
	search_offsets.push_back(std::make_pair(0., 0.5));
	search_offsets.push_back(std::make_pair(-0.5, 0.));
	search_offsets.push_back(std::make_pair(0.5, -0.5));
	*/
	// Use for pool A.
	search_offsets.push_back(std::make_pair(0., 0.));
	search_offsets.push_back(std::make_pair(-2., 2.));
	search_offsets.push_back(std::make_pair(0., -4.));
	search_offsets.push_back(std::make_pair(4., 0.));
	search_offsets.push_back(std::make_pair(0., 4.));
	search_offsets.push_back(std::make_pair(-2., -2.));
	// Use for pool D.
	/*
	search_offsets.push_back(std::make_pair(0., 0.));
	search_offsets.push_back(std::make_pair(5., -4.));
	search_offsets.push_back(std::make_pair(-4., 0.));
	search_offsets.push_back(std::make_pair(0., 4.));
	search_offsets.push_back(std::make_pair(4., 0.));
	search_offsets.push_back(std::make_pair(-2., -2.));
	*/
	for (int i = 0; i < search_offsets.size(); i++)
	{
		if (i != 0)
			addCoordinate(search_offsets[i]);
		std::this_thread::sleep_for(3s);
		std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
		bool isCentered = downContinuousAlign(10, Task::BINS_ML, DOWN, 0.3);
		if (isCentered)
		{
			std::cout << "Found initial bins set at index" << std::endl;
			break;
		}
	}

	std::cout << "Set second depth." << std::endl;
	control_client::write_depth(1.5);
	std::this_thread::sleep_for(8s);

	std::cout << "Find second offsets for bins." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	downContinuousAlign(10, Task::BINS_ML, DOWN, 0.1);

	std::cout << "Set third depth." << std::endl;
	control_client::write_depth(0.75);
	std::this_thread::sleep_for(6s);

	std::cout << "Find third offsets for bins." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	downContinuousAlign(10, Task::BINS_ML, DOWN, 0.1);
	
	std::cout << "Drop the balls." << std::endl;
	for (int i = 0; i < 2; i++)
	{
		control_client::write("g 0 1\n");
		std::this_thread::sleep_for(1s);
		control_client::write("g 1 1\n");
		std::this_thread::sleep_for(3s);
		control_client::write("g 0 0\n");
		control_client::write("g 1 0\n");
		std::this_thread::sleep_for(2s);
	}

	/*
	std::cout << "Reset depth." << std::endl;
	// control_client::write("z -1\n" << std::endl;
	control_client::write_depth(-1.);
	std::this_thread::sleep_for(6s);
	*/
}

void target()
{
	std::cout << "Beginning TARGET function." << std::endl;

	State return_state = control_client::state();

	std::cout << "Set initial state." << std::endl;
	State initial = control_client::state();
	initial.yaw = 180.;
	if (POOL_BC) 
		initial.yaw = angleAdd(initial.yaw, 10.);
	if (!POOL_BC)
		initial.yaw = angleAdd(initial.yaw, -10.);
	std::cout << "State @ " << state_to_text(initial) << std::endl;
	move(initial);

	std::cout << "Set initial depth." << std::endl;
	control_client::write_depth(1.65);
	std::this_thread::sleep_for(4s);

	/*
	std::cout << "Going towards the target." << std::endl;
	setForward(3.);
	std::this_thread::sleep_for(1s);
	*/
	/*
	if (POOL_BC)
		setHorizontal(-1.);
	if (!POOL_BC)
		setHorizontal(1.);
	*/
	setForward(4.);
	std::this_thread::sleep_for(2s);
	
	// Added wider angles because in some cases sub couldn't find target in sim
	std::cout << "Setup search angles." << std::endl;
	std::vector<float> search_angles;
	search_angles.push_back(0.);
	search_angles.push_back(30.);
	search_angles.push_back(30.);
	search_angles.push_back(-120.);
	search_angles.push_back(30.);
	search_angles.push_back(30.);

	std::cout << "First turn towards target." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	State original = control_client::state();

	// Sweep check for target until target in view
	bool inSight = false;
	for (int c = 0; c < 4; c++)
	{
		if (c != 0) setForward(2.);
		for (int i = 0; i < search_angles.size(); i++)
		{
			if (i != 0) addAngle(search_angles[i]);
			float angle = align(4, Task::TARGET_ML, FRONT);
			if (isValidAngle(angle))
			{
				std::cout << "Target found." << std::endl;
				std::cout << "Align with target." << std::endl;
				setAngle(angle);
				inSight = true;
				break;
			}
		}
		if (inSight) break;
	}

	if (inSight)
	{
		forwardAlign(12., Task::TARGET_ML, FRONT, 80.);
		std::this_thread::sleep_for(10s);

		std::cout << "Backup from target." << std::endl;
		std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
		float backup_dist = -6.;
		setForward(backup_dist);
	}
	else 
	{
		std::cout << "Could not find target." << std::endl;
		std::cout << "Returning to original state" << std::endl;
		move(original);
	}

	std::cout << "Return to angle." << std::endl;
	State angle_state = control_client::state();
	angle_state.yaw = 180.;
	if (POOL_BC) 
		angle_state.yaw = angleAdd(angle_state.yaw, 5.);
	if (!POOL_BC)
		angle_state.yaw = angleAdd(angle_state.yaw, -5.);
	std::cout << "State @ " << state_to_text(angle_state) << std::endl;
	move(angle_state);

	std::cout << "First turn towards second target." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;

	// Sweep check for target until target in view
	float hdist;
	if (POOL_BC) hdist = 2.;
	else hdist = -2.;

	int num_iters = 2;
	inSight = false;

	for (int c = 0; c < num_iters; c++)
	{
		if (c != 0) setForward(2.);
		for (int i = 0; i < search_angles.size(); i++)
		{
			if (i != 0) addAngle(search_angles[i]);
			float angle = align(4, Task::SECOND_TARGET_ML, FRONT);
			if (isValidAngle(angle))
			{
				std::cout << "Second target found." << std::endl;
				std::cout << "Align with second target." << std::endl;
				setAngle(angle);
				inSight = true;
				break;
			}
		}
		if (inSight) break;
	}

	if (inSight)
	{
		forwardAlign(12., Task::SECOND_TARGET_ML, FRONT, 80.);
		std::this_thread::sleep_for(10s);

		std::cout << "Backup from second target." << std::endl;
		std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
		float backup_dist = -6.;
		setForward(backup_dist);
	}
	else 
	{
		std::cout << "Could not find second target." << std::endl;
		std::cout << "Returning to original state" << std::endl;
		move(original);
	}

	std::cout << "Turn back towards octagon." << std::endl;
	if (POOL_BC) 
		setAngle(8.);
	if (!POOL_BC) 
		setAngle(-8.);
	std::this_thread::sleep_for(6s);

	move(return_state);
	std::this_thread::sleep_for(3s);
}

void octagon()
{
	std::cout << "Beginning OCTAGON function." << std::endl;

	std::cout << "Set initial state." << std::endl;
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	std::cout << "State @ " << state_to_text(initial) << std::endl;
	move(initial);

	std::cout << "Going towards octagon." << std::endl;
	if (!SIM)
	{
		setForward(5.);

		std::cout << "Surfacing." << std::endl;
		control_client::write("p 0\n");
	}
	else
	{
		// Hard code octagon location for sim, don't use this for actual hydrophone testing
		State octagon = create_state(34, 11, 0, 0, 0, 0);
		move(octagon);

		std::cout << "Surfacing." << std::endl;
		disableAltitudeControl();
		move(octagon);
		control_client::write("p 0\n");
	}
}

void gate_debug()
{
	std::cout << "Beginning GATE_DEBUG function." << std::endl;

	/*
	 * Each pressure of 1.23 is 1m below the surface.
	 */
	/*
	std::cout << "Set initial state." << std::endl;
	// State initial = control_client::state();
	State initial(0., 0., 0., 0., 0., 0.);
	initial.z = 0.75;
	std::cout << "State @ " << state_to_text(initial) << std::endl;
	move(initial);
	std::this_thread::sleep_for(4s);
	*/
	
	std::cout << "Set initial depth." << std::endl;
	control_client::write_depth(2.);
	std::this_thread::sleep_for(6s);

	std::cout << "Set initial forward distance." << std::endl;
	setForwardAtDepth(1.5, 0.75);
	std::this_thread::sleep_for(2s);

	std::cout << "Turn towards gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	float angle = align(3, Task::GATE_ML, FRONT);
	std::cout << "Angle @ " << angle << std::endl;
	if (isValidAngle(angle)) 
	{
		std::cout << "Found good angle for 40 percent adjustment." << std::endl;
		if (GATE_LEFT)
		{
			angle = angleAdd(angle, -0.);
			std::cout << "Choosing left side of gate for 40." << std::endl;
		}
		else
		{
			angle = angleAdd(angle, 0.);
			std::cout << "Choosing right side of gate for 40." << std::endl;
		}
		setAngle(angle);
	}
	else 
	{
		std::cout << "No angle found for 40 percent adjustment and continue." << std::endl;
	}
	std::this_thread::sleep_for(4s);

	std::cout << "Go through gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	setForwardAtDepth(1.5, 0.75);
	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;

	std::cout << "Spin in gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	for (int i = 0; i < 8; i++)
	{
		State temp = control_client::state();
		temp.yaw = angleAdd(temp.yaw, 90.);
		temp.z = 0.75;
		move(temp);
		std::cout << "Spin " << i+1 << " @ " << state_to_text(control_client::state()) << std::endl;
	}
	std::cout << "New State @ " << state_to_text(control_client::state()) << std::endl;

	std::cout << "Return to original angle." << std::endl;
	if (POOL_BC)
		setAngle(0.);
	else 
		setAngle(-0.);
	std::this_thread::sleep_for(4s);

	std::cout << "Continue forward through gate." << std::endl;
	std::cout << "State @ " << state_to_text(control_client::state()) << std::endl;
	setForwardAtDepth(1.5, 0.75);
	std::this_thread::sleep_for(4s);
	std::cout << "Final State @ " << state_to_text(control_client::state()) << std::endl;
}