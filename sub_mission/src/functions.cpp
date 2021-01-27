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


void vision_test()
{
	control_client::write_depth(2.);
	RCLCPP_INFO(LOGGER, "DIVED");
	downAlign(3, Task::BINS_ML, DOWN);
	RCLCPP_INFO(LOGGER, "DONE");
}

void pid_tuning_sequence()
{
	/* 
	 * Sequence of movements to tune pid gains for each degree of freedom.
	 * Make sure you have enough space in the pool or edit the movements accordingly.
	 */
	control_client::write_depth(1.);
	sleep(10.);

	while (rclcpp::ok())
	{
		// Basically moves in a square
		RCLCPP_INFO(LOGGER, "Testing X");
		move(create_state(2., 0., 0., 0., 0., 0.));
		sleep(8.);

		RCLCPP_INFO(LOGGER, "Testing Y");
		move(create_state(2., 2., 0., 0., 0., 0.));
		sleep(8.);

		RCLCPP_INFO(LOGGER, "Testing Z");
		disableAltitudeControl();
		move(create_state(2., 2., 1., 0., 0., 0.));
		sleep(8.);

		RCLCPP_INFO(LOGGER, "Testing YAW");
		move(create_state(2., 2., 1., 180., 0., 0.));
		move(create_state(2., 2., 1., 90., 0., 0.));
		move(create_state(2., 2., 1., 0., 0., 0.));
		move(create_state(2., 2., 1., -180., 0., 0.));
		move(create_state(2., 2., 1., -90., 0., 0.));
		move(create_state(2., 2., 1., 0., 0., 0.));
		sleep(8.);

		RCLCPP_INFO(LOGGER, "Testing ALTITUDE");
		control_client::write_depth(1.);
		sleep(12.);

		RCLCPP_INFO(LOGGER, "Returning to start");
		move(create_state(0., 0., 0., 0., 0., 0.));
		sleep(8.);
	}
}

void gate()
{
	RCLCPP_INFO(LOGGER, "Beginning GATE function.");

	RCLCPP_INFO(LOGGER, "Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	initial.z = 1.25;
	
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(initial).c_str());
	move(initial);
	sleep(3.);

	RCLCPP_INFO(LOGGER, "Turn towards gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	float dist = distAndAlign(5, Task::GATE_ML, FRONT);

	/* Different from porpoise */
	if (dist == 0)
	{
		RCLCPP_INFO(LOGGER, "Failed to locate gate");
		setForward(4.0);
	}
	RCLCPP_INFO(LOGGER, "Turn towards gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	dist = distAndAlign(5, Task::GATE_ML, FRONT);
	/* Different from porpoise */
	
	// RCLCPP_INFO(LOGGER, "Angle @ %f.", angle);
	// if (angle > -180. && angle < 180.) setAngle(angle);
	sleep(1.);

	RCLCPP_INFO(LOGGER, "Go through gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	setForward(dist);
	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());
}

void gate_extra()
{
	RCLCPP_INFO(LOGGER, "Beginning GATE_EXTRA function.");
	
	RCLCPP_INFO(LOGGER, "Set initial depth.");
	control_client::write_depth(2.8);
	sleep(6.);

	RCLCPP_INFO(LOGGER, "Go towards gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	setCoordinate(std::make_pair(6.5, 0));
	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());
	
	RCLCPP_INFO(LOGGER, "Set 40/60 depth.");
	control_client::write_depth(3.4);
	sleep(6.);
	
	RCLCPP_INFO(LOGGER, "Go through gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	setCoordinate(std::make_pair(13., 0));
	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());

	RCLCPP_INFO(LOGGER, "Spin in gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	for (int i = 0; i < 8; i++)
	{
		State temp = control_client::state();
		temp.yaw = angleAdd(temp.yaw, 90.);
		move(temp);
		RCLCPP_INFO(LOGGER, "Spin %d @ %s", i+1, state_to_text(control_client::state()).c_str());
	}
	RCLCPP_INFO(LOGGER, "New state @ %s.", state_to_text(control_client::state()).c_str());
	
	RCLCPP_INFO(LOGGER, "Return to original angle.");
	if (POOL_BC)
		setAngle(6.);
	if (!POOL_BC) 
		setAngle(-6.);
	sleep(4.);

	RCLCPP_INFO(LOGGER, "Continue forward through gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	setForward(3.);
	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());
}

void gate_extra_vision()
{
	RCLCPP_INFO(LOGGER, "Beginning GATE_EXTRA_VISION function.");
	
	RCLCPP_INFO(LOGGER, "Set initial depth.");
	control_client::write_depth(2.7);
	sleep(6.);

	RCLCPP_INFO(LOGGER, "Align and pass through gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	State desired = forwardAlign(15., Task::GATE_ML, FRONT, 120.);
	move(desired);
	sleep(8.);

	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());

	RCLCPP_INFO(LOGGER, "Spin 2 meters after gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	// for (int i = 0; i < 8; i++)
	// {
	// 	State temp = control_client::state();
	// 	temp.yaw = angleAdd(temp.yaw, 90.);
	// 	move(temp);
	// 	RCLCPP_INFO(LOGGER, "Spin %d @ %s", i+1, state_to_text(control_client::state()).c_str());
	// }
	continuousSpin(8);

	RCLCPP_INFO(LOGGER, "New state @ %s.", state_to_text(control_client::state()).c_str());
	
	RCLCPP_INFO(LOGGER, "Return to original angle.");
	if (POOL_BC)
		setAngle(6.);
	if (!POOL_BC) 
		setAngle (-6.);
	sleep(4.);

	RCLCPP_INFO(LOGGER, "Continue forward through gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	setForward(1.5);
	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());
}

void bins()
{
	RCLCPP_INFO(LOGGER, "Beginning BINS function.");
	control_client::write_depth(2.);

	/*
	 * Used to navigate between a few meters in front of gate and bins.
	 */

	// Increased angle because sub crashed into the wall in sim
	RCLCPP_INFO(LOGGER, "Go towards bins, avoid target.");
	if (POOL_BC) 
		setAngle(16.);
	if (!POOL_BC) 
		setAngle(-16.5);
	sleep(6.);
	setForwardAtDepth(12.5, 1.);
	sleep(3.);
	
	RCLCPP_INFO(LOGGER, "Set initial depth.");
	control_client::write_depth(3.5);

//	RCLCPP_INFO(LOGGER, "Look for bins in pool.");
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
		sleep(3.);
		RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
		bool isCentered = downContinuousAlign(10, Task::BINS_ML, DOWN, 0.2);
		if (isCentered)
		{
			RCLCPP_INFO(LOGGER, "Found initial bins set at index");
			break;
		}
	}

	RCLCPP_INFO(LOGGER, "Set second depth.");
	control_client::write_depth(1.5);
	sleep(8.);

	RCLCPP_INFO(LOGGER, "Find second offsets for bins.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	downContinuousAlign(10, Task::BINS_ML, DOWN, 0.1);

	RCLCPP_INFO(LOGGER, "Set third depth.");
	control_client::write_depth(0.75);
	sleep(6.);

	RCLCPP_INFO(LOGGER, "Find third offsets for bins.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	downContinuousAlign(10, Task::BINS_ML, DOWN, 0.1);
	
	RCLCPP_INFO(LOGGER, "Drop the balls.");
	for (int i = 0; i < 2; i++)
	{
		control_client::write("g 0 1\n");
		sleep(1.);
		control_client::write("g 1 1\n");
		sleep(3.);
		control_client::write("g 0 0\n");
		control_client::write("g 1 0\n");
		sleep(2.);
	}

	/*
	RCLCPP_INFO(LOGGER, "Reset depth.");
	// control_client::write("z -1\n");
	control_client::write_depth(-1.);
	sleep(6.);
	*/
}

void target()
{
	RCLCPP_INFO(LOGGER, "Beginning TARGET function.");

	State return_state = control_client::state();
	float goalTargetDist = 2.;

	RCLCPP_INFO(LOGGER, "Set initial state.");
	State initial = control_client::state();
	initial.yaw = 180.;
	if (POOL_BC) 
		initial.yaw = angleAdd(initial.yaw, 10.);
	if (!POOL_BC)
		initial.yaw = angleAdd(initial.yaw, -10.);
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(initial).c_str());
	move(initial);

	RCLCPP_INFO(LOGGER, "Set initial depth.");
	control_client::write_depth(1.65);
	sleep(4.);

	/*
	RCLCPP_INFO(LOGGER, "Going towards the target.");
	setForward(3.);
	sleep(1.);
	*/
	/*
	if (POOL_BC)
		setHorizontal(-1.);
	if (!POOL_BC)
		setHorizontal(1.);
	*/
	setForward(4.);
	sleep(2.);
	
	// Added wider angles because in some cases sub couldn't find target in sim
	RCLCPP_INFO(LOGGER, "Setup search angles.");
	std::vector<float> search_angles;
	search_angles.push_back(0.);
	search_angles.push_back(30.);
	search_angles.push_back(30.);
	search_angles.push_back(-120.);
	search_angles.push_back(30.);
	search_angles.push_back(30.);

	RCLCPP_INFO(LOGGER, "First turn towards target.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
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
				RCLCPP_INFO(LOGGER, "Target found.");
				RCLCPP_INFO(LOGGER, "Align with target.");
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
		sleep(10.);

		RCLCPP_INFO(LOGGER, "Backup from target.");
		RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
		float backup_dist = -6.;
		setForward(backup_dist);
	}
	else 
	{
		RCLCPP_INFO(LOGGER, "Could not find target.");
		RCLCPP_INFO(LOGGER, "Returning to original state");
		move(original);
	}

	RCLCPP_INFO(LOGGER, "Return to angle.");
	State angle_state = control_client::state();
	angle_state.yaw = 180.;
	if (POOL_BC) 
		angle_state.yaw = angleAdd(angle_state.yaw, 5.);
	if (!POOL_BC)
		angle_state.yaw = angleAdd(angle_state.yaw, -5.);
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(angle_state).c_str());
	move(angle_state);

	RCLCPP_INFO(LOGGER, "First turn towards second target.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());

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
				RCLCPP_INFO(LOGGER, "Second target found.");
				RCLCPP_INFO(LOGGER, "Align with second target.");
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
		sleep(10.);

		RCLCPP_INFO(LOGGER, "Backup from second target.");
		RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
		float backup_dist = -6.;
		setForward(backup_dist);
	}
	else 
	{
		RCLCPP_INFO(LOGGER, "Could not find second target.");
		RCLCPP_INFO(LOGGER, "Returning to original state");
		move(original);
	}

	RCLCPP_INFO(LOGGER, "Turn back towards octagon.");
	if (POOL_BC) 
		setAngle(8.);
	if (!POOL_BC) 
		setAngle(-8.);
	sleep(6.);

	move(return_state);
	sleep(3.);
}

void octagon()
{
	RCLCPP_INFO(LOGGER, "Beginning OCTAGON function.");

	RCLCPP_INFO(LOGGER, "Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(initial).c_str());
	move(initial);

	RCLCPP_INFO(LOGGER, "Going towards octagon.");
	if (!SIM)
	{
		setForward(5.);

		RCLCPP_INFO(LOGGER, "Surfacing.");
		control_client::write("p 0\n");
	}
	else
	{
		// Hard code octagon location for sim, don't use this for actual hydrophone testing
		State octagon = create_state(34, 11, 0, 0, 0, 0);
		move(octagon);
		
		RCLCPP_INFO(LOGGER, "Surfacing.");
		control_client::write("p 0\n");
	}
}

void gate_debug()
{
	RCLCPP_INFO(LOGGER, "Beginning GATE_DEBUG function.");

	/*
	 * Each pressure of 1.23 is 1m below the surface.
	 */
	/*
	RCLCPP_INFO(LOGGER, "Set initial state.");
	// State initial = control_client::state();
	State initial(0., 0., 0., 0., 0., 0.);
	initial.z = 0.75;
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(initial).c_str());
	move(initial);
	sleep(4.);
	*/
	
	RCLCPP_INFO(LOGGER, "Set initial depth.");
	control_client::write_depth(2.);
	sleep(6.);

	RCLCPP_INFO(LOGGER, "Set initial forward distance.");
	setForwardAtDepth(1.5, 0.75);
	sleep(2.);

	RCLCPP_INFO(LOGGER, "Turn towards gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	float angle = align(3, Task::GATE_ML, FRONT);
	RCLCPP_INFO(LOGGER, "Angle @ %f.", angle);
	if (isValidAngle(angle)) 
	{
		RCLCPP_INFO(LOGGER, "Found good angle for 40 percent adjustment.");
		if (GATE_LEFT)
		{
			angle = angleAdd(angle, -0.);
			RCLCPP_INFO(LOGGER, "Choosing left side of gate for 40.");
		}
		else
		{
			angle = angleAdd(angle, 0.);
			RCLCPP_INFO(LOGGER, "Choosing right side of gate for 40.");
		}
		setAngle(angle);
	}
	else 
	{
		RCLCPP_INFO(LOGGER, "No angle found for 40 percent adjustment and continue.");
	}
	sleep(4.);

	RCLCPP_INFO(LOGGER, "Go through gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	setForwardAtDepth(1.5, 0.75);
	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());

	RCLCPP_INFO(LOGGER, "Spin in gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	for (int i = 0; i < 8; i++)
	{
		State temp = control_client::state();
		temp.yaw = angleAdd(temp.yaw, 90.);
		temp.z = 0.75;
		move(temp);
		RCLCPP_INFO(LOGGER, "Spin %d @ %s", i+1, state_to_text(control_client::state()).c_str());
	}
	RCLCPP_INFO(LOGGER, "New state @ %s.", state_to_text(control_client::state()).c_str());

	RCLCPP_INFO(LOGGER, "Return to original angle.");
	if (POOL_BC)
		setAngle(0.);
	else 
		setAngle(-0.);
	sleep(4.);

	RCLCPP_INFO(LOGGER, "Continue forward through gate.");
	RCLCPP_INFO(LOGGER, "State @ %s.", state_to_text(control_client::state()).c_str());
	setForwardAtDepth(1.5, 0.75);
	sleep(4.);
	RCLCPP_INFO(LOGGER, "Final State @ %s", state_to_text(control_client::state()).c_str());
}