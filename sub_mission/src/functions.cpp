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
#include "sub_mission/observation.hpp"


void vision_test()
{
	control_client::writeDepth(2.);
	ROS_INFO("DIVED");
	downAlign(3, Task::BINS_ML, DOWN);
	ROS_INFO("DONE");
}

void pid_tuning_sequence()
{
	/* 
	 * Sequence of movements to tune pid gains for each degree of freedom.
	 * Make sure you have enough space in the pool or edit the movements accordingly.
	 */
	control_client::writeDepth(1.);
	ros::Duration(10.).sleep();

	while (ros::ok())
	{
		// Basically moves in a square
		ROS_INFO("Testing X");
		move(State(2., 0., 0., 0., 0., 0.));
		ros::Duration(8.).sleep();

		ROS_INFO("Testing Y");
		move(State(2., 2., 0., 0., 0., 0.));
		ros::Duration(8.).sleep();

		ROS_INFO("Testing Z");
		disableAltitudeControl();
		move(State(2., 2., 1., 0., 0., 0.));
		ros::Duration(8.).sleep();

		ROS_INFO("Testing YAW");
		move(State(2., 2., 1., 180., 0., 0.));
		move(State(2., 2., 1., 90., 0., 0.));
		move(State(2., 2., 1., 0., 0., 0.));
		move(State(2., 2., 1., -180., 0., 0.));
		move(State(2., 2., 1., -90., 0., 0.));
		move(State(2., 2., 1., 0., 0., 0.));
		ros::Duration(8.).sleep();

		ROS_INFO("Testing ALTITUDE");
		control_client::writeDepth(1.);
		ros::Duration(12.).sleep();

		ROS_INFO("Returning to start");
		move(State(0., 0., 0., 0., 0., 0.));
		ros::Duration(8.).sleep();
	}
}

void gate()
{
	ROS_INFO("Beginning GATE function.");

	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	initial.axis[Z] = 1.25;
	
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(3.).sleep();

	ROS_INFO("Turn towards gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float dist = distAndAlign(5, Task::GATE_ML, FRONT);

	/* Different from porpoise */
	if (dist == 0)
	{
		ROS_INFO("Failed to locate gate");
		setForward(4.0);
	}
	ROS_INFO("Turn towards gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	dist = distAndAlign(5, Task::GATE_ML, FRONT);
	/* Different from porpoise */
	
	// ROS_INFO("Angle @ %f.", angle);
	// if (angle > -180. && angle < 180.) setAngle(angle);
	ros::Duration(1.).sleep();

	ROS_INFO("Go through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForward(dist);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}

void gate_extra()
{
	ROS_INFO("Beginning GATE_EXTRA function.");
	
	ROS_INFO("Set initial depth.");
	control_client::writeDepth(2.8);
	ros::Duration(6.).sleep();

	ROS_INFO("Go towards gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setCoordinate(std::make_pair(6.5, 0));
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
	
	ROS_INFO("Set 40/60 depth.");
	control_client::writeDepth(3.4);
	ros::Duration(6.).sleep();
	
	ROS_INFO("Go through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setCoordinate(std::make_pair(13., 0));
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());

	ROS_INFO("Spin in gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	for (int i = 0; i < 8; i++)
	{
		State temp = control_client::state();
		temp.axis[YAW] = angleAdd(temp.axis[YAW], 90.);
		move(temp);
		ROS_INFO("Spin %d @ %s", i+1, control_client::state().text().c_str());
	}
	ROS_INFO("New state @ %s.", control_client::state().text().c_str());
	
	ROS_INFO("Return to original angle.");
	if (POOL_BC)
		setAngle(6.);
	if (!POOL_BC) 
		setAngle(-6.);
	ros::Duration(4.).sleep();

	ROS_INFO("Continue forward through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForward(3.);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}


/* Original Porpoise gate extra vision function, 6/12/20 */
// void gate_extra_vision()
// {
// 	ROS_INFO("Beginning GATE_EXTRA_VISION function.");
	
// 	ROS_INFO("Set initial depth.");
// 	control_client::writeDepth(2.8);
// 	ros::Duration(6.).sleep();

// 	ROS_INFO("Go towards gate.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	setCoordinate(std::make_pair(6.5, 0));
// 	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
	
// 	ROS_INFO("Set 40/60 depth.");
// 	control_client::writeDepth(3.5);
// 	ros::Duration(3.).sleep();

// 	ROS_INFO("Turn towards gate.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	float angle = align(3, Task::GATE_ML, FRONT);
// 	ROS_INFO("Angle @ %f.", angle);
// 	if (isValidAngle(angle)) 
// 	{
// 		ROS_INFO("Found good angle for 40 percent adjustment.");
// 		if (GATE_LEFT)
// 		{
// 			angle = angleAdd(angle, -0.);
// 			ROS_INFO("Choosing left side of gate for 40.");
// 		}
// 		else
// 		{
// 			angle = angleAdd(angle, 0.);
// 			ROS_INFO("Choosing right side of gate for 40.");
// 		}
// 		setAngle(angle);
// 	}
// 	else 
// 	{
// 		ROS_INFO("No angle found for 40 percent adjustment and continue.");
// 	}
// 	ros::Duration(4.).sleep();

// 	ROS_INFO("Go through gate.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	setForward(6.25);
// 	ROS_INFO("Final State @ %s", control_client::state().text().c_str());

// 	ROS_INFO("Spin in gate.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	for (int i = 0; i < 8; i++)
// 	{
// 		State temp = control_client::state();
// 		temp.axis[YAW] = angleAdd(temp.axis[YAW], 90.);
// 		move(temp);
// 		ROS_INFO("Spin %d @ %s", i+1, control_client::state().text().c_str());
// 	}
// 	ROS_INFO("New state @ %s.", control_client::state().text().c_str());
	
// 	ROS_INFO("Return to original angle.");
// 	if (POOL_BC)
// 		setAngle(6.);
// 	if (!POOL_BC) 
// 		setAngle(-6.);
// 	ros::Duration(4.).sleep();

// 	ROS_INFO("Continue forward through gate.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	setForward(3.);
// 	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
// }

void gate_extra_vision()
{
	ROS_INFO("Beginning GATE_EXTRA_VISION function.");
	
	ROS_INFO("Set initial depth.");
	// control_client::writeDepth(2.8);
	control_client::writeDepth(2.7);
	ros::Duration(6.).sleep();

	// // Reduce initial forward distance compared to original porpoise, original went a little too far
	// ROS_INFO("Go towards gate.");
	// ROS_INFO("State @ %s.", control_client::state().text().c_str());
	// setCoordinate(std::make_pair(5., 0));
	// ROS_INFO("Final State @ %s", control_client::state().text().c_str());

	// // Made sub go deeper, original depth seemed too high	
	// ROS_INFO("Set 40/60 depth.");
	// control_client::writeDepth(2.7);
	// ros::Duration(3.).sleep();

	ROS_INFO("Align and pass through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	State desired = forwardAlign(15., Task::GATE_ML, FRONT, 120.);
	move(desired);
	ros::Duration(8.).sleep();

	// for (int i = 0; i < 3; i++)
	// {
	// 	if (i != 0) setForward(1.);
	// 	bool found = frontConsecutiveAngleAlign(2, Task::GATE_ML, FRONT);
	// 	if (found)
	// 	{
	// 		ROS_INFO("Gate found.");
	// 		break;
	// 	}
	// }

	// ros::Duration(4.).sleep();

	// ROS_INFO("State @ %s.", control_client::state().text().c_str());

	// // Find distance to gate
	// float dist = distance(4, Task::GATE_ML, FRONT);
	// ROS_INFO("Gate distance of %f.", dist);
	// if (!isValidDistance(dist))
	// {
	// 	ROS_INFO("Moving forward to find gate.");
	// 	setForward(1.5);
	// 	dist = distance(4, Task::GATE_ML, FRONT);
	// }
	// if (!isValidDistance(dist))
	// {
	// 	dist = 5;
	// 	ROS_INFO("Adjust distance to %f", dist);
	// }

	// ROS_INFO("Go through gate.");
	// setForward(dist + 2.);
	// ros::Duration(8.).sleep();
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());

	ROS_INFO("Spin 2 meters after gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	// for (int i = 0; i < 8; i++)
	// {
	// 	State temp = control_client::state();
	// 	temp.axis[YAW] = angleAdd(temp.axis[YAW], 90.);
	// 	move(temp);
	// 	ROS_INFO("Spin %d @ %s", i+1, control_client::state().text().c_str());
	// }
	continuousSpin(8);

	ROS_INFO("New state @ %s.", control_client::state().text().c_str());
	
	ROS_INFO("Return to original angle.");
	if (POOL_BC)
		setAngle(6.);
	if (!POOL_BC) 
		setAngle (-6.);
	ros::Duration(4.).sleep();

	ROS_INFO("Continue forward through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForward(1.5);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}

// This is the original bins code from Porpoise 6/9/20
// void bins()
// {
// 	ROS_INFO("Beginning BINS function.");
// 	control_client::writeDepth(2);
// 	/*
// 	 * Used to navigate between a few meters in front of gate and bins.
// 	 */
// 	ROS_INFO("Go towards bins, avoid target.");
// //	if (POOL_BC) 
// //		setAngle(11.);
// //	if (!POOL_BC) 
// //		setAngle(-11.5);
// //	ros::Duration(6.).sleep();
// //	setForwardAtDepth(12.5, 1.);
// //	ros::Duration(3.).sleep();
	
// //	ROS_INFO("Set initial depth.");
// //	control_client::writeDepth(3.5);

// //	ROS_INFO("Look for bins in pool.");
// 	std::vector<Coordinate> search_offsets;
// 	// Use for pool B/C.
// 	search_offsets.push_back(std::make_pair(0., 0.));
// 	search_offsets.push_back(std::make_pair(-0.5, -0.5));
// 	search_offsets.push_back(std::make_pair(0.5, 0.5));
// 	search_offsets.push_back(std::make_pair(0., 0.5));
// 	search_offsets.push_back(std::make_pair(-0.5, 0.));
// 	search_offsets.push_back(std::make_pair(0.5, -0.5));
// 	// Use for pool A.
// 	/*
// 	search_offsets.push_back(std::make_pair(0., 0.));
// 	search_offsets.push_back(std::make_pair(-2., 2.));
// 	search_offsets.push_back(std::make_pair(0., -4.));
// 	search_offsets.push_back(std::make_pair(4., 0.));
// 	search_offsets.push_back(std::make_pair(0., 4.));
// 	search_offsets.push_back(std::make_pair(-2., -2.));
// 	*/
// 	// Use for pool D.
// 	/*
// 	search_offsets.push_back(std::make_pair(0., 0.));
// 	search_offsets.push_back(std::make_pair(5., -4.));
// 	search_offsets.push_back(std::make_pair(-4., 0.));
// 	search_offsets.push_back(std::make_pair(0., 4.));
// 	search_offsets.push_back(std::make_pair(4., 0.));
// 	search_offsets.push_back(std::make_pair(-2., -2.));
// 	*/
// 	for (int i = 0; i < search_offsets.size(); i++)
// 	{
// 		if (i != 0)
// 			addCoordinate(search_offsets[i]);
// 		ros::Duration(3.).sleep();
// 		ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 		Coordinate coordinate1 = downAlign(4, Task::BINS, DOWN);
// 		ROS_INFO("searching %d",i);
// 		ROS_INFO("Bins position @ %f, %f.", coordinate1.first, coordinate1.second);
// 		if (isValidCoordinate(coordinate1))
// 		{
// 			ROS_INFO("Found initial bins set at index");
// 			setCoordinate(coordinate1);
// 			ros::Duration(4.).sleep();
// 			break;
// 		}
// 	}

// 	/*
// 	 * Non-searching bins code.
// 	 */
	
// 	ROS_INFO("Find offsets for bins.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	Coordinate coordinate1 = downAlign(5, Task::BINS_ML, DOWN);
// 	ROS_INFO("Bins position @ %f, %f.", coordinate1.first, coordinate1.second);
// 	if (isValidCoordinate(coordinate1))
// 	{
// 		setCoordinate(coordinate1);
// 		ros::Duration(6.).sleep();
// 	}
	

// 	ROS_INFO("Set second depth.");
// 	control_client::writeDepth(1.5);
// 	ros::Duration(6.).sleep();

// 	ROS_INFO("Find second offsets for bins.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	Coordinate coordinate2 = downAlign(4, Task::BINS, DOWN);
// 	ROS_INFO("Bins position @ %f, %f.", coordinate2.first, coordinate2.second);
// 	if (isValidCoordinate(coordinate2))
// 	{
// 		setCoordinate(coordinate2);
// 		ros::Duration(6.).sleep();
// 	}

// 	ROS_INFO("Set third depth.");
// 	control_client::writeDepth(0.75);
// 	ros::Duration(6.).sleep();

// 	ROS_INFO("Find third offsets for bins.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	Coordinate coordinate3 = downAlign(4, Task::BINS, DOWN);
// 	ROS_INFO("Bins position @ %f, %f.", coordinate3.first, coordinate3.second);
// 	if (isValidCoordinate(coordinate3))
// 	{
// 		setCoordinate(coordinate3);
// 		ros::Duration(6.).sleep();
// 	}

// 	/*ROS_INFO("Set fourth depth.");
// 	control_client::writeDepth(1.);
// 	ros::Duration(6.).sleep();
// 	ROS_INFO("Find fourth offsets for bins.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	Coordinate coordinate4 = downAlign(4, Task::BINS, DOWN);
// 	ROS_INFO("Bins position @ %f, %f.", coordinate4.first, coordinate4.second);
// 	if (isValidCoordinate(coordinate4))
// 	{
// 		setCoordinate(coordinate4);
// 		ros::Duration(6.).sleep();
// 	}
// 	ROS_INFO("Set dropping depth.");
// 	float dropping_depth = 0.75;
// 	control_client::writeDepth(dropping_depth);
// 	ros::Duration(9.).sleep();
// */
// 	ROS_INFO("Drop the balls.");
// 	for (int i = 0; i < 2; i++)
// 	{
// 		control_client::write("g 0 1\n");
// 		ros::Duration(1.).sleep();
// 		control_client::write("g 1 1\n");
// 		ros::Duration(3.).sleep();
// 		control_client::write("g 0 0\n");
// 		control_client::write("g 1 0\n");
// 		ros::Duration(2.).sleep();
// 	}

// 	/*
// 	ROS_INFO("Reset depth.");
// 	// control_client::write("z -1\n");
// 	control_client::writeDepth(-1.);
// 	ros::Duration(6.).sleep();
// 	*/
// }


// Testing continuous down alignment so object is in the center of the camera
// Before moving down
void bins()
{
	ROS_INFO("Beginning BINS function.");
	// control_client::writeDepth(2.);

	/*
	 * Used to navigate between a few meters in front of gate and bins.
	 */
	ROS_INFO("Go towards bins, avoid target.");
	// Increased angle because sub crashed into the wall in sim
	if (POOL_BC) 
		setAngle(16.);
	if (!POOL_BC) 
		setAngle(-16.5);
	ros::Duration(6.).sleep();
	setForwardAtDepth(12.5, 1.);
	ros::Duration(3.).sleep();
	
	ROS_INFO("Set initial depth.");
	control_client::writeDepth(3.5);

//	ROS_INFO("Look for bins in pool.");
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
		ros::Duration(3.).sleep();
		ROS_INFO("State @ %s.", control_client::state().text().c_str());
		// Coordinate coordinate1 = downAlign(4, Task::BINS_ML, DOWN);
		// ROS_INFO("searching %d",i);
		bool isCentered = downContinuousAlign(10, Task::BINS_ML, DOWN, 0.2);
		if (isCentered)
		{
			ROS_INFO("Found initial bins set at index");
			break;
		}
		// ROS_INFO("Bins position @ %f, %f.", coordinate1.first, coordinate1.second);
		// if (isValidCoordinate(coordinate1))
		// {
		// 	ROS_INFO("Found initial bins set at index");
		// 	setCoordinate(coordinate1);
		// 	ros::Duration(6.).sleep();
		// 	break;
		// }
	}

	/*
	 * Non-searching bins code.
	 */
	/*
	ROS_INFO("Find offsets for bins.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	Coordinate coordinate1 = downAlign(5, Task::BINS_ML, DOWN);
	ROS_INFO("Bins position @ %f, %f.", coordinate1.first, coordinate1.second);
	if (isValidCoordinate(coordinate1))
	{
		setCoordinate(coordinate1);
		ros::Duration(6.).sleep();
	}
	*/

	ROS_INFO("Set second depth.");
	control_client::writeDepth(1.5);
	ros::Duration(8.).sleep();

	ROS_INFO("Find second offsets for bins.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	downContinuousAlign(10, Task::BINS_ML, DOWN, 0.1);

	ROS_INFO("Set third depth.");
	control_client::writeDepth(0.75);
	ros::Duration(6.).sleep();

	ROS_INFO("Find third offsets for bins.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	downContinuousAlign(10, Task::BINS_ML, DOWN, 0.1);

	/*ROS_INFO("Set fourth depth.");
	control_client::writeDepth(1.);
	ros::Duration(6.).sleep();

	ROS_INFO("Find fourth offsets for bins.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	Coordinate coordinate4 = downAlign(4, Task::BINS, DOWN);
	ROS_INFO("Bins position @ %f, %f.", coordinate4.first, coordinate4.second);
	if (isValidCoordinate(coordinate4))
	{
		setCoordinate(coordinate4);
		ros::Duration(6.).sleep();
	}

	ROS_INFO("Set dropping depth.");
	float dropping_depth = 0.75;
	control_client::writeDepth(dropping_depth);
	ros::Duration(9.).sleep();
*/
	
	ROS_INFO("Drop the balls.");
	for (int i = 0; i < 2; i++)
	{
		control_client::write("g 0 1\n");
		ros::Duration(1.).sleep();
		control_client::write("g 1 1\n");
		ros::Duration(3.).sleep();
		control_client::write("g 0 0\n");
		control_client::write("g 1 0\n");
		ros::Duration(2.).sleep();
	}

	/*
	ROS_INFO("Reset depth.");
	// control_client::write("z -1\n");
	control_client::writeDepth(-1.);
	ros::Duration(6.).sleep();
	*/
}

/* Original Porpoise target function */
// void target()
// {
// 	ROS_INFO("Beginning TARGET function.");

// 	State return_state = control_client::state();

// 	ROS_INFO("Set initial state.");
// 	State initial = control_client::state();
// 	initial.axis[YAW] = 180.;
// 	if (POOL_BC) 
// 		initial.axis[YAW] = angleAdd(initial.axis[YAW], 5.);
// 	if (!POOL_BC)
// 		initial.axis[YAW] = angleAdd(initial.axis[YAW], -5.);
// 	ROS_INFO("State @ %s.", initial.text().c_str());
// 	move(initial);

// 	ROS_INFO("Set initial depth.");
// 	control_client::writeDepth(1.65);
// 	ros::Duration(4.).sleep();

// 	/*
// 	ROS_INFO("Going towards the target.");
// 	setForward(3.);
// 	ros::Duration(1.).sleep();
// 	*/
// 	/*
// 	if (POOL_BC)
// 		setHorizontal(-1.);
// 	if (!POOL_BC)
// 		setHorizontal(1.);
// 	*/
// 	setForward(4.);
// 	ros::Duration(2.).sleep();
	
// 	ROS_INFO("Setup search angles.");
// 	std::vector<float> search_angles;
// 	search_angles.push_back(0.);
// 	search_angles.push_back(30.);
// 	search_angles.push_back(-60.);
// 	search_angles.push_back(30.);

// 	ROS_INFO("First turn towards target.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	for (int i = 0; i < search_angles.size(); i++)
// 	{
// 		if (i != 0)
// 			addAngle(search_angles[i]);
// 		float angle = align(4, Task::TARGET_ML, FRONT);
// 		ROS_INFO("Angle @ %f.", angle);
// 		if (isValidAngle(angle)) 
// 		{
// 			ROS_INFO("Angle is good.");
// 			setAngle(angle);
// 			ros::Duration(4.).sleep();

// 			ROS_INFO("First move towards target.");
// 			setForward(4.);
// 			ros::Duration(2.).sleep();

// 			ROS_INFO("Second turn towards target.");
// 			ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 			angle = align(4, Task::TARGET_ML, FRONT);
// 			ROS_INFO("Angle @ %f.", angle);
// 			if (isValidAngle(angle)) setAngle(angle);
// 			ros::Duration(4.).sleep();

// 			ROS_INFO("Calculate ram distance.");
// 			ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 			// float ram_dist = distance(5, Task::TARGET, FRONT);
// 			float ram_dist = 4.;
// 			ROS_INFO("Ram distance of %f.", ram_dist);
// 			/*
// 			if (ram_dist < 0. || ram_dist > 8.) 
// 				ram_dist = 5.;
// 			*/

// 			/*
// 			 * Don't use standard move method in case we attempt to move past the
// 			 * target.
// 			 */
// 			ROS_INFO("Ram target with 10 seconds.");
// 			State ram_state = control_client::state();
// 			float ram_input[3] = {ram_dist, 0., 0.};
// 			float ram_angles[3] = {ram_state.axis[YAW], 0., 0.};
// 			float ram_output[3];
// 			bodyToInertial(ram_input, ram_angles, ram_output);
// 			ram_state.axis[X] += ram_output[0];
// 			ram_state.axis[Y] += ram_output[1];
// 			control_client::writeState(ram_state);
// 			ros::Duration(10.).sleep();

// 			ROS_INFO("Backup from target.");
// 			ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 			float backup_dist = -6.;
// 			setForward(backup_dist);

// 			/*
// 			ROS_INFO("Return to angle.");
// 			State angle_state = control_client::state();
// 			angle_state.axis[YAW] = 180.;
// 			if (POOL_BC) 
// 				angle_state.axis[YAW] = angleAdd(angle_state.axis[YAW], 5.);
// 			if (!POOL_BC)
// 				angle_state.axis[YAW] = angleAdd(angle_state.axis[YAW], -5.);
// 			ROS_INFO("State @ %s.", angle_state.text().c_str());
// 			move(angle_state);
// 			*/

// 			break;
// 		}
// 	}
	
// 	/*
// 	ROS_INFO("First turn towards second target.");
// 	ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 	search_angles.clear();
// 	search_angles.push_back(0.);
// 	for (int i = 0; i < search_angles.size(); i++)
// 	{
// 		if (i != 0)
// 			addAngle(search_angles[i]);
// 		float angle = align(4, Task::SECOND_TARGET_ML, FRONT);
// 		ROS_INFO("Angle @ %f.", angle);
// 		if (isValidAngle(angle)) 
// 		{
// 			ROS_INFO("Second angle is good.");
// 			setAngle(angle);
// 			ros::Duration(4.).sleep();
			
// 			ROS_INFO("First move towards second target.");
// 			setForward(4.);
// 			ros::Duration(2.).sleep();
// 			ROS_INFO("Second turn towards second target.");
// 			ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 			angle = align(4, Task::SECOND_TARGET_ML, FRONT);
// 			ROS_INFO("Angle @ %f.", angle);
// 			if (isValidAngle(angle)) setAngle(angle);
// 			ros::Duration(4.).sleep();
// 			ROS_INFO("Calculate ram distance to second target.");
// 			ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 			// float ram_dist = distance(5, Task::TARGET, FRONT);
// 			float ram_dist = 4.;
// 			ROS_INFO("Ram distance of %f.", ram_dist);
// 			if (ram_dist < 0. || ram_dist > 8.) 
// 				ram_dist = 5.;
// 			ROS_INFO("Ram target with 10 seconds.");
// 			State ram_state = control_client::state();
// 			float ram_input[3] = {ram_dist, 0., 0.};
// 			float ram_angles[3] = {ram_state.axis[YAW], 0., 0.};
// 			float ram_output[3];
// 			bodyToInertial(ram_input, ram_angles, ram_output);
// 			ram_state.axis[X] += ram_output[0];
// 			ram_state.axis[Y] += ram_output[1];
// 			control_client::writeState(ram_state);
// 			ros::Duration(10.).sleep();
// 			ROS_INFO("Backup from target.");
// 			ROS_INFO("State @ %s.", control_client::state().text().c_str());
// 			float backup_dist = -8.;
// 			setForward(backup_dist);
// 			break;
// 		}
// 	}
// 	*/

// 	ROS_INFO("Turn back towards octagon.");
// 	if (POOL_BC) 
// 		setAngle(8.);
// 	if (!POOL_BC) 
// 		setAngle(-8.);
// 	ros::Duration(6.).sleep();

// 	move(return_state);
// 	ros::Duration(3.).sleep();
// }

// Simulator target, using continuous align
void target()
{
	ROS_INFO("Beginning TARGET function.");

	State return_state = control_client::state();
	float goalTargetDist = 2.;

	ROS_INFO("Set initial state.");
	State initial = control_client::state();
	initial.axis[YAW] = 180.;
	if (POOL_BC) 
		initial.axis[YAW] = angleAdd(initial.axis[YAW], 10.);
	if (!POOL_BC)
		initial.axis[YAW] = angleAdd(initial.axis[YAW], -10.);
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);

	ROS_INFO("Set initial depth.");
	control_client::writeDepth(1.65);
	ros::Duration(4.).sleep();

	/*
	ROS_INFO("Going towards the target.");
	setForward(3.);
	ros::Duration(1.).sleep();
	*/
	/*
	if (POOL_BC)
		setHorizontal(-1.);
	if (!POOL_BC)
		setHorizontal(1.);
	*/
	setForward(4.);
	ros::Duration(2.).sleep();
	
	// Added wider angles because in some cases sub couldn't find target in sim
	ROS_INFO("Setup search angles.");
	std::vector<float> search_angles;
	search_angles.push_back(0.);
	search_angles.push_back(30.);
	search_angles.push_back(30.);
	search_angles.push_back(-120.);
	search_angles.push_back(30.);
	search_angles.push_back(30.);

	ROS_INFO("First turn towards target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
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
				ROS_INFO("Target found.");
				ROS_INFO("Align with target.");
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
		// ROS_INFO("Calculate ram distance.");
		// float ram_dist = distance(4, Task::TARGET_ML, FRONT);
		// ROS_INFO("Ram distance @ %f", ram_dist);
		// if (!isValidDistance(ram_dist))
		// {
		// 	ROS_INFO("Moving forward to find target.");
		// 	setForward(1.5);
		// 	ram_dist = distance(4, Task::TARGET_ML, FRONT);
		// 	ROS_INFO("Ram distance @ %f", ram_dist);

		// }
		// if (ram_dist > goalTargetDist)
		// {
		// 	ROS_INFO("Moving closer to target.");
		// 	float adjust_dist = std::fabs(ram_dist - goalTargetDist);
		// 	setForward(adjust_dist);
		// 	ram_dist = distance(4, Task::TARGET_ML, FRONT);
		// 	ROS_INFO("Ram distance @ %f", ram_dist);
		// }
		// if (!isValidDistance(ram_dist))
		// {
		// 	ram_dist = 4.;
		// 	ROS_INFO("Couldn't calculate ram distance.");
		// 	ROS_INFO("Setting ram distance to %f", ram_dist);
		// }

		// ram_dist += 1;
		// ROS_INFO("Adjust ram distance to be safe.");
		// ROS_INFO("Ram distance @ %f", ram_dist);

		// ROS_INFO("Align with target.");
		// float angle = align(4, Task::TARGET_ML, FRONT);
		// if (isValidAngle(angle))
		// 	setAngle(angle);
		// /*
		//  * Don't use standard move method in case we attempt to move past the
		//  * target.
		//  */
		// ROS_INFO("Ram target with 10 seconds.");
		// ROS_INFO("Ram distance @ %f", ram_dist);
		// State ram_state = control_client::state();
		// float ram_input[3] = {ram_dist, 0., 0.};
		// float ram_angles[3] = {ram_state.axis[YAW], 0., 0.};
		// float ram_output[3];
		// bodyToInertial(ram_input, ram_angles, ram_output);
		// ram_state.axis[X] += ram_output[0];
		// ram_state.axis[Y] += ram_output[1];
		// control_client::writeState(ram_state);
		ros::Duration(10.).sleep();

		ROS_INFO("Backup from target.");
		ROS_INFO("State @ %s.", control_client::state().text().c_str());
		float backup_dist = -6.;
		setForward(backup_dist);
	}
	else 
	{
		ROS_INFO("Could not find target.");
		ROS_INFO("Returning to original state");
		move(original);
	}

	ROS_INFO("Return to angle.");
	State angle_state = control_client::state();
	angle_state.axis[YAW] = 180.;
	if (POOL_BC) 
		angle_state.axis[YAW] = angleAdd(angle_state.axis[YAW], 5.);
	if (!POOL_BC)
		angle_state.axis[YAW] = angleAdd(angle_state.axis[YAW], -5.);
	ROS_INFO("State @ %s.", angle_state.text().c_str());
	move(angle_state);

	ROS_INFO("First turn towards second target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());

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
				ROS_INFO("Second target found.");
				ROS_INFO("Align with second target.");
				setAngle(angle);
				inSight = true;
				break;
			}
		}
		if (inSight) break;

		// Move horizontally in case second target is tilted
		// Commented about because targets in comp aren't tilted yet.
		// setHorizontal(hdist);
		// hdist *= -1.;

		// for (int i = 0; i < search_angles.size(); i++)
		// {
		// 	if (i != 0) addAngle(search_angles[i]);
		// 	float angle = align(4, Task::SECOND_TARGET_ML, FRONT);
		// 	if (isValidAngle(angle))
		// 	{
		// 		ROS_INFO("Second target found.");
		// 		ROS_INFO("Align with second target.");
		// 		setAngle(angle);
		// 		inSight = true;
		// 		break;
		// 	}
		// }
		// if (inSight) break;
	}

	if (inSight)
	{
		forwardAlign(12., Task::SECOND_TARGET_ML, FRONT, 80.);
		// ROS_INFO("Calculate ram distance.");
		// float ram_dist = distance(4, Task::SECOND_TARGET_ML, FRONT);
		// ROS_INFO("Ram distance @ %f", ram_dist);
		// if (!isValidDistance(ram_dist))
		// {
		// 	ROS_INFO("Moving forward to find second target.");
		// 	setForward(1.5);
		// 	ram_dist = distance(4, Task::SECOND_TARGET_ML, FRONT);
		// 	ROS_INFO("Ram distance @ %f", ram_dist);
		// }
		// if (ram_dist > goalTargetDist)
		// {
		// 	ROS_INFO("Moving closer to second target.");
		// 	float adjust_dist = std::fabs(ram_dist - goalTargetDist);
		// 	setForward(adjust_dist);
		// 	ram_dist = distance(4, Task::SECOND_TARGET_ML, FRONT);
		// 	ROS_INFO("Ram distance @ %f", ram_dist);
		// }
		// if (!isValidDistance(ram_dist))
		// {
		// 	ram_dist = 4.;
		// 	ROS_INFO("Couldn't calculate ram distance.");
		// 	ROS_INFO("Setting ram distance to %f", ram_dist);
		// }

		// ram_dist += 1;
		// ROS_INFO("Adjust ram distance to be safe.");
		// ROS_INFO("Ram distance @ %f", ram_dist);

		// ROS_INFO("Align with second target.");
		// float angle = align(4, Task::SECOND_TARGET_ML, FRONT);
		// if (isValidAngle(angle))
		// 	setAngle(angle);		/*
		//  * Don't use standard move method in case we attempt to move past the
		//  * target.
		//  */
		// ROS_INFO("Ram second target with 10 seconds.");
		// ROS_INFO("Ram distance @ %f", ram_dist);
		// State ram_state = control_client::state();
		// float ram_input[3] = {ram_dist, 0., 0.};
		// float ram_angles[3] = {ram_state.axis[YAW], 0., 0.};
		// float ram_output[3];
		// bodyToInertial(ram_input, ram_angles, ram_output);
		// ram_state.axis[X] += ram_output[0];
		// ram_state.axis[Y] += ram_output[1];
		// control_client::writeState(ram_state);
		ros::Duration(10.).sleep();

		ROS_INFO("Backup from second target.");
		ROS_INFO("State @ %s.", control_client::state().text().c_str());
		float backup_dist = -6.;
		setForward(backup_dist);
	}
	else 
	{
		ROS_INFO("Could not find second target.");
		ROS_INFO("Returning to original state");
		move(original);
	}

	ROS_INFO("Turn back towards octagon.");
	if (POOL_BC) 
		setAngle(8.);
	if (!POOL_BC) 
		setAngle(-8.);
	ros::Duration(6.).sleep();

	move(return_state);
	ros::Duration(3.).sleep();
}

void octagon()
{
	ROS_INFO("Beginning OCTAGON function.");

	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);

	ROS_INFO("Going towards octagon.");
	if (!SIM)
	{
		setForward(5.);

		ROS_INFO("Surfacing.");
		control_client::write("p 0\n");
	}
	else
	{
		// Hard code octagon location for sim, don't use this for actual hydrophone testing
		State octagon(34, 11, 0, 0, 0, 0);
		move(octagon);
		
		ROS_INFO("Surfacing.");
		control_client::write("p 0\n");
	}
}

void gate_debug()
{
	ROS_INFO("Beginning GATE_DEBUG function.");

	/*
	 * Each pressure of 1.23 is 1m below the surface.
	 */
	/*
	ROS_INFO("Set initial state.");
	// State initial = control_client::state();
	State initial(0., 0., 0., 0., 0., 0.);
	initial.axis[Z] = 0.75;
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(4.).sleep();
	*/
	
	ROS_INFO("Set initial depth.");
	control_client::writeDepth(2.);
	ros::Duration(6.).sleep();

	ROS_INFO("Set initial forward distance.");
	setForwardAtDepth(1.5, 0.75);
	ros::Duration(2.).sleep();

	ROS_INFO("Turn towards gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float angle = align(3, Task::GATE_ML, FRONT);
	ROS_INFO("Angle @ %f.", angle);
	if (isValidAngle(angle)) 
	{
		ROS_INFO("Found good angle for 40 percent adjustment.");
		if (GATE_LEFT)
		{
			angle = angleAdd(angle, -0.);
			ROS_INFO("Choosing left side of gate for 40.");
		}
		else
		{
			angle = angleAdd(angle, 0.);
			ROS_INFO("Choosing right side of gate for 40.");
		}
		setAngle(angle);
	}
	else 
	{
		ROS_INFO("No angle found for 40 percent adjustment and continue.");
	}
	ros::Duration(4.).sleep();

	ROS_INFO("Go through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForwardAtDepth(1.5, 0.75);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());

	ROS_INFO("Spin in gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	for (int i = 0; i < 8; i++)
	{
		State temp = control_client::state();
		temp.axis[YAW] = angleAdd(temp.axis[YAW], 90.);
		temp.axis[Z] = 0.75;
		move(temp);
		ROS_INFO("Spin %d @ %s", i+1, control_client::state().text().c_str());
	}
	ROS_INFO("New state @ %s.", control_client::state().text().c_str());

	ROS_INFO("Return to original angle.");
	if (POOL_BC)
		setAngle(0.);
	else 
		setAngle(-0.);
	ros::Duration(4.).sleep();

	ROS_INFO("Continue forward through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForwardAtDepth(1.5, 0.75);
	ros::Duration(4.).sleep();
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}