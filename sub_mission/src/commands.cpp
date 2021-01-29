/** @file commands.cpp
 *  @brief Functions for generic competition actions.
 *
 *  @author David Zhang
 *  @author Suhas Nagar
 *  @author Craig Wang
 */
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <string.h>

#include "sub_mission/commands.hpp"
#include "sub_mission/functions.hpp"
#include "sub_mission/client.hpp"
#include "sub_mission/euler.hpp"
#include "sub_mission/config.hpp"
#include "sub_mission/util.hpp"


float angleDifference(float a1, float a2)
{
	// For [0, 360].
	/*
	   float c1 = a1 - a2;
	   float c2 = a1 - a2 + 360.;
	   float c3 = a1 - a2 - 360.;
	   if (abs(c1) < abs(c2) && abs(c1) < abs(c3))
	   return c1;
	   if (abs(c2) < abs(c3))
	   return c2;
	   else 
	   return c3;
	*/

	// For [-180, 180].
	float b1 = a1-a2;
	if (std::fabs(b1) > 180.)
	{
		if (a1 < a2)
			a1 += 360.;
		else 
			a2 += 360.;
		b1 = a1-a2;
	}
	return b1;
}

float angleAdd(float a1, float add)
{
	float temp = a1 + add;
	if (temp > 180.)
		return temp - 360.;
	else if (temp < -180.)
		return temp + 360.;

	return temp;
}

float align(int attempts, Task task, int camera)
{
	int threshold = attempts/2;
	int original = attempts;
	float average = 0.;
	for (int i = 0; i < original; i++)
	{
		Observation obs = vision_client::vision(task, camera);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		
		// Probabilities should be checked before.
		if (obs.prob > 0.30) 
			average += obs.hangle; 
		else 
			attempts -= 1;
		// std::this_thread::sleep_for(0.25s);
	}

	if (attempts == 0)
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for alignment.");
		return -999.;
	}
	// Don't add current yaw inside of loop because when sub is turned
	// 180 degrees around, it oscillates between positive and negative
	// and the average turns out to be around 0 degrees, going the opposite direction
	State now = control_client::state();
	average /= attempts;
	average = angleAdd(now.yaw, average);
	return average;
}

float relativeAlign(int attempts, Task task, int camera)
{
	/* Output relative angle to object instead of absolute */
	int threshold = attempts/2;
	int original = attempts;
	float average = 0.;
	for (int i = 0; i < original; i++)
	{
		State current_state = control_client::state();
		Observation obs = vision_client::vision(task, camera);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		
		// Probabilities should be checked before.
		if (obs.prob > 0.30)
		{
			average += obs.hangle; 
		}
		else 
		{
			attempts -= 1;
		}
		// std::this_thread::sleep_for(0.25s);
	}

	if (attempts == 0)
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for alignment.");
		return -999.;
	}
	average /= attempts;
	return average;
}

float distance(int attempts, Task task, int camera)
{
	int threshold = attempts/2;
	int original = attempts;
	float average = 0.;
	for (int i = 0; i < original; i++)
	{
		State current_state = control_client::state();
		Observation obs = vision_client::vision(task, camera);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		
		if (obs.prob > 0.30)
		{
			average += obs.dist;
		}
		else 
		{
			attempts -= 1;
		}
		// std::this_thread::sleep_for(0.25s);
	}

	if (attempts <= 1)
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for alignment.");
		return -999.;
	}
	average /= attempts;
	return average;
}

float distAndAlign(int attempts, Task task, int camera)
{
	/* 
	 * Not used for now because slight inaccuracies in horizontal movement
	 * can significantly disrupt distance calculation.
	 * Recommended to use distance() instead.
	 */
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Calculating distance.");
	State original = control_client::state();
	float ang1 = relativeAlign(attempts, task, camera);
	if (!isValidAngle(ang1))
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Could not find object to align to.");
		return 0;
	}
	float hdist = 0.5;
	ang1 *= M_PI / 180.;

	//Aim to move towards the object to keep it in view
	if (ang1 < 0.)
	{
		hdist *= -1;
	}
	setHorizontal(hdist);
	float ang2 = relativeAlign(attempts, task, camera);
	if (!isValidAngle(ang2))
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Could not find object to align to.");
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Returning to original state.");
		move(original);
		return 0;
	}
	ang2 *= M_PI/180;
	//angle1
	float tri_1 = M_PI/2 - fabs(ang1);
	//angle2
	float tri_2 = M_PI/2 + fabs(ang2);

	//If the sub crossed over the object in its movement
	if (ang1 * ang2 < 0.)
	{
		tri_2 = M_PI / 2 - fabs(ang2);	
	}

	//angle3	
	float tri_3 = M_PI - tri_1 - tri_2;
	//Use the law of sines to calculate the angle
	float l = fabs(sin(tri_1) * hdist / (sin(tri_3)));
	//Calculate the horizontal translation to align with the object
	float halign = fabs(cos(tri_2) * l);
	//Move left or right depending on where the object is
	if (ang2 < 0.)
	{
		halign *= -1;
	}
	setHorizontal(halign);
	//Find the distance forward to reach the object
	float dist = sin(tri_2) * l;
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Distance to object of %f", dist);
	return dist;
}

bool isValidDistance(float dist)
{
	return (dist > 0. && dist < 17.5);
}

State forwardAlign(float max_dist, Task task, int camera, float timeout)
{
	/* 
	 * Continuously move forward and adjust yaw to align with object.
	 * Then, set the sub to move two meters through that object.
	 * The timeout is added as a fail safe to make sure the sub doesn't get stuck
	 * trying to move past the buoy if ML fails and it can't detect the buoy.
	 * Can be used for gate and buoys.
	 */
	auto start_time = std::chrono::high_resolution_clock::now();;
	State initial = control_client::state();
	float initial_x = initial.x;
	float initial_y = initial.y;

	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting forward align");
	while (rclcpp::ok())
	{
		State state = control_client::state();

		// Break if travelled too far or time is up
		float travelled = sqrt(
			std::pow(state.x - initial_x, 2) +
			std::pow(state.y - initial_y, 2)
		);
		float elapsed_seconds = seconds_since_start(start_time);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "%f seconds since forward align call.", elapsed_seconds);
		if (travelled > max_dist || elapsed_seconds > timeout)
			break;

		// Detect object
		Observation obs = vision_client::vision(task, camera);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());

		// Break if object is within 3 meters, aim to move 2 meters through object
		if (isValidDistance(obs.dist) && obs.dist < 3.)
		{
			// Don't use setForward because sub can't move through buoy, sub will get stuck
			State end = control_client::state();
			end.yaw = state.yaw;
			float input[3] = {obs.dist + 2., 0., 0.};
			float angles[3] = {end.yaw, 0., 0.};
			float output[3];
			bodyToInertial(input, angles, output);
			end.x += output[X];
			end.y += output[Y];
			control_client::write_state(end);
			return end;
		}

		// Calculate angle to turn towards the object
		state = control_client::state();
		if (isValidAngle(obs.hangle) && obs.prob > 0.5)
			state.yaw = angleAdd(state.yaw, obs.hangle);

		// Move forward
		float input[3] = {1.5, 0., 0.};
		float angles[3] = {state.yaw, 0., 0.};
		float output[3];
		bodyToInertial(input, angles, output);
		state.x += output[X];
		state.y += output[Y];
		control_client::write_state(state);
		std::this_thread::sleep_for(0.75s);
	}
	// Return current state if it failed
	return control_client::state();
}

Coordinate downAlign(int attempts, Task task, int camera)
{
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting function");
	int threshold = attempts/2;
	int original = attempts;
	float x_avg = 0.;
	float y_avg = 0.;
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Entering for loop, iterating %d times", original);
	for (int i = 0; i < original; i++)
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Getting State");
		State state = control_client::state();
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Received state at %f %f %f %f %f %f",state.x,state.y,state.z,state.yaw, state.pitch, state.roll);
		float dist = control_client::depth();
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting observation code");
		Observation obs = vision_client::vision(task, camera);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		if (obs.prob > 0.5)
		{
			// Get body offsets.
			float x = std::tan(obs.vangle*M_PI/180.)*(dist);
			float y = std::tan(obs.hangle*M_PI/180.)*(dist);

			// Attempt to align dropper instead of down cam
			x -= DROPPER_X_OFFSET;
			y -= DROPPER_Y_OFFSET;
			
			// Convert body offsets to inertial frame.
			float input[3] = {x, y, 0.};
			float angles[3] = {state.yaw, state.pitch, 
				state.roll};
			float output[3];
			bodyToInertial(input, angles, output);

			// Add to average.
			x_avg += state.x + output[0];
			y_avg += state.y + output[1];
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug offset @ %f %f", input[0], input[1]);
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug state @ %s", state_to_text(state).c_str());
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug angles @ %f %f %f", angles[0], angles[1], 
					angles[2]);
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Valid observation offset @ %f %f depth @ %f\n", output[0], 
					output[1], dist);
		}
		else 
		{
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed detection means attempts is decreased.");
			attempts -= 1;
		}
		std::this_thread::sleep_for(0.75s);
	}

	if (attempts == 0)
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for down alignment.");
		return std::make_pair(-999., -999.);
	}
	x_avg /= (float)attempts;
	y_avg /= (float)attempts;
	return std::make_pair(x_avg, y_avg);
}

bool frontContinuousAlign(float min_seconds, Task task, int camera, float thres)
{

	/* 
	 * Continuously inference and move sub until object is in center of camera
	 * First move vertically then horizontally
	 */

	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Enter front continuous alignment call");
	bool isValigned = frontContinuousValign(min_seconds, task, camera, thres);
	bool isHaligned = frontContinuousHalign(min_seconds, task, camera, thres);
	return (isValigned && isHaligned);
}

bool frontContinuousValign(float min_seconds, Task task, int camera, float thres)
{
	/* 
	* Use continuous inferencing and vertical readjustment until time is up or object is centered.
	* Assumes sub has no pitch or roll.
	*/
	float elapsed_seconds = 0.;
	float max_seconds = 30.;
	int attempts = 5;

	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting vertical continuous alignment.");
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Minimum of %f seconds, maximum of %f seconds", min_seconds, max_seconds);
	warmupInference(1, task, camera);
	auto start_time = std::chrono::high_resolution_clock::now();

	while (rclcpp::ok())
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Getting State");
		State state = control_client::state();
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Received state at %f %f %f %f %f %f",state.x,state.y,state.z,state.yaw, state.pitch, state.roll);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting observation code");
		Observation obs = vision_client::vision(task, camera);
		float dist = obs.dist;
		if (!isValidDistance(dist)) 
			dist = 4;
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		if (obs.prob > 0.5)
		{
			// If object is already centered, then terminate the loop.
			if (elapsed_seconds > min_seconds)
			{
				if (isValigned(obs.r, camera, thres))
				{
					// Stay in aligned position in case the state overshoots
					RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Object is vertically aligned.");
					float currentDepth = control_client::depth();
					control_client::write_depth(currentDepth);
					return true;
				}
				else RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Object is not vertically aligned.");
			}
			// Get vertical offset.
			float z = std::tan(obs.vangle*M_PI/180.)*(dist);

			// Move to align.
			float depth = control_client::depth() - z;
			control_client::write_depth(depth);

			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug state @ %s", state_to_text(state).c_str());
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Valid observation offset @ %f distance @ %f\n", z, dist);
		}
		else 
		{
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed detection means attempts is decreased.");
			attempts -= 1;
		}
		if (attempts == 0)
		{
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for vertical alignment.");
			return false;
		}

		// Check if time is up
		float elapsed_seconds = seconds_since_start(start_time);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "%f seconds since vertical align call.", elapsed_seconds);

		// If minimum seconds > maximum seconds, it will prioritize the minimum first, going over the max
		if (elapsed_seconds >= min_seconds && elapsed_seconds >= max_seconds) break;
		std::this_thread::sleep_for(0.75s);
	}

	return false;
}

bool frontContinuousHalign(float min_seconds, Task task, int camera, float thres)
{
	/* 
	* Use continuous inferencing and horizontal readjustment until time is up or object is centered.
	* Is not compatible with pitch and roll.
	*/

	State now = control_client::state();
	float elapsed_seconds = 0.;
	float max_seconds = 30.;
	int attempts = 5;

	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting horizontal continuous alignment.");
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Minimum of %f seconds, maximum of %f seconds", min_seconds, max_seconds);
	warmupInference(1, task, camera);
	auto start_time = std::chrono::high_resolution_clock::now();

	while (rclcpp::ok())
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Getting State");
		State state = control_client::state();
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Received state at %f %f %f %f %f %f",state.x,state.y,state.z,state.yaw, state.pitch, state.roll);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting observation code");
		Observation obs = vision_client::vision(task, camera);
		float dist = obs.dist;
		if (!isValidDistance(dist)) 
			dist = 4;
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		if (obs.prob > 0.5)
		{
			// If object is already centered, then terminate the loop.
			if (elapsed_seconds > min_seconds)
			{
				if (isHaligned(obs.c, camera, thres))
				{
					// Stay in aligned position in case the original state overshoots
					RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Object is horizontally aligned.");
					control_client::write_state(state);
					return true;
				}
				else RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Object is not horizontally aligned.");
			}
			// Get body offsets.
			float y = std::tan(obs.hangle*M_PI/180.)*(dist);

			// Convert body offsets to inertial frame.
			float input[3] = {0., y, 0.};
			float angles[3] = {state.yaw, 0., 0.};
			float output[3];
			bodyToInertial(input, angles, output);

			// Move to align.
			state.x += output[0];
			state.y += output[1];
			control_client::write_state(state);

			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug offset @ %f", input[1]);
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug state @ %s", state_to_text(state).c_str());
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug angles @ %f %f %f", angles[0], angles[1], 
					angles[2]);
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Valid observation offset @ %f %f distance @ %f\n", output[0], 
					output[1], dist);
		}
		else 
		{
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed detection means attempts is decreased.");
			attempts -= 1;
		}
		if (attempts == 0)
		{
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for horizontal alignment.");
			return false;
		}

		// Check if time is up
		float elapsed_seconds = seconds_since_start(start_time);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "%f seconds since horizontal align call.", elapsed_seconds);

		// If minimum seconds > maximum seconds, it will prioritize the minimum first, going over the max
		if (elapsed_seconds >= min_seconds && elapsed_seconds >= max_seconds) break;
		std::this_thread::sleep_for(0.75s);
	}

	return false;
}

bool frontConsecutiveAngleAlign(int attempts, Task task, int camera)
{
	/* 
	 * Use consecutive inferencing and turning until attempts are up
	 */

	for (int i = 0; i < attempts; i++)
	{
		float angle = align(4, task, camera);
		if (isValidAngle(angle))
			setAngle(angle);
		else
			return false;
	}
	return true;
}

bool downContinuousAlign(float min_seconds, Task task, int camera, float thres)
{
	/* Use continuous inferencing and readjustment until time is up or object is centered */
	float elapsed_seconds = 0.;
	float max_seconds = 30.;
	int attempts = 5;

	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting down continuous alignment.");
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Entering loop for a minimum of %f seconds, maximum of %f seconds", min_seconds, max_seconds);
	warmupInference(1, task, camera);
	auto start_time = std::chrono::high_resolution_clock::now();

	while (rclcpp::ok())
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Getting State");
		State state = control_client::state();
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Received state at %f %f %f %f %f %f",state.x,state.y,state.z,state.yaw, state.pitch, state.roll);
		float dist = control_client::depth();
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Starting observation code");
		Observation obs = vision_client::vision(task, camera);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		if (obs.prob > 0.5)
		{
			// If object is already centered, then terminate the loop.
			if (elapsed_seconds > min_seconds && objIsCentered(obs.r, obs.c, camera, thres))
			{
				// Stay in current position
				control_client::write_state(state);
				return true;
			}
			// Get body offsets.
			float x = std::tan(obs.vangle*M_PI/180.)*(dist);
			float y = std::tan(obs.hangle*M_PI/180.)*(dist);

			// Attempt to align dropper instead of down cam
			x -= DROPPER_X_OFFSET;
			y -= DROPPER_Y_OFFSET;

			// Convert body offsets to inertial frame.
			float input[3] = {x, y, 0.};
			float angles[3] = {state.yaw, state.pitch, 
				state.roll};
			float output[3];
			bodyToInertial(input, angles, output);

			// Move to align.
			state.x += output[0];
			state.y += output[1];
			control_client::write_state(state);

			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug offset @ %f %f", input[0], input[1]);
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug state @ %s", state_to_text(state).c_str());
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Debug angles @ %f %f %f", angles[0], angles[1], 
					angles[2]);
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Valid observation offset @ %f %f depth @ %f\n", output[0], 
					output[1], dist);
		}
		else 
		{
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed detection means attempts is decreased.");
			attempts -= 1;
		}
		if (attempts == 0)
		{
			RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for down alignment.");
			return false;
		}

		// Check if time is up
		float elapsed_seconds = seconds_since_start(start_time);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "%f seconds since down align call.", elapsed_seconds);

		// If minimum seconds > maximum seconds, it will prioritize the minimum first, going over the max
		if (elapsed_seconds >= min_seconds && elapsed_seconds >= max_seconds) break;
		std::this_thread::sleep_for(0.75s);
	}

	return false;
}

bool testObjIsCentered(int attempts, Task task, int camera, float thres)
{
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Testing if object is centered.");
	int original = attempts;
	float avg_r = 0.;
	float avg_c = 0.;
	for (int i = 0; i < original; i++)
	{
		Observation obs = vision_client::vision(task, camera);
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Observation @ %s", obs.text().c_str());
		
		if (obs.prob > 0.30)
		{
			avg_r += obs.r;
			avg_c += obs.c;
		}
		else 
		{
			attempts -= 1;
		}
		// std::this_thread::sleep_for(0.25s);
	}

	if (attempts == 0)
	{
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Failed to pass threshold for alignment.");
		return false;
	}

	avg_r /= attempts;
	avg_c /= attempts;

	return objIsCentered(avg_r, avg_c, camera, thres);
}

bool objIsCentered(float r, float c, int camera, float thres)
{
	/* 
	* Calculate if object is in desired threshold of center of image 
	* r is the y coordinate of the bounding box center, c is the x coordinate
	*/

	bool isCentered = (isValigned(r, camera, thres) && isHaligned(c, camera, thres));

	if (isCentered) RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Object is centered.");
	else RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Object is not centered.");

	return isCentered;
}

bool isValigned(float r, int camera, float thres)
{
	/* 
	* Calculate if object is on same horizontal plane as camera
	* r is the y coordinate of the bounding box center
	*/
	float y_mid, y_thres;

	if (camera == FRONT)
	{
		y_mid = FIMG_DIM_RES[0] / 2;
		y_thres = FIMG_DIM_RES[0] * thres;
	}
	else if (camera == DOWN)
	{
		y_mid = DIMG_DIM_RES[0] / 2;
		y_thres = DIMG_DIM_RES[0] * thres;
	}

	float ymin = y_mid - y_thres/2;
	float ymax = y_mid + y_thres/2;

	return (r >= ymin && r <= ymax);
}

bool isHaligned(float c, int camera, float thres)
{
	/* 
	* Calculate if object is on same vertical plane as camera
	* c is the x coordinate of the bounding box center
	*/
	float x_mid, x_thres;

	if (camera == FRONT)
	{
		x_mid = FIMG_DIM_RES[1] / 2;
		x_thres = FIMG_DIM_RES[1] * thres;
	}
	else if (camera == DOWN)
	{
		x_mid = DIMG_DIM_RES[1] / 2;
		x_thres = DIMG_DIM_RES[1] * thres;
	}

	float xmin = x_mid - x_thres/2;
	float xmax = x_mid + x_thres/2;

	return (c >= xmin && c <= xmax);
}

void setForward(float dist)
{
	State now = control_client::state();
	float input[3] = {dist, 0., 0.};
	float angles[3] = {now.yaw, 0., 0.};
	float output[3];
	bodyToInertial(input, angles, output);
	now.x += output[0];
	now.y += output[1];
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting forward state @ %s", state_to_text(now).c_str());
	move(now);
}

void setHorizontal(float dist)
{
	State now = control_client::state();
	float input[3] = {0., dist, 0.};
	float angles[3] = {now.yaw, 0., 0.};
	float output[3];
	bodyToInertial(input, angles, output);
	now.x += output[0];
	now.y += output[1];
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting horizontal state @ %s", state_to_text(now).c_str());
	move(now);
}

void setForwardAtDepth(float dist, float depth)
{
	disableAltitudeControl();

	State now = control_client::state();
	float input[3] = {dist, 0., 0.};
	float angles[3] = {now.yaw, 0., 0.};
	float output[3];
	bodyToInertial(input, angles, output);
	now.x += output[0];
	now.y += output[1];
	now.z = depth;
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting forward with depth state @ %s", state_to_text(now).c_str());
	move(now);
}

void setHorizontalAtDepth(float dist, float depth)
{
	disableAltitudeControl();

	State now = control_client::state();
	float input[3] = {0., dist, 0.};
	float angles[3] = {now.yaw, 0., 0.};
	float output[3];
	bodyToInertial(input, angles, output);
	now.x += output[0];
	now.y += output[1];
	now.z = depth;
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting horizontal with depth state @ %s", state_to_text(now).c_str());
	move(now);
}
void setAngle(float angle)
{
	State now = control_client::state();
	now.yaw = angle;
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting angle state @ %s.", state_to_text(now).c_str());
	move(now);
}

void addAngle(float angle)
{
	State now = control_client::state();
	now.yaw = angleAdd(now.yaw, angle);
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting add angle state @ %s.", state_to_text(now).c_str());
	move(now);
}

void continuousSpin(int num_90_turns)
{
	/* 
	 * Continuously spin by writing a new state once Nemo gets within 45 degrees to give the illusion of a continuous spin.
	 * Can be used for spinning in gate to save time.
	 */

	// Hold x and y
	State original = control_client::state();
	float original_x = original.x;
	float original_y = original.y;
	float current_angle = original.yaw;
	float desired_angle = angleAdd(current_angle, 90.);

	for (int i = 0; i < num_90_turns; i++)
	{
		// Tell Nautical to move to desired angle state
		State desired = control_client::state();
		desired.x = original_x;
		desired.y = original_y;
		desired.yaw = desired_angle;
		RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Spin %d to %s", i + 1, state_to_text(desired).c_str());
		control_client::write_state(desired);

		bool completedTurn = false;
		while (!completedTurn && rclcpp::ok())
		{
			// Preemptively Jump into the next turn to continuously spin
			State now = control_client::state();
			if (std::fabs(angleDifference(now.yaw, desired.yaw)) < 45.)
				completedTurn = true;
			else std::this_thread::sleep_for(1s);
		}
		desired_angle = angleAdd(desired_angle, 90.);
	}
}

bool isValidAngle(float angle)
{
	return std::fabs(angle) < 240.;
}

void setCoordinate(Coordinate coordinate)
{
	State now = control_client::state();
	now.x = coordinate.first;
	now.y = coordinate.second;
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting coordinate state @ %s.", state_to_text(now).c_str());
	move(now);
}

void addCoordinate(Coordinate coordinate)
{
	State now = control_client::state();
	float input[3] = {coordinate.first, coordinate.second, 0.};
	float angles[3] = {now.yaw, 0., 0.};
	float output[3];
	bodyToInertial(input, angles, output);
	now.x += output[0];
	now.y += output[1];
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Setting add coordinate state @ %s.", state_to_text(now).c_str());
	move(now);
}

bool isValidCoordinate(Coordinate coordinate)
{
	return std::fabs(coordinate.first) < 50. && std::fabs(coordinate.second) < 50.;
}

bool isValidOffsetCoordinate(Coordinate coordinate)
{
	return std::fabs(coordinate.first) < 5. && std::fabs(coordinate.second) < 5.;
}

void warmupInference(int attempts, Task task, int camera)
{
	RCLCPP_INFO(rclcpp::get_logger("sub_mission"), "Warming up ML model.");
	for (int i = 0; i < attempts; i++)
		Observation obs = vision_client::vision(task, camera);
}

void disableAltitudeControl()
{
	/* Tell Nautical to override depth from bottom to instead maintain Z/depth from surface */
	control_client::write_depth(-1.);
	std::this_thread::sleep_for(1s);
}

void move(const State &dest)
{
	control_client::write_state(dest);
	bool quit = false;
	while (!quit && rclcpp::ok())
	{
		State now = control_client::state();
		if (std::fabs(dest.x - now.x) > 1.)
			std::this_thread::sleep_for(1s);
		else if (std::fabs(dest.y - now.y) > 1.)
			std::this_thread::sleep_for(1s);
		else if (std::fabs(angleDifference(dest.yaw, now.yaw)) > 5.)
			std::this_thread::sleep_for(1s);
		else
		{
			quit = true; 
			std::this_thread::sleep_for(1s);
		}
	}
}