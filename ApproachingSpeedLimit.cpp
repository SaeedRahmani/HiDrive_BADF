#include "ApproachingSpeedLimit.h"
#include "Calibration.h"
#include <cstdlib>
#include <math.h>
#include <limits>
#include <algorithm>
#include <vector>
#include "SimData.h"
#ifdef NAN
/* NAN is supported*/
#endif // NAN

void BaselineADF::ApproachingSpeedLimit(car_data& curr_data, car_data& prev_data) {
	/* The vehicle contorl toward speed limits is based on the ADFs in L3Pilot.
	This function applies for the situations that there is a reduced speed limit within 180 meters.
	when speed limit remains the same or will increase shortly, there is no speed limit control.
	When the speed limit increases, vehicle response needs to be tested!!!
	when the vehicle has not yet reached the desired speed (setspeed), then no speed limit controll will be applied.

	Model output are a_x_SPL and v_set, which will be sent to operational layer controller.
	NOTE: the L3Pilot model is able to work for multiple speed limits within 180m. However, it is impossible 
	in current implementation due to that VISSIM can only provide one speed limit information ahead.
	NOTE: single and multiple speed limit signs require different reset conditions depending on if the distance between 
	two signs is smaller than the look-ahead distance.  
	
	Urban case: to avoid reading incorreact reduced speed area on a different route, the routing decision need to be 
	placed further away than the look ahead distance of BADF/EADF; Overtaking(passing) from right is allowed in urban scenarios.
	*/



	/* questions:
	the acceleration output is based on a deceration that is required to brake constantly*/
	
	/* ============================= PARAMETERS ================================ */
	// default parameters for motorway
	double overtaking_speed_threshold = 60 / 3.6; // speed threshold to allow ovetakeing from right, defaut value 60 km/h [German rules]
	double min_headway_to_left_front = 1; // minimal time gap to left front leader [s]
	double setspeed_change_to_left_front = 0.2; // the changing rate of setspeed regarding the distance to left front leader
	double SpeedLimitControlDistance = 180.0; // distance range for speed limit control
	
	// scenario-based parameters
	if (curr_data.urban_scenario == 1) {
		SpeedLimitControlDistance = 60.0;
	}

	/* ============================= Initialization ================================ */
	double a_required = 0;
	double v_set = prev_data.veh_BADF_setspeed; 
	double a_x_SPL = NAN; 
	double a_required_set = prev_data.veh_BADF_SPL_a_required_set; // note, could be 999;
	bool SPL_Control = prev_data.veh_BADF_SPL_control; // by default: false

	if (curr_data.veh_current_speed_limit == -999) {
		curr_data.veh_current_speed_limit = curr_data.veh_desired_velocity;
	}
	else
	{
		curr_data.veh_current_speed_limit = prev_data.veh_current_speed_limit;
	}


	/* ============================= Not overtaking from right ================================ */
	double v_left_front = 999.0;
	double v_left_front_raw = curr_data.veh_velocity - curr_data.left_lead_veh_rel_velocity;
	if (curr_data.left_lead_veh_id > 0 && v_left_front_raw >= overtaking_speed_threshold && curr_data.urban_scenario != 1) {
			if (curr_data.left_lead_veh_distance > 0) { 
				// the setspeed considering left front vehicle's speed is a linear function of the distance betwwen two vehicles; a minimum distance is assumed to ensure the ego vehicle will synchronize 
				// the speed with left front vehicle before overtaking.
				// the minimum distance is 1 second that a lane change is still possible for the left front vehicle
				curr_data.minimum_distance_to_left_front = min_headway_to_left_front * curr_data.veh_velocity;
				double v_left_front_expected = v_left_front_raw + setspeed_change_to_left_front * (curr_data.left_lead_veh_distance - curr_data.minimum_distance_to_left_front);
				v_left_front = max(v_left_front_raw, v_left_front_expected);
			}
			else
			{
				v_left_front = v_left_front_raw; 
			}
	}
	else {
		v_left_front = 999.0;
	}


	/* ============================= speed limit control ================================ */
	// if there is a speed limit sign within perceived distance (default 180 m) and the speed limit is activated
		if (curr_data.veh_speed_limit_distance > 0 && curr_data.veh_speed_limit_distance <= SpeedLimitControlDistance && curr_data.veh_speed_limit_distance != 1) {  //&& prev_data.veh_speed_limit_distance >0
			if (curr_data.veh_speed_limit_value < curr_data.veh_current_speed_limit && curr_data.veh_velocity > curr_data.veh_speed_limit_value) { //if facing a lower speed limit and vehicle travel faster than the speed limit
				a_required = -(curr_data.veh_velocity * curr_data.veh_velocity - curr_data.veh_speed_limit_value * curr_data.veh_speed_limit_value) / (2 * curr_data.veh_speed_limit_distance - 1);
				// if a deceleration larger than 0.5 m/s^2 is required, take the previous setspeed
				if (std::abs(a_required) > 0.5) {
					v_set = min(prev_data.veh_BADF_setspeed, curr_data.veh_speed_limit_value);
					// if it is the first time to activate speed limit control or the desired acceleration is smaller than previous acceleration, use the desired acceleration as output; 
					if (!prev_data.veh_BADF_SPL_control || a_required < prev_data.veh_BADF_SPL_a_required_set) {
						a_required_set = a_required;
					}
					// otherwise, use the previous acceleration.
					a_x_SPL = a_required_set;
					SPL_Control = true;
					// [LIN] it means the maximum deceleration will be applied during the whole deceleration phase;
				}
				else
					// when a small deceleration is desired
				{
					// when it is first time to activate speed limit control, setspeed is the min of previous setspeed and the left front vehicle; prevent overtakeing from right 
					if (!prev_data.veh_BADF_SPL_control) {
						v_set = min(min(prev_data.veh_BADF_setspeed, curr_data.veh_current_speed_limit), v_left_front);
						a_x_SPL = 0;
					}
					else
					{
						// when it has been under speed limit control, setspeed use speed limit and consider the left front vehicle
						v_set = min(min(prev_data.veh_BADF_setspeed, curr_data.veh_speed_limit_value), v_left_front);
						a_x_SPL = prev_data.veh_BADF_SPL_a_required_set;
					}
				}
			}

			else
			{	// if the current speed limit is higher than setspeed
				if (curr_data.veh_speed_limit_value > prev_data.veh_BADF_setspeed) {
					v_set = prev_data.veh_BADF_setspeed;
				}
				else
				{
					v_set = min(min(curr_data.veh_current_speed_limit, prev_data.veh_BADF_setspeed), v_left_front);
				}
				a_x_SPL = NAN;

				SPL_Control = false;
				a_required_set = 0;
			}
		}
		else
		{
			// if it is the first timestep to activat BADF, set the current speed limit to BADF setspeed.
			if (v_set == 999) {
				v_set = curr_data.veh_current_speed_limit;
				SPL_Control = false;
				a_required_set = 0;
			}
			// else, considering the current speed limit and left front vehicle
			else
			{
					v_set = min(curr_data.veh_current_speed_limit, v_left_front); // generally consider left front leader for setspeed
			}

			

			// arriving the reduced speed area (single and multiple signs)
			// single speed limit sign (low speed limit), change the current speed limit, keep the previous setspeed
			if (curr_data.veh_speed_limit_distance < 0 && prev_data.veh_speed_limit_distance > 0 && prev_data.veh_speed_limit_distance != 1) {
				SPL_Control = false;
				a_required_set = 0;
				curr_data.veh_desired_velocity = v_set;
				curr_data.veh_current_speed_limit = prev_data.veh_speed_limit_value;
				// if it is one signle speed limit sign with higher speed limit, or the last sign with higher speed limit in the series, change the setspeed to the new speed limit when it arrive the signs;
				if (curr_data.veh_current_speed_limit > v_set) {
					v_set = curr_data.veh_current_speed_limit;
					curr_data.veh_desired_velocity = v_set;
				}
			}

			// multiple speed limit sign
			//if (prev_data.veh_speed_limit_distance!= -999 && prev_data.veh_speed_limit_distance != -1 && prev_data.veh_speed_limit_distance != 1 && curr_data.veh_speed_limit_distance != 1 && curr_data.veh_speed_limit_distance > prev_data.veh_speed_limit_distance ) {
			if (curr_data.veh_speed_limit_distance > prev_data.veh_speed_limit_distance && prev_data.veh_speed_limit_distance > 0 && prev_data.veh_speed_limit_distance != 1) {
				SPL_Control = false;
				a_required_set = 0;
				curr_data.veh_desired_velocity = v_set;
				curr_data.veh_current_speed_limit = prev_data.veh_speed_limit_value;
			}

			// the next time step of arriving the end of reduce speed area, ignore the received reset desired speed and still get the current speed limit
			if (curr_data.veh_desired_velocity != prev_data.veh_desired_velocity && curr_data.veh_speed_limit_distance == 1) {
				curr_data.veh_desired_velocity = curr_data.veh_current_speed_limit;
			}


		}

		if (curr_data.veh_velocity < v_set) {
			SPL_Control = false;
			a_required_set = 0;
			a_x_SPL = NAN;
		}
	
	curr_data.veh_BADF_SPL_a_required_set = a_required_set;
	curr_data.veh_BADF_SPL_control = SPL_Control;
	curr_data.veh_BADF_desired_acceleration_speed_limit = a_x_SPL;
	curr_data.veh_BADF_setspeed = v_set;

};
