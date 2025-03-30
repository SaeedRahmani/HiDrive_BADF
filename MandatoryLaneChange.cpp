#include "MandatoryLaneChange.h"
#include "Calibration.h"
#include <cmath>


// TO CONFIRM: detection range of NVEH in DLL (250m?) and whether need to update detection via COM
// TO CONFIRM: remove the max_force_lane_change condition, to be check if that is ok
// TO IMPLEMENT: mandatory LC not needed for on-ramp, but for off-ramp or lane-drop --> need a network 
//               how to generalizing mandatory LC decision regardless of network?

void LaneChangeDecision::MandatoryLaneChange(car_data& curr_data, car_data& prev_data) {
	/* ============================= PARAMETERS ================================ */
	// default parameters for motorway
	double min_distance_front_mandatory_LC = 5.0; // acceptable distance gap with a front vehicle in target lane for mandatory lane change [m]
	double min_distance_rear_mandatory_LC = 5.0; // acceptable distance gap with a rear vehicle in target lane for mandatory lane change [m]
	double min_headway_front_mandatory_LC = 0.5; //acceptable time gap with a front vehicle in target lane for mandatory lane change [s] [Lin] should be 0.25 second? [Jie] L3Pilot uses 0.25, but even 0.5 might be too critical for VISSIM
	double min_headway_rear_mandatory_LC = 0.5; // acceptable time gap with a rear vehicle in target lane for mandatory lane change [s] [Lin] should be 0.25 second? [Jie] L3Pilot uses 0.25, but even 0.5 might be too critical for VISSIM
	double min_TTC_mandatory_LC = 3; // adaption1, minimal TTC to lead vehicle in target lane for mandatory lane change [s]
	double max_deceleration_mandatory_LC = -3.0; // maximal deceleration of rear vehicle for mandatory lane change [m/s2]

	// scenario-based parameters
	if (curr_data.urban_scenario == 1) {
		
	}

	/* ============================= Initialization ================================ */
	int mandatory_LC_direction = 0;
	double required_rear_deceleration = 0.0;
	/*
	// a mandatory lane change when the lane end distance is within 500m 
	if (curr_data.veh_current_link == 2 && curr_data.veh_current_lane == 1) { // specific to the ramp merge network by TNO and VTEC, [Lin] later use lane_end_distance > 0	
		// for on-ramp, mandatory LC is considered when the ego vehicle is in the acclereration lane
		// for off-ramp or lane-drop, consider curr_data.veh_lane_end_distance < MLC_Distance, not yet implemented	
		curr_data.in_mandatory_LC_area = true;
		mandatory_LC_direction = 1; //[Lin] the direction can be parameterized in the simData.h 
									//[Jie] Can the direction be determined from vehicle route? in order to generalize mandatory LC for different networks
	} */

	// lane change rules for urban
	if (curr_data.veh_use_preferred_lane == 1 && curr_data.veh_preferred_rel_lane != 0) {
		curr_data.in_mandatory_LC_area = true;
		mandatory_LC_direction = curr_data.veh_preferred_rel_lane;
		curr_data.veh_BADF_mandatory_lane_change_decision = mandatory_LC_direction;
	}

	if (curr_data.in_mandatory_LC_area) { // in acceleration lane
	
		// keep an existing mandatory LC decision until it is executed
		if (prev_data.veh_BADF_mandatory_lane_change_decision != 0) {		
			curr_data.veh_BADF_mandatory_lane_change_decision = prev_data.veh_BADF_mandatory_lane_change_decision;
			if (prev_data.veh_BADF_LC_Status != 0) { // LC is executed [Lin] precisely, LC is ongoing, not completed yet			
				curr_data.veh_BADF_mandatory_lane_change_decision = 0; // [Lin] if the MLC require more than one lane change, then this condition is not valid
			}
		}

		// otherwise check if to make a mandatory LC decision
		else {
			curr_data.veh_BADF_mandatory_lane_change_decision = mandatory_LC_direction;

			// gap acceptance for left LC
			if (curr_data.veh_BADF_mandatory_lane_change_decision > 0) {
				
				// lead vehicle			
				if (curr_data.left_lead_veh_id > 0) {			
					if (curr_data.left_lead_veh_distance < min_distance_front_mandatory_LC || curr_data.left_lead_veh_headway < min_headway_front_mandatory_LC
						|| (curr_data.activate_adaption1_mLC_TTC == true && curr_data.left_lead_veh_TTC < min_TTC_mandatory_LC)) {
						curr_data.veh_BADF_mandatory_lane_change_decision = 0;
						curr_data.veh_lead_gap_reject = true;
					}
				} 
				
				// rear vehicle
				if (curr_data.left_rear_veh_id > 0) {
					
					// required deceleration of rear vehicle
					if (curr_data.left_rear_veh_distance > 0) {
						if (curr_data.left_rear_veh_rel_velocity >= 0) { // a slower or same speed rear vehicle					
							required_rear_deceleration = 0.0;
						}
						else {
							//use kinematic euqation for now, later can be changed to simplified PID contorl euqation for more accurate deceleration of BADF
							required_rear_deceleration = -(pow((curr_data.veh_velocity - curr_data.left_rear_veh_rel_velocity), 2.0) - pow((curr_data.veh_velocity), 2.0)) / (2 * curr_data.left_rear_veh_distance);
							//required_rear_deceleration = -(pow(curr_data.right_rear_veh_rel_velocity, 2.0)) / (2 * curr_data.right_rear_veh_distance);  // [Lin] this is incorrect since this is the acceleration to avoid collision, but not the actual response of BADF which is to maintain a constant time gap.  Also, here the simplified square results in a large error.
						}
					}
					else {
						required_rear_deceleration = -9.81;
					}

					if (curr_data.left_rear_veh_distance < min_distance_rear_mandatory_LC || curr_data.left_rear_veh_headway < min_headway_rear_mandatory_LC 
						|| required_rear_deceleration < max_deceleration_mandatory_LC
						|| (curr_data.activate_adaption1_mLC_TTC == true && curr_data.left_rear_veh_TTC < min_TTC_mandatory_LC)) {
						curr_data.veh_BADF_mandatory_lane_change_decision = 0;
						curr_data.veh_rear_gap_reject = true;
					}
				}
			}
			
			// gap acceptance for right LC
			else {
				
				// lead vehicle			
				if (curr_data.right_lead_veh_id > 0) {
					if (curr_data.right_lead_veh_distance < min_distance_front_mandatory_LC || curr_data.right_lead_veh_headway < min_headway_front_mandatory_LC
						|| (curr_data.activate_adaption1_mLC_TTC == true && curr_data.right_lead_veh_TTC < min_TTC_mandatory_LC)) {
						curr_data.veh_BADF_mandatory_lane_change_decision = 0;
						curr_data.veh_lead_gap_reject = true;
					}
				}
				// rear vehicle
				if (curr_data.right_rear_veh_id > 0) {

					// required deceleration of rear vehicle
					if (curr_data.right_rear_veh_distance > 0) {
						if (curr_data.right_rear_veh_rel_velocity >= 0) { // a slower or same speed rear vehicle					
							required_rear_deceleration = 0.0;
						}
						else {
							//use kinematic euqation for now, later can be changed to simplified PID contorl euqation for more accurate deceleration of BADF
							required_rear_deceleration = -(pow((curr_data.veh_velocity - curr_data.right_rear_veh_rel_velocity), 2.0) - pow((curr_data.veh_velocity), 2.0)) / (2 * curr_data.right_rear_veh_distance);
							//required_rear_deceleration = -(pow(curr_data.right_rear_veh_rel_velocity, 2.0)) / (2 * curr_data.right_rear_veh_distance);  // [Lin] this is incorrect since this is the acceleration to avoid collision, but not the actual response of BADF which is to maintain a constant time gap.  Also, here the simplified square results in a large error.
						}
					}
					else {
						required_rear_deceleration = -9.81;
					}
					if (curr_data.right_rear_veh_distance < min_distance_rear_mandatory_LC || curr_data.right_rear_veh_headway < min_headway_rear_mandatory_LC
						|| required_rear_deceleration < required_rear_deceleration
						|| (curr_data.activate_adaption1_mLC_TTC == true && curr_data.right_rear_veh_TTC < min_TTC_mandatory_LC)) {
						curr_data.veh_BADF_mandatory_lane_change_decision = 0;
						curr_data.veh_rear_gap_reject = true;
					}
				}	
			}
			
			// conditions to cancel a mandatory LC decision
			if (curr_data.veh_BADF_mandatory_lane_change_decision != 0) {
				if (prev_data.veh_BADF_LC_Status != 0) { // no mandatory LC during an on-going LC (not active due to that veh_BADF_LC_Status is not set)
					curr_data.veh_BADF_mandatory_lane_change_decision = 0;
				}
			}
		}
	}
};
