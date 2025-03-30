#include "DiscretionaryLaneChange.h"
#include "Calibration.h"
#include <cmath>

void LaneChangeDecision::DiscretionaryLaneChange(car_data& curr_data, car_data& prev_data) {

	/* ============================= PARAMETERS ================================ */
	// default parameters for motorway
	double min_distance_front_discretionary_LC = 5.0; // acceptable distance gap with a front vehicle in target lane for discretionary lane change [m]
	double min_distance_rear_slow_discretionary_LC = 5.0; // acceptable distance gap with a slow rear vehiclein target lane for discretionary lane change [m]
	double min_distance_rear_fast_discretionary_LC = 5.0; // acceptable distance gap with a fast rear vehiclein target lane for discretionary lane change [m]
	double min_headway_front_discretionary_LC = 0.5; // acceptable time gap with a front vehicle in target lane for discretionary lane change [s]
	double min_headway_rear_discretionary_LC = 0.5; // acceptable time gap with a rear vehicle in target lane for discretionary lane change [s]
	double max_deceleration_discretionary_LC = -3.0; // maximal deceleration of rear vehicle for discretionary lane change [m/s2]
	double threshold_difference_to_set_speed = 10.8 / 3.6; // threshold of difference to BADF set speed to trigger discretionary lane change [km/h]
	double threshold_headway_to_lead_vehicle = 4.0;  // threshold of headway to leading vehicle to trigger discretionary lane change [s]
	double threshold_headway_to_adjacent_lead_vehicle = 3.0;  // threshold of headway to adjacent leading vehicle to trigger discretionary lane change [s]
	double threshold_distance_to_adjacent_lead_vehicle = 100.0;  // threshold of distance to adjacent leading vehicle to trigger discretionary lane change [s]
	double threshold_speed_congestion = 60.0 / 3.6;  // threshold speed for congestion situation [m/s]: 60 km/h for motorway, value for urban need to be checked
	double threshold_time_since_last_LC = 10.0; // threshold of time since the last LC to initiate a new LC [s]
	double threshold_current_braking = -1.5; // threshold of braking rate to cancel a discretionary LC[m/s2]
	double NoLaneChangeDistance = 150; // No discretionary lane change within this distance to the signal head or conflict area, urban scenario only
	double warming_up_period = 5.0; // simulation warming up period [s]

	// scenario-based parameters
	if (curr_data.urban_scenario == 1) {
		threshold_speed_congestion = 15.0 / 3.6;
		threshold_difference_to_set_speed = 5 / 3.6;
	}

	/* ============================= Initialization ================================ */
	// local vairables
	double difference_to_set_speed = 0.0;
	bool has_left_lane = false;
	bool has_right_lane = false;
	double required_rear_deceleration = 0.0;
	bool intersection = false;
	double min_distance_rear_discretionary_LC = 0.0;

	// update has_left_lane and has_right_lane
	if (curr_data.veh_current_lane < curr_data.veh_n_lanes) {
		has_left_lane = true;
	}

	if (curr_data.veh_current_lane > 1) {
		has_right_lane = true;
	}

	// recoginze if a vehicle is at the intersection by the distance to signals/conflict area is smaller than 150 meters)
	if ((curr_data.veh_signal_distance > 0 && curr_data.veh_signal_distance <= NoLaneChangeDistance) || (curr_data.conf_area_dist[0] > 0 && curr_data.conf_area_dist[0] <= NoLaneChangeDistance)) {
		intersection = true;
	}

	// decision for discretionary lane change
	difference_to_set_speed = curr_data.veh_desired_velocity - curr_data.veh_velocity;

	// right discretionary LC
	if (!intersection && has_right_lane &&
			(curr_data.right_lead_veh_distance > threshold_distance_to_adjacent_lead_vehicle || curr_data.right_lead_veh_headway > threshold_headway_to_adjacent_lead_vehicle)) { // right discretionary LC

			curr_data.veh_BADF_discretionary_lane_change_decision = changeRight;

			// gap acceptance
			if (curr_data.right_lead_veh_id > 0) { // lead vehicle		
				if (curr_data.right_lead_veh_distance < min_distance_front_discretionary_LC || curr_data.right_lead_veh_headway < min_headway_front_discretionary_LC) {				
					curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
				}
			}

			if (curr_data.right_rear_veh_id > 0) { // rear vehicle
				
				// required deceleration of rear vehicle
				if (curr_data.right_rear_veh_distance > 0) {				
					if (curr_data.right_rear_veh_rel_velocity >= 0) { // a slower or same speed rear vehicle					
						required_rear_deceleration = 0.0;
						min_distance_rear_discretionary_LC = min_distance_rear_slow_discretionary_LC;
					}
					else {
						min_distance_rear_discretionary_LC = min_distance_rear_fast_discretionary_LC;
						//use kinematic euqation for now, later can be changed to simplified PID contorl euqation for more accurate deceleration of BADF
						required_rear_deceleration = -(pow((curr_data.veh_velocity - curr_data.right_rear_veh_rel_velocity), 2.0) - pow((curr_data.veh_velocity), 2.0)) / (2 * curr_data.right_rear_veh_distance); 
						//required_rear_deceleration = -(pow(curr_data.right_rear_veh_rel_velocity, 2.0)) / (2 * curr_data.right_rear_veh_distance);  // [Lin] this is incorrect since this is the acceleration to avoid collision, but not the actual response of BADF which is to maintain a constant time gap.  Also, here the simplified square results in a large error.
					}
				}
				else {
					required_rear_deceleration = -9.81;
				}

				if (curr_data.right_rear_veh_distance < min_distance_rear_discretionary_LC || curr_data.right_rear_veh_headway < min_headway_rear_discretionary_LC
					|| required_rear_deceleration < max_deceleration_discretionary_LC) {				
					curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
				}
			}

			// conditions to cancel right discretionary LC
			 if (curr_data.veh_BADF_discretionary_lane_change_decision == changeRight) {
				if (curr_data.veh_lane_end_distance > 0 && curr_data.veh_current_lane == 2) { // no right LC to the acceleration lane			
				 curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
				}				
			}
		}

	// left discretionary LC
	if (!intersection && difference_to_set_speed > threshold_difference_to_set_speed && curr_data.veh_time_headway < threshold_headway_to_lead_vehicle) {
			// motivation for left discreationary LC: speed is restricted in the current lane
		
		//if (has_left_lane &&
			//(curr_data.left_lead_veh_distance > threshold_distance_to_adjacent_lead_vehicle || curr_data.left_lead_veh_headway > threshold_headway_to_adjacent_lead_vehicle)) {
		if (has_left_lane) {
			curr_data.veh_BADF_discretionary_lane_change_decision = changeLeft;

			// gap acceptance
			
			if (curr_data.left_lead_veh_id > 0) { // lead vehicle
				if (curr_data.left_lead_veh_distance < min_distance_front_discretionary_LC || curr_data.left_lead_veh_headway < min_headway_front_discretionary_LC) {
					curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
				}
			}
			
			if (curr_data.left_rear_veh_id > 0) { // rear vehicle			

				// required deceleration of rear vehicle
				if (curr_data.left_rear_veh_distance > 0) {
					if (curr_data.left_rear_veh_rel_velocity >= 0) {
						required_rear_deceleration = 0.0;
						min_distance_rear_discretionary_LC = min_distance_rear_slow_discretionary_LC;
					}
					else {
						min_distance_rear_discretionary_LC = min_distance_rear_fast_discretionary_LC;
						//use kinematic euqation for now, later can be changed to simplified PID contorl euqation for more accurate deceleration of BADF
						required_rear_deceleration = -(pow((curr_data.veh_velocity - curr_data.left_rear_veh_rel_velocity), 2.0) - pow((curr_data.veh_velocity), 2.0)) / (2 * curr_data.left_rear_veh_distance);
						//required_rear_deceleration = -(pow(curr_data.left_rear_veh_rel_velocity, 2.0)) / (2 * curr_data.left_rear_veh_distance);
					}
				}
				else {
					required_rear_deceleration = -9.81;
				}
				
				if (curr_data.left_rear_veh_distance < min_distance_rear_discretionary_LC || curr_data.left_rear_veh_headway < min_headway_rear_discretionary_LC
					|| required_rear_deceleration < max_deceleration_discretionary_LC) {
					curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
				}
			}
			// conditions to cancel left discretionary LC (cureently not relevent)
		}
	}
	
	// conditions to cancel any discretionary LC (regardless of left or right)
	if (curr_data.veh_BADF_discretionary_lane_change_decision != 0) {

		if (prev_data.veh_BADF_LC_Status != 0) { // no discretionary LC during an on-going LC		
			curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
		}
		if (curr_data.in_mandatory_LC_area) { // no discretionary LC in acceleration lane
			curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
		}
		if (curr_data.veh_velocity < threshold_speed_congestion) { // no discretionary LC in congested traffic
			curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
		}
		if (curr_data.time_since_last_LC < threshold_time_since_last_LC) { // no discretionary LC within a certain period after the last LC	
			curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
		}
		if (curr_data.timestep <= warming_up_period) { // no discretionary LC within warming up period of simulation
			curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
		}
		if (curr_data.veh_desired_acceleration <= threshold_current_braking) { // no discretionary LC when braking more than -1.5 m/s2
			curr_data.veh_BADF_discretionary_lane_change_decision = noChangeLane;
		}
		// TO CONFIRM: the condition about MRM is not implemented. Not sure if relevant for E&E. 
	}
};