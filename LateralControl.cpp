#include "LateralControl.h"
#include "Calibration.h"

// DEBUG (partially solved): for unknown reasons, veh_desired_active_lane_change = 1 not triggered by this code but from VISSIM SetValue. VISSIM LC decision still apply??
//                           partially solved by adding curr_data.veh_desired_active_lane_change = 0;

void BaselineADF::LateralControl(car_data& curr_data, car_data& prev_data) { // document 3#	
	
	curr_data.visualize_LC_decision = prev_data.visualize_LC_decision; // to visulize LC
	curr_data.veh_desired_active_lane_change = 0;  // to ignore VISSIM LC decision
	
	// trigger a simple LC
	if (curr_data.veh_active_lane_change == 0 && curr_data.veh_BADF_lane_change_decision != 0) {
		// convert the 2 or -2 value to 1 or -1 for simple lane change
		if (curr_data.veh_BADF_lane_change_decision>0) {
			curr_data.veh_desired_active_lane_change = 1;
		}
		else
		{
			curr_data.veh_desired_active_lane_change = -1;
		}
		//curr_data.veh_desired_active_lane_change = curr_data.veh_BADF_lane_change_decision; // this should only be triigered once
		if (curr_data.veh_BADF_mandatory_lane_change_decision != 0) {
			curr_data.visualize_LC_decision = 1;  // visualize mandatory LC
		}
		else {
			curr_data.visualize_LC_decision = -1; // visualize discretionary LC
		}
	}
	else {
		// 	recognize a simple LC is finished
		if (curr_data.veh_active_lane_change == 0 && curr_data.visualize_LC_decision != 0) {
			curr_data.visualize_LC_decision = 0;
		}	
	}

	/*
	// trigger a simple LC
	if (curr_data.veh_active_lane_change == 0 && curr_data.veh_BADF_lane_change_decision != 0) {
		curr_data.veh_desired_active_lane_change = curr_data.veh_BADF_lane_change_decision; // this should only be triigered once
		curr_data.blue_veh = true;
	}
	else {
		// 	recognize a simple LC is finished
		if (curr_data.veh_active_lane_change == 0 && curr_data.blue_veh == true) {
			curr_data.blue_veh = false;
		}
	}
	*/

	//curr_data.veh_BADF_LC_Status = curr_data.veh_active_lane_change; // link veh_BADF_LC_Status with veh_active_lane_change

	// OUTPUT: set veh_desired_active_lane_change (to trigger VISSIM simple lane change), veh_BADF_LC_Status (feedback to LaneChangeDecision), visualize_LC_decision (for visualization)
}