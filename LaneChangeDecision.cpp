#include "LaneChangeDecision.h"
#include "Calibration.h"
#include <cmath>
#include <windows.h>
#include <string>  // for sprintf

// TO CONFIRM: sync variables with CarFollowing module: veh_distance_headway, veh_time_headway
// TO IMPLEMENT: LC direction in SimData.h
// DEBUG: BADF removed at the end of acceleration lane when mandatory LC failed
// DEBUG: motorway vehicle does not change to inner lane in congested condition (due to too conservative dircretionary LC decision when speed difference is high?)
// DEBUG: crash observed (veh 444 at timestep 603) due to that no LC abortion or that gap acceptance conditions are too critical

void BaselineADF::LaneChangeDecision(car_data& curr_data, car_data& prev_data) {

	// set if adaptions for lane change model are active
	curr_data.activate_adaption1_mLC_TTC = true; // adaption1: introduce TTC condition into mandatory LC gap accpetance

	curr_data.veh_BADF_LC_Status = curr_data.veh_active_lane_change; // link veh_BADF_LC_Status with veh_active_lane_change

	// Time since last LC
	if (prev_data.veh_BADF_LC_Status == 0) {
		curr_data.time_since_last_LC = prev_data.time_since_last_LC + curr_data.ts_length; //[Lin] there need to be somewhere to define curr_data.time_since_last_LC = 0;
	}		

	LaneChangeDecision::MandatoryLaneChange(curr_data, prev_data); // document 1#
	LaneChangeDecision::DiscretionaryLaneChange(curr_data, prev_data); // document 2#
	
	// mandatory LC decision can override discretionary LC decision
	if (curr_data.veh_BADF_mandatory_lane_change_decision != 0) {

		curr_data.veh_BADF_lane_change_decision = curr_data.veh_BADF_mandatory_lane_change_decision;
	}
	else {
		// if there is a mandatory LC desire but there is no acceptable gap, not taking discretionary lane change decision
		if (curr_data.veh_lead_gap_reject || curr_data.veh_rear_gap_reject) {
			curr_data.veh_BADF_lane_change_decision = curr_data.veh_BADF_mandatory_lane_change_decision;
			curr_data.veh_active_gap_searching = true;
			//std::string message = "Active Gap Search in LC decision for Vehicle ID: " + std::to_string(curr_data.veh_id) + "\n";
			//OutputDebugString(message.c_str());
		}
		else {
			if (curr_data.veh_rel_target_lane == 0 && curr_data.veh_signal_distance >= 0) {
				curr_data.veh_BADF_lane_change_decision = 0;  // Block further lane changes
				//OutputDebugString("Vehicle is already in the target lane, no lane change allowed.\n");
				//std::string message = "Vehicle ID: " + std::to_string(curr_data.veh_id) + " is in target lane.\n";
				//OutputDebugString(message.c_str());
				return;  // Exit the function early to prevent further lane change logic
			}
			curr_data.veh_BADF_lane_change_decision = curr_data.veh_BADF_discretionary_lane_change_decision;
		}
	}
};

