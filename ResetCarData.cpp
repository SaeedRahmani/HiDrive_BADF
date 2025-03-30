#include "ResetCarData.h"
#include <math.h>
#include <limits>
#ifdef NAN
/* NAN is supported*/
#endif // NAN

// To make this model up to date, copy all non-database values from CarData.h including their default values
void DMFunctions::ResetCarData(car_data& curr_data) {
	/*---simdata--- */
	curr_data.ts_length = 0.0;
	curr_data.urban_scenario = 0;

	/*-.-.-.-.-.-.-.-.-.-.-own vehicle-.-.-.-.-.-.-.-.-.-.-.*/
	//======================= veh specification ======================
	curr_data.veh_color = RGB(0, 0, 0);
	curr_data.veh_type = 0; // vehicle type number (user defined)
	curr_data.veh_category = 0; // car = 1, truck = 2, bus = 3, tram = 4, pedestrian = 5, bike =6
	curr_data.veh_length = 0.0; // [m] vehicle length

	//======================= location=======================
	curr_data.veh_current_lane = 0; // current lane number (rightmost = 1)
	curr_data.veh_current_link = 0; // current link number
	curr_data.veh_n_lanes = 0; // number of lanes in current link

	// ===================== traffic signal =====================
	curr_data.veh_signal_distance = 999.0; // -1 if not visable; otherwise distnce to signal in [m]
	curr_data.veh_signal_state = 0; //red = 1, amber = 2, green = 3, red/amber = 4, amber flashing = 5, off = 6, green arrow = 7  */

	//======================= conflict area =======================
	curr_data.veh_confl_areas_count = 0;
	curr_data.veh_confl_area_distance = 999.0;

	//======================= speed limit =======================
	curr_data.veh_speed_limit_distance = 999.0; // distance [m] to 'speed limit sign'; (reduced speed area: real distance; desired speed decision: 1.0 m when just passed; negative: no sign visible)
	curr_data.veh_speed_limit_value = 999.0; //speed limit [km/h] (0 = end of reduced speed area)

	// ==============longitudinal============================
	curr_data.veh_distance_headway = 999.0; // distance gap [m] with a leading vehicle
	curr_data.veh_time_headway = 999.0;
	curr_data.veh_velocity = 999.0; //current speed [m/s] 
	curr_data.veh_acceleration = 0; // current acceleration [m/s^2]
	curr_data.veh_desired_velocity = 999.0; //desired speed [m/s]
	curr_data.veh_max_acceleration = 999.0; // [m/s^2] maximum possible acceleration based on current speed
	curr_data.veh_desired_acceleration = 999.0; //desired acceleration [m/s²] in next time step
	curr_data.veh_desired_time_gap = 999.0;
	//==============lateral=========================================
	curr_data.veh_desired_lane_angle = 0.0; //desired angle relative to the middle of the lane [rad] (positive = turning left)
	curr_data.veh_active_lane_change = 0; // direction of an active lane change movement(+1 = to the left, 0 = none, -1 = to the right)
	curr_data.veh_desired_active_lane_change = 0; // direction of an active lane change movement [TNO NOTE: currently willing to do a lane change, adviced by VISSIM]
	curr_data.veh_rel_target_lane = 0; //target lange (+1 = next one left, 0 = current lane, -1 = next one right)
	curr_data.veh_turning_indicator = 0; // left = 1, right = -1, none = 0, both = 2;
	curr_data.veh_preferred_rel_lane = 0; // [TNO NOTE:direction of desired mandatory lane change] positive = left, 0 = current lane, negative = right;
	curr_data.veh_use_preferred_lane = 0; // 0 = only preferable (e.g. European highway) 1 = necessary (e.g. before a connector) [TNO NOTE: mandatory lane change = 1, discretionary lane change = 0;]
	/*
	curr_data.NoLaneChangeRight = false; // true, if lane change is not allowed.
	curr_data.NoLaneChangeLeft = false; // true, if lane change is not allowed.
	*/
	//==============   BADF ===========================
	// speed limit control
	curr_data.veh_BADF_setspeed = 999.0; // set speed of BADF
	curr_data.veh_BADF_SPL_control = false;
	curr_data.veh_BADF_desired_acceleration_speed_limit = std::numeric_limits<double>::quiet_NaN();
	curr_data.veh_BADF_SPL_a_required_set = 999.0;
	curr_data.minimum_distance_to_left_front = 3.5;
	// signalized intersection
	curr_data.veh_BADF_desired_aceleration_signalized_intersection = 999.0;
	// unsignalized intersection
	curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = 999.0;

	//****** lane change ******************************
	curr_data.veh_BADF_mandatory_lane_change_decision = 0;
	curr_data.veh_BADF_discretionary_lane_change_decision = 0;
	curr_data.veh_BADF_lane_change_decision = 0;
	curr_data.veh_BADF_LC_abortion_decision = 0;

	curr_data.veh_BADF_LC_Status = 0;
	curr_data.veh_BADF_LC_abortion_Status = 0;

	curr_data.in_mandatory_LC_area = false;
	curr_data.time_since_last_LC = 0.0;

	curr_data.visualize_LC_decision = 0;

	curr_data.veh_lead_gap_reject = false;
	curr_data.veh_rear_gap_reject = false;
	curr_data.veh_active_gap_searching = false;
	
	// adaptions for lane change
	curr_data.activate_adaption1_mLC_TTC = false;

	//**************************************************

	// free flow
	curr_data.veh_BADF_desired_acceleration_free_flow = 999.0; // desired acceleration from cruising controller in free flow
	// car following
	curr_data.veh_BADF_desired_acceleration_following = 999.0;
	curr_data.veh_PI_control_I_term = 1.0; // used by the PI controller for smoothing out acceleration response

	curr_data.veh_AEB_active = false; // state of AEB
	curr_data.veh_AEB_counter = 0; // used by the AEB logic to count the length of braking
	curr_data.veh_BADF_driving_mode = 0;

	/* ==================  Authority Transition status ====================== */
	curr_data.veh_automation_state = HumanTakeOver; // 1 is human, 2 is critical, 3 is automated system

	/* ================== Authority Transition variables ====================== */
	curr_data.at_decision = noTransition; // deactivation decision before constraints
	curr_data.at_reactionTime = -99.0; // Reaction time after a DIDC/AIDC


	/* ==================  Authority Transition status ====================== */
	curr_data.veh_automation_state = HumanTakeOver; // 1 is human, 2 is critical, 3 is automated system
	

	/* ================== Authority Transition variables ====================== */
	curr_data.at_decision = noTransition; // deactivation decision before constraints
	curr_data.at_reactionTime = -99.0; // Reaction time after a DIDC/AIDC
		/* ================== Active Gap Search ====================== */
	curr_data.veh_BADF_temporary_setspeed = -999.0;
	/* -.-.-.-.-.-.-.-.-.-.-lead vehicle-.-.-.-.-.-.-.-.-.-.-*/
	curr_data.lead_veh_distance = 999.0;
	curr_data.lead_veh_rel_velocity = 999.0;
	curr_data.lead_veh_velocity = 999.0;
	curr_data.lead_veh_acceleration = 0.0;
	curr_data.lead_veh_length = 0.0;
	curr_data.lead_veh_id = 0;
	curr_data.lead_veh_type = 0;

	/* -.-.-.-.-.-.-.-.-.-.-left lead vehicle-.-.-.-.-.-.-.-.-.-.-*/
	curr_data.left_lead_veh_id = 0;
	curr_data.left_lead_veh_length = 0.0;
	curr_data.left_lead_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	curr_data.left_lead_veh_distance = 999.0;
	curr_data.left_lead_veh_rel_velocity = 999.0;
	curr_data.left_lead_veh_headway = 999.0;

	curr_data.left_lead_veh_TTC = 999.0; // adaption1, TTC [s]
	
	/* -.-.-.-.-.-.-.-.-.-.-left rear vehicle-.-.-.-.-.-.-.-.-.-.-*/
	curr_data.left_rear_veh_id = 0;
	curr_data.left_rear_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	curr_data.left_rear_veh_distance = 999.0;
	curr_data.left_rear_veh_rel_velocity = 999.0;
	curr_data.left_rear_veh_headway = 999.0;

	curr_data.left_rear_veh_TTC = 999.0; // adaption1, TTC [s]

	/* -.-.-.-.-.-.-.-.-.-.-right lead vehicle-.-.-.-.-.-.-.-.-.-.-*/
	curr_data.right_lead_veh_id = 0;
	curr_data.right_lead_veh_length = 0.0;
	curr_data.right_lead_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	curr_data.right_lead_veh_distance = 999.0;
	curr_data.right_lead_veh_rel_velocity = 999.0;
	curr_data.right_lead_veh_headway = 999.0; // adaption1, TTC [s]

	curr_data.right_lead_veh_TTC = 999.0;
	/* -.-.-.-.-.-.-.-.-.-.-right rear vehicle-.-.-.-.-.-.-.-.-.-.-*/
	curr_data.right_rear_veh_id = 0;
	curr_data.right_rear_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	curr_data.right_rear_veh_distance = 999.0;
	curr_data.right_rear_veh_rel_velocity = 999.0;
	curr_data.right_rear_veh_headway = 999.0;
	
	curr_data.right_rear_veh_TTC = 999.0; // adaption1, TTC [s]


	/* -.-.-.-.-.-.-.-.-.-.-Next conflict area (to be added in the header file)-.-.-.-.-.-.-.-.-.-.-*/

	for (int i = 0; i < 20; i++) {
		curr_data.conf_area_dist[i]=999.0;
		curr_data.conf_area_length[i] = 999.0;
		curr_data.conf_area_type[i] = 999;
		curr_data.conf_area_yield[i] = 999;
		curr_data.conf_area_number[i] = 999;
		curr_data.time_enter_0[i] = 999.0;
		curr_data.time_enter_1[i] = 999.0;
		curr_data.time_exit_0[i] = 999.0;
		curr_data.time_exit_1[i] = 999.0;
		curr_data.time_vehicle_pass[i] = 999.0;
		curr_data.has_waiting_priority[i] = 999; /*(1=NO, 0=YES)*/
		curr_data.waiting_time_others[i] = 0.0;
		curr_data.time_enter_unique[i]=999;

	}


	curr_data.time_vehicle_enter = 999.0;
	curr_data.time_vehicle_stop = 999.0;
	curr_data.decided_to_yield = 999;
	curr_data.target_acceleration = 999.0;
	curr_data.conf_first = 999;
	curr_data.waiting_time = 0.0;
	curr_data.accel_temp = 0.0;
	
	
};


