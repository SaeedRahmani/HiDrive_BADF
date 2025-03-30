#pragma once
#ifndef _CONSOLE
#include <windows.h>
#include <math.h>
#include <string>
#include "simData.h"
#ifdef NAN
/* NAN is supported*/
#endif // NAN
#endif

using namespace std;


struct car_data {
	/*WARNING adding new (non-database) variables should also be add to the reset_car_data function!*/
	/*---simdata--- */
	double timestep = 0.0;
	double ts_length = 0.0;
	int urban_scenario = 0;

	/*-.-.-.-.-.-.-.-.-.-.-own vehicle-.-.-.-.-.-.-.-.-.-.-.*/
	//=======================veh specification======================
	int	   veh_id = 0;
	int    veh_color = RGB(0, 0, 0);
	int    veh_type = 0; // vehicle type number (user defined)
	int    veh_category = 0; // car = 1, truck = 2, bus = 3, tram = 4, pedestrian = 5, bike =6
	double  veh_length = 0.0; // [m] vehicle length

	//======================= location=======================
	int     veh_current_lane = 0;  //current lane number (rightmost = 1)
	int     veh_current_link = 0;  //current link number
	int		veh_n_lanes = 0; // number of lanes in current link


	//======================= traffic signal =======================
	double	veh_signal_distance = 999.0; // -1 if not visable; otherwise distnce to signal in [m]
	int		veh_signal_state = 0; //red = 1, amber = 2, green = 3, red/amber = 4, amber flashing = 5, off = 6, green arrow = 7  */
	
	//======================= conflict area =======================
	int		veh_confl_areas_count = 0;
	double	veh_confl_area_distance = 999.0;
								  
	//======================= speed limit =======================
	double	veh_speed_limit_distance = -999.0; // distance [m] to 'speed limit sign'; (reduced speed area: real distance; desired speed decision: 1.0 m when just passed; negative: no sign visible)
	double	veh_speed_limit_value = -999.0; //speed limit [km/h] (0 = end of reduced speed area)

	//======================= lane end =======================
	double veh_lane_end_distance = 999.0; //- 1 if not visable; otherwise distnce to lane end in[m]

	// ==============longitudinal============================
	double	veh_distance_headway = 999.0; // distance gap [m] with a leading vehicle, net distance
	double	veh_time_headway = 999.0; // time gap [s] with a leading vehicle, net headway
	double	veh_velocity = 999.0;  //current speed [m/s] 
	double  veh_acceleration = 0; // current acceleration [m/s^2]
	double  veh_desired_velocity = 999.0; //desired speed [m/s]
	double  veh_max_acceleration = 999.0; // [m/s^2] maximum possible acceleration based on current speed
	double  veh_desired_acceleration = 999.0; //desired acceleration [m/s²] in next time step 
	double  veh_desired_time_gap = 999.0; // [s] desired time gap of the ADF
	double  veh_temporary_desired_time_gap = 999.0; // [s] temporary desired time gap of the ADF
	bool	veh_cut_in_flag = false; // Not in the reset function
	double	delta_desired_time_gap = 999.0; // the change of desired time gap during the handeling cut-in


	//==============lateral=========================================
	double  veh_desired_lane_angle = 0.0; //desired angle relative to the middle of the lane [rad] (positive = turning left)
	int		veh_active_lane_change = 0; // direction of an active lane change movement (+1 = to the left, 0 = none, -1 = to the right)
	int		veh_desired_active_lane_change = 0; // direction of an active lane change movement [TNO NOTE: currently willing to do a lane change, adviced by VISSIM]
	int		veh_rel_target_lane = 0; //target lange (+1 = next one left, 0 = current lane, -1 = next one right)
	int		veh_turning_indicator = 0; // left = 1, right = -1, none = 0, both = 2;
	int		veh_preferred_rel_lane = 0; // [TNO NOTE:direction of desired mandatory lane change] positive = left, 0 = current lane, negative = right;
	int		veh_use_preferred_lane = 0; // 0 = only preferable (e.g. European highway) 1 = necessary (e.g. before a connector) [TNO NOTE: mandatory lane change = 1, discretionary lane change = 0;]

	//==============   BADF ===========================
	// speed limit control
	double veh_current_speed_limit = -999; // by default, it is the driver's desired speed;changed by the speed limit signs.
	double veh_BADF_setspeed = 999.0; // set speed of BADF
	bool veh_BADF_SPL_control = false;
	double veh_BADF_desired_acceleration_speed_limit = NAN;
	double veh_BADF_SPL_a_required_set = 999.0;
	double minimum_distance_to_left_front = 3.5; // the distance to left front when the exact left front speed is considered setspeed

	// signalized intersection
	double	veh_BADF_desired_aceleration_signalized_intersection = 999.0;
	// unsignalized intersection
	double	veh_BADF_desired_aceleration_unsignalized_intersection = 999.0;
	
	//****** lane change ******************************
	int		veh_BADF_mandatory_lane_change_decision = 0; //mandatory lane change decision (1 = left, 0 = no LC, -1 = right)
	int		veh_BADF_discretionary_lane_change_decision = 0; // discretionary lane change decision (1 = left, 0 = no LC, -1 = right)
	int		veh_BADF_lane_change_decision = 0; // lane change decision (1 = left, 0 = no LC, -1 = right)
	int		veh_BADF_LC_abortion_decision = 0; // if vehicle is aborting a LC (0 = not abort, 1 = abort)

	int		veh_BADF_LC_Status = 0; // if vehicle is performing a LC (1 = left, 0 = not in LC, -1 = right)
	int		veh_BADF_LC_abortion_Status = 0; // if vehicle is aborting a LC (0 = not abort, 1 = abort)

	bool	in_mandatory_LC_area = false; // if vehicle is in the area of mandatory LC (e.g., in acceleration lane for on-ramp merge)
	double	time_since_last_LC = 0.0; // counting time elasped since the last LC [s]
	/*
	int		NoLaneChangeRight = false; // accessibility to the right lane(lane change is not allow --> true)
	int		NoLaneChangeLeft = false; // accessibility to the left lane(lane change is not allow --> true)
	*/
	int visualize_LC_decision = 0; // 1 = mandatory LC decision, -1 = discretionary LC decision, 0 = no LC
	bool veh_lead_gap_reject = false;
	bool veh_rear_gap_reject = false;

	bool veh_active_gap_searching = false;

	// adaptions for lane change
	bool	activate_adaption1_mLC_TTC = false; // if to active adaption1

	//**************************************************
	
	// free flow
	double	veh_BADF_desired_acceleration_free_flow = 999.0; // desired acceleration from cruising controller in free flow
	// car following

	double veh_BADF_desired_acceleration_following = 999.0;
	double  veh_PI_control_I_term = 1.0; // used by the PI controller for smoothing out acceleration response

	bool	veh_AEB_active = false; // state of AEB
	double	veh_AEB_counter = 0; // used by the AEB logic to count the length of braking
	double	veh_BADF_driving_mode = 0; 

	/* ==================  Authority Transition status ====================== */
	int	veh_automation_state = HumanTakeOver; // 1 is human, 2 is critical, 3 is automated system
	double	veh_reactionTimer = -99.0; // A timer to record the driver's reaction time
	double  veh_inactivatedTimer = -99.0; // A timer to record the inactivated time period

	/* ================== Authority Transition variables ====================== */
	int at_decision = noTransition; // deactivation decision before constraints
	bool at_wantSetAutoOff = false; // An indicator to show if a driver want to switch off the system control
	bool at_canSetAutoOn = true; //  An indicator to show if a driver can switch on the system control
	double at_reactionTime = -99.0; // Reaction time after a DIDC/AIDC
	bool veh_toc_gap_search = false; // a TOC request when both lane change gaps are rejected for a mandatory lane change 

	/* ================== Active Gap Search ====================== */
	double veh_BADF_temporary_setspeed = -999.0;
	/* -.-.-.-.-.-.-.-.-.-.-lead vehicle-.-.-.-.-.-.-.-.-.-.-*/
	double  lead_veh_distance = 999.0; // gross distance [m] (front end to front end)
	double  lead_veh_rel_velocity = 999.0; //speed difference [m/s] (veh. speed - nveh. speed)
	double	lead_veh_velocity = 999.0;
	double  lead_veh_acceleration = 0.0;
	double  lead_veh_length = 0.0;
	int		lead_veh_id = 0;
	int		lead_veh_type = 0;


	/* -.-.-.-.-.-.-.-.-.-.-left lead vehicle-.-.-.-.-.-.-.-.-.-.-*/
	int		left_lead_veh_id = 0;
	double	left_lead_veh_length = 0.0;
	double	left_lead_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	double	left_lead_veh_distance = 999.0;
	double  left_lead_veh_rel_velocity = 999.0; //speed difference [m/s] (veh. speed - nveh. speed)
	double	left_lead_veh_headway = 999.0;

	double	left_lead_veh_TTC = 999.0; // adaption1, TTC [s]
	
	/* -.-.-.-.-.-.-.-.-.-.-left rear vehicle-.-.-.-.-.-.-.-.-.-.-*/
	int		left_rear_veh_id = 0;
	double	left_rear_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	double	left_rear_veh_distance = 999.0;
	double  left_rear_veh_rel_velocity = 999.0; //speed difference [m/s] (veh. speed - nveh. speed)
	double	left_rear_veh_headway = 999.0;

	double	left_rear_veh_TTC = 999.0; // adaption1, TTC [s]

	/* -.-.-.-.-.-.-.-.-.-.-right lead vehicle-.-.-.-.-.-.-.-.-.-.-*/
	int		right_lead_veh_id = 0;
	double	right_lead_veh_length = 0.0;
	double	right_lead_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	double	right_lead_veh_distance = 999.0;
	double  right_lead_veh_rel_velocity = 999.0; //speed difference [m/s] (veh. speed - nveh. speed)
	double	right_lead_veh_headway = 999.0;

	double	right_lead_veh_TTC = 999.0; // adaption1, TTC [s]

	/* -.-.-.-.-.-.-.-.-.-.-right rear vehicle-.-.-.-.-.-.-.-.-.-.-*/
	int		right_rear_veh_id = 0;
	double	right_rear_veh_distance_gross = 999.0; // gross distance [m] (front end to front end)
	double	right_rear_veh_distance = 999.0;
	double  right_rear_veh_rel_velocity = 999.0; //speed difference [m/s] (veh. speed - nveh. speed)
	double	right_rear_veh_headway = 999.0;
	
	double	right_rear_veh_TTC = 999.0; // adaption1, TTC [s]


	/*-------------------------------Databases---------------------------*/
	// WARNING: Adding a database variable should also come with a method to save this data in DelayDataBase.cpp
	double	veh_used_rel_vel = 999.0;
	double	veh_past_rel_vel_1 = 999.0;
	double	veh_past_rel_vel_2 = 999.0;
	double	veh_past_rel_vel_3 = 999.0;
	double	veh_past_rel_vel_4 = 999.0;
	double	veh_past_rel_vel_5 = 999.0;

	double  veh_used_distance_headway = 999.0;
	double	veh_past_distance_headway_1 = 999.0;
	double	veh_past_distance_headway_2 = 999.0;
	double	veh_past_distance_headway_3 = 999.0;
	double	veh_past_distance_headway_4 = 999.0;
	double	veh_past_distance_headway_5 = 999.0;

	char static_route[100];

	
	/* -.-.-.-.-.-.-.-.-.-.-Next conflict area (to be added in the header file)-.-.-.-.-.-.-.-.-.-.-*/ 
	int conf_num = 0;

	double conf_area_dist[20] = { 999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0 };
	double conf_area_length[20] = { 999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0 };
	int conf_area_type[20] = { 999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999};
	int conf_area_yield[20] = { 999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999 };
	int conf_area_number[20] = { 999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999 };

	double time_enter_0[20] = { 999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0 };
	double  time_enter_1[20] = { 999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0 };
	double  time_exit_0[20] = { 999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0 };
	double  time_exit_1[20] = { 999.0,999.0,999.0,999.0,999.0,999.0,999.0 ,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0 ,999.0,999.0,999.0 };
	double  time_vehicle_pass[20] = { 999.0,999.0,999.0,999.0,999.0,999.0,999.0 ,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0,999.0 ,999.0,999.0,999.0 };
	int time_enter_unique[20] = { 999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999 };
	
	double time_vehicle_enter = 999.0;
	double time_vehicle_stop = 999.0;
	double target_acceleration = 999.0;
	double accel_temp = 0.0;
	
	

	int decided_to_yield = 999; /*(1=yield, 2=pass after having yielded, 3 adjust speed and pass after first conflicting vehicle)*/
	int has_waiting_priority[20] = { 999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999,999 }; /*(1=NO, 0=YES)*/
	double waiting_time = 0.0;
	double waiting_time_others[20] = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

	
	int next_link = 999;
	int conf_first = 999;
	int sum_yields = 999;
	
};