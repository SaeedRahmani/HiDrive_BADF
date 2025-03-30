#include "CarFollowing.h"
#include "Calibration.h"

void BaselineADF::CarFollowing(car_data& curr_data, car_data& prev_data) {

	// linear interpolation
	auto lerp = [](double a, double b, double t) -> double { return a + t * (b - a); };

	/* ============================= PARAMETERS ================================ */
	// default parameters for motorway
	double desired_timegap = 1.6; // desired time gap to follower the leading vehicle in second [Lin: to handle cut-in, the desired_timegap has been made as a variable "veh_desired_time_gap" that could be relaxed]
	double dx_min = 5; //standstill distance [m]
	double senserDelay = 0; // sensor delay introduced in second
	double a_max_5 = 4; // m/s², maximum acceleration at 5 m/s
	double a_max_20 = 2.0; // m/s², maximum acceleration at 20 m/s
	double a_min_5 = -6; // m/s², minimum acceleration at 5 m/s
	double a_min_20 = -5; // m/s²,  minimum acceleration at 20 m/s

	// control gains
	struct {
		double K_d = 1; // 5;
		double K_v1 = 0.5;
		double K_v2 = 0.4;
		double K_p = 0.1;
		double K_i = 1;
		double k_tg = 10; // specific for cut-in situation
	} CF_params;

	// scenario-based parameters
	if (curr_data.urban_scenario == 1) {
		dx_min = 2;
	}
	//=========================================================
	// VARIABLES

	// Variables from "sensors"

	// Current sensor values
	double v_front_veh = curr_data.lead_veh_velocity;
	double v_front_veh_pre = prev_data.lead_veh_velocity;
	//double dx_current = curr_data.veh_distance_headway - dx_min;
	//double dv_current = curr_data.lead_veh_rel_velocity;

	// Delayed sensor values
	DMFunctions::DelayDataBase(curr_data, prev_data, senserDelay); // [Lin] change to 0 to see if -3 brake still happen
	double dx_current = curr_data.veh_used_distance_headway; // [Lin] remove the -dx_min;
	double dv_current = curr_data.veh_used_rel_vel;

	// Vehicle state variables
	double v_current = curr_data.veh_velocity;
	double v_previous = prev_data.veh_velocity;
	double a_current = curr_data.veh_acceleration;
	double a_previous = prev_data.veh_acceleration;
	double thw_current = dx_current / v_current; // [Lin] remove the -dx_min;

	// Simulator variables
	double time_step = curr_data.ts_length;
	double speed_limit = curr_data.veh_BADF_setspeed;
		// using a temporary setspeed when active gap search is active
	if (curr_data.veh_active_gap_searching) {
		speed_limit = curr_data.veh_BADF_temporary_setspeed;
	}
		

	// Variable maximum acceleration
	double a_max = lerp(a_max_5, a_max_20, v_current * 3.6 / (20 - 5));
	if (a_max < a_max_20) { a_max = a_max_20; }
	if (a_max > a_max_5) { a_max = a_max_5; }

	// Variable minimum acceleration
	double a_min = lerp(a_min_5, a_min_20, v_current * 3.6 / (20 - 5));
	if (a_min < a_min_5) { a_min = a_min_5; }
	if (a_min > a_min_20) { a_min = a_min_20; }

	// lead vehicle present
	bool lead_vehicle_present = curr_data.lead_veh_type != 0;
	//bool lead_vehicle_present = thw_current < 8;

	//=========================================================
	// Lead vehicle acceleration (if used)
	/*
	double a_front_veh = 0;
	double a_front_veh_pre = prev_data.lead_veh_acceleration;

	if (lead_vehicle_present) { 
		a_front_veh = curr_data.lead_veh_acceleration;
	};
	if ((a_front_veh > 0) || (a_front_veh < -15)) {
		a_front_veh = 0;
	};

	// PT1 filter
	double K_PT1_ax = 1;
	double T_ax = 0.2;
	double a_front_veh_PT1 = T_ax * (K_PT1_ax * a_front_veh - a_front_veh_pre) + a_front_veh_pre;
	double a_front_veh_modified = 0;
	if (a_front_veh_PT1 < -1.5) {
		a_front_veh_modified = a_front_veh_PT1;
	};

	// Constant for scaling front vehicle acceleration
	double K_a = max(0.2, (0.2 - 0.7 / (2.5 - 0.9) * (thw_current - 2.5)));
	K_a = min(1, K_a);

	// Implement a "dead zone" to relative speed between -1 and 1
	// can introduce noise to the signal, if desired
	if (fabs(dv_current) > 1) {
		dv_current = 0;
	}
	*/

	// ========================================================
	// relax the desired_timegap when a cut-in is identified
	// Milanés, Vicente / Shladover, Steven E. Handling Cut - In Vehicles in Strings of Cooperative Adaptive Cruise Control Vehicles. Journal of Intelligent Transportation Systems.
	// 2015, Vol. 20, No. 2, p. 178 - 191
	// Formula tg_current(t) = tg_cutin + 1/K_tg(tg_desired - tg_cutin)*t 
	// k_tg = 10 second;

	curr_data.veh_desired_time_gap = desired_timegap;
	if (prev_data.veh_cut_in_flag==false && curr_data.veh_time_headway < 0.8 * prev_data.veh_time_headway && curr_data.veh_time_headway < 0.8 * desired_timegap && curr_data.lead_veh_id != prev_data.lead_veh_id) {
		curr_data.veh_cut_in_flag = true;
		curr_data.delta_desired_time_gap = (curr_data.veh_desired_time_gap - curr_data.veh_time_headway) * curr_data.ts_length / CF_params.k_tg;
		if (curr_data.veh_time_headway < 0) { // near vehicle inputs, unsafe lane changes by VISSIM are observed, which veh_time_headway < 0; 
			curr_data.veh_time_headway = 0; // set the time_headway to zero in such critical situation for the desired time gap to be  used later
		}
		curr_data.veh_temporary_desired_time_gap = curr_data.veh_time_headway;
		curr_data.veh_desired_time_gap = curr_data.veh_temporary_desired_time_gap;
	}
	else  // need to add else condition, otherwise the temprary time gap will be calculated twice at first swithcing moment.
	{
		if (prev_data.veh_cut_in_flag == true) { // change to prev
			curr_data.veh_temporary_desired_time_gap = prev_data.veh_temporary_desired_time_gap + prev_data.delta_desired_time_gap; //[Lin] prev temporary time gap and prev value of delta time gap (they are not calculated)
			curr_data.veh_desired_time_gap = curr_data.veh_temporary_desired_time_gap;
			curr_data.veh_cut_in_flag = prev_data.veh_cut_in_flag;
			curr_data.delta_desired_time_gap = prev_data.delta_desired_time_gap;
			// reset when the the time gap reach the original desired time gap, assumption of the switch when time gap error is smaller than 0.05 second
			if (abs(curr_data.veh_desired_time_gap - desired_timegap) <= 0.05 || curr_data.veh_time_headway >= desired_timegap) {
				curr_data.veh_desired_time_gap = desired_timegap;
				curr_data.veh_cut_in_flag = false;
				curr_data.delta_desired_time_gap = 999.0;
			}
		}
		else {
			curr_data.veh_cut_in_flag = prev_data.veh_cut_in_flag;
			curr_data.delta_desired_time_gap = prev_data.delta_desired_time_gap;
			curr_data.veh_temporary_desired_time_gap = 999.0;
		}
		
	}

	//=========================================================
	// Car following controller

	// distance error
	double dv_des = CF_params.K_d * (curr_data.veh_desired_time_gap * v_current - dx_current + dx_min);

	if (dv_des > 10) { dv_des = 10; }
	if (dv_des < -10) { dv_des = -10; }

	// car following (speed error)
	double a_des_cf = CF_params.K_v1 * (-1 * dv_current - dv_des);// +K_a * a_front_veh_modified;

	// free flow
	double a_des_free = CF_params.K_v2 * (speed_limit - v_current);
	// if speed limit control is active, ignore the free-flow acceleration
	if (curr_data.veh_BADF_SPL_control) {
		a_des_free = 0;   
	}

	double a_des = a_des_free;
	if (lead_vehicle_present) {
		a_des = min(a_des_cf, a_des_free);
	}

	if (a_des > a_max) { a_des = a_max; };
	if (a_des < a_min) { a_des = a_min; };

	// ========================================================
	// Additional constraints on maximum and minimum acceleration

	double TTC_front = 999;
	if ((dv_current > 0) && lead_vehicle_present) {
		TTC_front = dx_current / dv_current;
	}

	// maximum and minimum acceleration
	// note: v_max_d needs to be a monotonically increasing sequence
	double v_max_d[6] = { 0,10,80,129,130,999 };
	double a_max_d[6] = { 2.5,2.5,1.5,1.5,0.1,0.01 };
	double a_min_d[6] = { -6, -6, -5, -5, -5, -5 };
	int v_i = 0;

	// finds the correct indices to interpolate between
	for (int i = 0; i < 6; i++) {
		if ((v_current * 3.6) > v_max_d[i]) { 
			continue; 
		}
		else { 
			v_i = i; 
			break; }
	}
	// interpolates between the relevant values
	double a_max_new = 2.5;
	double a_min_new = -5;
	if (v_i > 0) {
		// acceleration
		if (a_max_d[v_i - 1] == a_max_d[v_i]) { // only if function is flat in this range
			a_max_new = a_max_d[v_i]; 
		}
		else {
			a_max_new = lerp(a_max_d[v_i - 1], a_max_d[v_i], (v_current * 3.6) / (v_max_d[v_i] - v_max_d[v_i - 1]));
		}
		// deceleration
		if (a_min_d[v_i - 1] == a_min_d[v_i]) { // only if function is flat in this range
			a_min_new = a_min_d[v_i];
		}
		else {
			a_min_new = lerp(a_min_d[v_i - 1], a_min_d[v_i], (v_current * 3.6) / (v_max_d[v_i] - v_max_d[v_i - 1]));
		}
		
	}
	// caps maximum acceleration to a_max_new
	if (a_des > a_max_new) { a_des = a_max_new; };

	// caps minimum acceleration to a_min_new if TTC > critical threshold, otherwise a_min_TTC_target
	double TTC_critical_AD = 3;
	double a_min_TTC_target = -7;
	if ((TTC_front < TTC_critical_AD) && (a_des < a_min_new)) {
		a_des = a_min_TTC_target;
	}
	else if (a_des < a_min_new) {
		a_des = a_min_new;
	}


	//=========================================================
	// PI CONTROLLER (if necessary)

	/*
	double PI_I_stored = prev_data.veh_PI_control_I_term;
	double PI_I_new = (a_des - a_current) * (CF_params.K_i * time_step) + PI_I_stored;
	a_des = a_des + (a_des - a_current) * CF_params.K_p + PI_I_new;
	curr_data.veh_PI_control_I_term = PI_I_new;
	*/

	//curr_data.veh_BADF_desired_acceleration_following = a_des;

	//=========================================================
	// AEB

	double t_brake_stage_1 = 0.4;
	double a_x_build_up_velocity_1 = 35;
	double a_x_build_up_velocity_2 = 35;

	// this is the AEB required acceleration
	double x_remain = 0.5;
	double a_required = -1;
	if (dv_current > -0.5) {
		double temp = -(pow(dv_current, 2)) / (2 * (dx_current - x_remain));
		a_required = min(a_previous, temp);
	}

	// current AEB TTC threshold based on speed
	double current_threshold = 0;

	// front vehicle moving
	if (v_front_veh > 1) {
		if (v_current < 10 / 3.6) {
			current_threshold = 0.85;
		}
		else if (v_current > 80 / 3.6) {
			current_threshold = 1.5;
		}
		else {
			current_threshold = lerp(0.85, 1.5, v_current * 3.6 / 80);
			if (current_threshold < 0.85) { 
				current_threshold = 0.85;
			}
		}
	} 
	else {
		if (v_current < 10 / 3.6) {
			current_threshold = 0.75;
		}
		else if (v_current > 80 / 3.6) {
			current_threshold = 1.4;
		}
		else {
			current_threshold = lerp(0.75, 1.4, v_current * 3.6 / 80);
			if (current_threshold < 0.75) {
				current_threshold = 0.75;
			}
		}
	}

	// AEB activation
	bool AEB_active = prev_data.veh_AEB_active;
	double AEB_counter = prev_data.veh_AEB_counter;

	if (TTC_front < current_threshold) {
		AEB_active = true;
		AEB_counter = AEB_counter + time_step;
	}
	else {
		if ((AEB_active == true) && (a_required > 1)) {
			AEB_active = true;
			AEB_counter = AEB_counter + time_step;
		}
		else {
			AEB_active = false;
			AEB_counter = 0;
		}
	}

	// if AEB active, choose between
	// brake stage 1: acceleration limited to -3.5
	// brake stage 2: acceleration limited to -9.81

	double a_AEB_brake = 999;
	if (AEB_active == true) {
		if (AEB_counter <= t_brake_stage_1) {
			a_AEB_brake = max(a_previous - a_x_build_up_velocity_1 * time_step, -3.5);
		}
		else {
			a_AEB_brake = max(a_previous - a_x_build_up_velocity_2 * time_step, a_required);
			a_AEB_brake = max(a_AEB_brake, -9.81);
		}
	}

	curr_data.veh_AEB_active = AEB_active;
	curr_data.veh_AEB_counter = AEB_counter;

	// select smallest of the accelerations
	double a_final = min(a_des, a_AEB_brake);
	curr_data.veh_BADF_desired_acceleration_following = a_final;

	//=========== code for breakpoint =========================
	//if (curr_data.veh_id == 117 && curr_data.timestep >= 60) {
	//		int bug = 0;
	//}
	//=========================================================

};
