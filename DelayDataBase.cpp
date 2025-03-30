#include "DelayDataBase.h"

void DMFunctions::DelayDataBase(car_data& curr_data, car_data& prev_data, const int sensor_delay) {

	curr_data.veh_past_distance_headway_1 = prev_data.veh_distance_headway;
	curr_data.veh_past_distance_headway_2 = prev_data.veh_past_distance_headway_1;
	curr_data.veh_past_distance_headway_3 = prev_data.veh_past_distance_headway_2;
	curr_data.veh_past_distance_headway_4 = prev_data.veh_past_distance_headway_3;
	curr_data.veh_past_distance_headway_5 = prev_data.veh_past_distance_headway_4;
	

	curr_data.veh_past_rel_vel_1 = prev_data.lead_veh_rel_velocity;
	curr_data.veh_past_rel_vel_2 = prev_data.veh_past_rel_vel_1;
	curr_data.veh_past_rel_vel_3 = prev_data.veh_past_rel_vel_2;
	curr_data.veh_past_rel_vel_4 = prev_data.veh_past_rel_vel_3;
	curr_data.veh_past_rel_vel_5 = prev_data.veh_past_rel_vel_4;
	

	switch (sensor_delay) {
	case 0: { curr_data.veh_used_distance_headway = curr_data.veh_distance_headway; curr_data.veh_used_rel_vel = curr_data.lead_veh_rel_velocity; break; }
	case 1: { curr_data.veh_used_distance_headway = curr_data.veh_past_distance_headway_1; curr_data.veh_used_rel_vel = curr_data.veh_past_rel_vel_1; break; }
	case 2: { curr_data.veh_used_distance_headway = curr_data.veh_past_distance_headway_2; curr_data.veh_used_rel_vel = curr_data.veh_past_rel_vel_2; break; }
	case 3: { curr_data.veh_used_distance_headway = curr_data.veh_past_distance_headway_3; curr_data.veh_used_rel_vel = curr_data.veh_past_rel_vel_3; break; }
	case 4: { curr_data.veh_used_distance_headway = curr_data.veh_past_distance_headway_4; curr_data.veh_used_rel_vel = curr_data.veh_past_rel_vel_4; break; }
	case 5: { curr_data.veh_used_distance_headway = curr_data.veh_past_distance_headway_5; curr_data.veh_used_rel_vel = curr_data.veh_past_rel_vel_5; break; }
	}
	

}
