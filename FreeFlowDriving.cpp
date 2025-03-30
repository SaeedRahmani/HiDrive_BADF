#include "FreeFlowDriving.h"
#include "Calibration.h"

void BaselineADF::FreeFlowDriving(car_data& curr_data, car_data& prev_data) {


	double a_des_v = kv2 * (curr_data.veh_BADF_setspeed - curr_data.veh_velocity);
	curr_data.veh_BADF_desired_acceleration_free_flow = max(min(a_des_v, a_max), a_min);
};
