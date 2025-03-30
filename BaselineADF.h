#pragma once
#include "DMFunctions.h"

struct BaselineADF {

	static void FreeFlowDriving(car_data& curr_data, car_data& prev_data);
	static void CarFollowing(car_data& curr_data, car_data& prev_data);
	static void LaneChangeDecision(car_data& curr_data, car_data& prev_data);

	//static void LaneChangeAbortion(car_data& curr_data, car_data& prev_data);


	static void ApproachingSpeedLimit(car_data& curr_data, car_data& prev_data);
	static void ApproachingIntersection(car_data& curr_data, car_data& prev_data);
	static void LateralControl(car_data& curr_data, car_data& prev_data);
	static void ActiveGapSearch(car_data& curr_data, car_data& prev_data);

};
