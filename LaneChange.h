#pragma once
#include "BaselineADF.h"

struct LaneChange {
	
	static void MandatoryLaneChange(car_data& curr_data, car_data& prev_data);
	static void DiscretionaryLaneChange(car_data& curr_data, car_data& prev_data);

};

