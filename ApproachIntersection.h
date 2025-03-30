#pragma once
#include "BaselineADF.h"

struct ApproachingIntersection {
	
	static void SignalizedIntersection(car_data& curr_data, car_data& prev_data);
	static void UnsignalizedIntersection(car_data& curr_data, car_data& prev_data);
	
};