// #include "ApproachIntersection.h"
// #include "Calibration.h"


// void BaselineADF::ApproachingIntersection (car_data& curr_data, car_data& prev_data) {
		
// 	ApproachingIntersection::SignalizedIntersection(curr_data, prev_data);  
// 	ApproachingIntersection::UnsignalizedIntersection(curr_data, prev_data);

// 	if (curr_data.veh_signal_distance > 0 && curr_data.veh_confl_areas_count > 0) { // if both signalized and unsignalized intersection are detected, check the distance to determine the dominate acceleration and reset the other acceleration.
// 		if (curr_data.veh_signal_distance < curr_data.veh_confl_area_distance) {
// 			curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = 999.0;
// 		}
// 		else
// 		{
// 			curr_data.veh_BADF_desired_aceleration_signalized_intersection = 999.0;
// 		}
// 	}
// };

//### NEW CONDTIONS BY SAEED, TO BE CHECKED BY LIN ###//


#include "ApproachIntersection.h"
#include "Calibration.h"

void BaselineADF::ApproachingIntersection(car_data& curr_data, car_data& prev_data) {

	ApproachingIntersection::SignalizedIntersection(curr_data, prev_data);
	ApproachingIntersection::UnsignalizedIntersection(curr_data, prev_data);
	if (curr_data.veh_signal_distance >= 0) {
		curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = 999.0;
	}
	else if (curr_data.veh_confl_areas_count > 0) {
		curr_data.veh_BADF_desired_aceleration_signalized_intersection = 999.0;
	}
};
