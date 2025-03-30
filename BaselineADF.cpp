#include "BaselineADF.h"
#include "SimData.h"
#include "Calibration.h"
#include "math.h"
#include <cstdlib>
#include <vector>
#include <algorithm>


void DMFunctions::BaselineADF(car_data& curr_data, car_data& prev_data) {

	// Tactical Controller
	// Please specify your module output, e.g. desired speed
	if (curr_data.veh_id == 46 && curr_data.timestep >= 175) {
		int bug = 0;
	}

	BaselineADF::ApproachingSpeedLimit(curr_data, prev_data);
	BaselineADF::LaneChangeDecision(curr_data, prev_data); 
	//BaselineADF::LaneChangeAbortion(curr_data, prev_data);
	if (curr_data.veh_active_gap_searching) {
		BaselineADF::ActiveGapSearch(curr_data, prev_data);
	}

	BaselineADF::ApproachingIntersection(curr_data, prev_data);
	BaselineADF::CarFollowing(curr_data, prev_data);
	BaselineADF::LateralControl(curr_data, prev_data);

	
	//curr_data.veh_desired_active_lane_change = curr_data.veh_preferred_rel_lane;
	
	if (isnan(curr_data.veh_BADF_desired_acceleration_speed_limit)) {
		curr_data.veh_BADF_desired_acceleration_speed_limit = 999;
	}
	// integration of desired acceleration from different controllers
	std::vector<double> acceleration_list{curr_data.veh_BADF_desired_acceleration_free_flow,curr_data.veh_BADF_desired_acceleration_speed_limit, curr_data.veh_BADF_desired_aceleration_signalized_intersection,curr_data.veh_BADF_desired_aceleration_unsignalized_intersection,curr_data.veh_BADF_desired_acceleration_following};
	curr_data.veh_desired_acceleration = *std::min_element(acceleration_list.begin(), acceleration_list.end());


	// !!!!!![LIN] this give unsignalized intersection acceleration the top priority, which is incorrect. Needs to be removed.
if ((curr_data.veh_BADF_desired_aceleration_unsignalized_intersection != 999.0) && (curr_data.veh_BADF_desired_aceleration_unsignalized_intersection > 0.0))
{
	curr_data.veh_desired_acceleration = curr_data.veh_BADF_desired_aceleration_unsignalized_intersection;
}

if ((curr_data.veh_BADF_desired_aceleration_unsignalized_intersection <0.0) && (curr_data.veh_BADF_desired_acceleration_speed_limit < 0.0))
{
	curr_data.veh_desired_acceleration = min( curr_data.veh_BADF_desired_aceleration_unsignalized_intersection, curr_data.veh_BADF_desired_acceleration_speed_limit);
}


	if (curr_data.veh_id == 118 && curr_data.timestep >= 205) {
		int bug = 0;
	}

}