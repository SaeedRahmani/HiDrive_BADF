#include "LaneChange.h"
#include "Calibration.h"

void BaselineADF::LaneChange(car_data& curr_data, car_data& prev_data) {

	LaneChange::MandatoryLaneChange(curr_data, prev_data);
	LaneChange::DiscretionaryLaneChange(curr_data, prev_data);
	// how these two lane changes are integrated is not modelled yet. The integration should be specified here.
};