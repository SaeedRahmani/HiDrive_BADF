#include "ActiveGapSearch.h"
#include "Calibration.h"

void BaselineADF::ActiveGapSearch(car_data& curr_data, car_data& prev_data) {
	//Three behaviour is defined for a BADF vehicle to search for another gap when the current gap for a mandatory lane change is unacceptable. 
	//Behaviour 1: when only the rear gap is rejected and no front vehicle or current time gap is larger than twice of desired time gap, vehicle setspeed will be 2m/s higher (temporary_delta_setspeed)
	//Behaviour 2: when only the front gap is rejected, vehicle setspeed will be 2 m/s lower (temporary_delta_setspeed);
	//Behaviour 3: when both front and rear gap are rejected, driver take over and perform the mandatory lane change in the next step
	
	/* ============================= PARAMETERS ================================ */
	// default parameters for motorway
	double temporary_delta_setspeed = 2; // 2 m/s higher or lower than the setspeed for active gap searching

	// scenario-based parameters
	if (curr_data.urban_scenario == 1) {
		
	}
	
	/* ============================= adjust setspeed ================================ */
	if (!curr_data.veh_lead_gap_reject && curr_data.veh_rear_gap_reject && (curr_data.right_lead_veh_id <0 || curr_data.veh_time_headway > 2*curr_data.veh_desired_time_gap)) {
		curr_data.veh_BADF_temporary_setspeed = curr_data.veh_BADF_setspeed + temporary_delta_setspeed;
	}
	else
	{
		if (curr_data.veh_lead_gap_reject && !curr_data.veh_rear_gap_reject) {
			curr_data.veh_BADF_temporary_setspeed = curr_data.veh_BADF_setspeed - temporary_delta_setspeed;
		}
		else
		{
			curr_data.veh_toc_gap_search = true;
		}
	}
}
