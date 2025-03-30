#include "AuthorityTransition.h"
#include "Calibration.h"



void DMFunctions::AuthorityTransition(car_data& curr_data, car_data& prev_data) {

	/* This Authority transition module determines when the ADF will be activated/deactivated. In Hi-Drive, it is assumed that driver will
	activate ADF as much as possible and deactivate the ADF system only when it is out of the operational design demain.*/

	/* ============================= PARAMETERS ================================ */
	// default parameters for motorway
	double at_inactivatedTime = 5.0; // A cool down period to prevent automation is activated soon after a driver take-over
	double at_didc_reactionTime = 0.0; // driver's reaction time to driver-initiated take-over 
	double at_aidc_reactionTime = 1.5; // driver's reaction time to system-initiated take-over
	double rampTakeOverDistance = 500; // distance to the merging point at on-ramp for driver taking over control (Motorway only)

	// scenario-based parameters
	if (curr_data.urban_scenario == 1) {
		
	}

	/* ============================= Initialization ================================ */
	/* Driver-initiated deactivation (outside GEO fence)*/
	bool didc = false;
	/* System-initiated deactivation (when the BADF outputted acceleration beyond ODD)*/
	bool aidc = false;

	/* ============================= TOC conditions ================================ */
	// when the vehicle is heading an on/off-ramp, driver will take over control
	// this is identified by the combination of route information and lane end distance
	// it requires specific network settings:
	// - the route of which driver need to take over should be defined as X-1
	// - the look ahead distance in "driving behaviour" need to be increased to 600 m, vehicle will start reaction at 500 m
	// - add the "static_route" as an UDA with "No.2" in the network file. 
	if (curr_data.veh_lane_end_distance != -1 && curr_data.veh_lane_end_distance <= rampTakeOverDistance && atoi(&curr_data.static_route[2]) == 1) {  // this is the condition to check if heading to route X - 1
		didc = true;
	}

	/* a TOC request when both lane change gaps are rejected for a mandatory lane change */
	// first reset veh_toc_gap_search when there is no need for mandaotry lane change
	if (curr_data.veh_use_preferred_lane == 0) {
		curr_data.veh_toc_gap_search = false;
	}
	if (curr_data.veh_toc_gap_search) {
		aidc = true;
	}

	/* outside operational design domain (exceeding automation acceleration capability) */
	/*  When desired acceleration is smaller than - 3.5 m / s ^ 2, there is a warning and driver takes over. */
	/*if (prev_data.veh_BADF_desired_acceleration_following <= ODD_MAX_BRAKE) {
		aidc = true;
	}*/

	/* overall decision */
	if (didc) {
		curr_data.at_decision = DIDC;
	}
	else
	{
		if (aidc) {
			curr_data.at_decision = AIDC;
		}
		else
		{
			curr_data.at_decision = noTransition;
		}
	}

	/* ============================= Constraints ================================ */
	/* deactivation with delay */
	// Constraints - Automated control loop, a reaction time after AIDC
	if (prev_data.veh_automation_state == AutomatedSystem)
	{
		// when there is no event in the previous time step
		if (curr_data.veh_reactionTimer < 0)
		{
			// when there is a deactivation request
			if (curr_data.at_decision != noTransition)
			{
				// initial deactivation settings
				curr_data.at_wantSetAutoOff = true;
				curr_data.at_canSetAutoOn = false;
				curr_data.veh_reactionTimer = 0;

				// curr_data.veh_inactivatedTimer = 0;
				 // different reaction time for different cases
				switch (curr_data.at_decision)
				{
				case DIDC:
					curr_data.at_reactionTime = at_didc_reactionTime;
					break;
				case AIDC:
					curr_data.at_reactionTime = at_aidc_reactionTime;
					break;
				default:
					// it is better to write a warning message.
					break;
				}
			}
			else // continues with no event, use the previous control
			{
				curr_data.at_wantSetAutoOff = prev_data.at_wantSetAutoOff;
				curr_data.at_canSetAutoOn = prev_data.at_canSetAutoOn;
			}
		}
		else
		{
			// When there is an activated reaction timer
			curr_data.veh_reactionTimer = curr_data.veh_reactionTimer + curr_data.ts_length;
			curr_data.at_wantSetAutoOff = prev_data.at_wantSetAutoOff;
			curr_data.at_canSetAutoOn = prev_data.at_canSetAutoOn;
		}

		if (curr_data.at_reactionTime >= 0 && curr_data.at_wantSetAutoOff && curr_data.veh_reactionTimer >= curr_data.at_reactionTime)
		{
			curr_data.veh_automation_state = HumanTakeOver;
			curr_data.veh_reactionTimer = -99.0;
			curr_data.veh_inactivatedTimer = 0;
		}
		else
		{
			curr_data.veh_automation_state = prev_data.veh_automation_state;
		}

	}

	/* remain human driving before re-activation */
	else {
		// Constraints - Manual control loop, minimum inactive period after takeovers & BADF will not be activated when active lane change is on. 
		if (prev_data.veh_automation_state == HumanTakeOver)
		{
			// If the Timer is activated
			if (prev_data.veh_inactivatedTimer >= 0)
			{
				// Reset timer if encounter DIDC and AIDC, otherwise time add up
				if (curr_data.at_decision == DIDC || curr_data.at_decision == AIDC)
				{
					curr_data.veh_inactivatedTimer = 0;
					curr_data.at_wantSetAutoOff = true;
					curr_data.at_canSetAutoOn = false;
				}
				else
				{
					// When there is no more deactivation request
					curr_data.veh_inactivatedTimer = prev_data.veh_inactivatedTimer + curr_data.ts_length;
					curr_data.at_wantSetAutoOff = false;
					curr_data.at_canSetAutoOn = false;
				}
			}
			else  // when no Timer is activated
			{
				// set the Timer = 0 when encounter a DIDC or AIDC 
				if (curr_data.at_decision == DIDC || curr_data.at_decision == AIDC)
				{
					curr_data.veh_inactivatedTimer = 0;
					curr_data.at_wantSetAutoOff = true;
					curr_data.at_canSetAutoOn = false;
				}
				else
				{
					curr_data.at_wantSetAutoOff = prev_data.at_wantSetAutoOff;
					curr_data.at_canSetAutoOn = prev_data.at_canSetAutoOn;
				}
			}

			// Disable timer at the end of inactivated period
			if (curr_data.veh_inactivatedTimer > at_inactivatedTime)
			{
				curr_data.at_canSetAutoOn = true;
				curr_data.veh_inactivatedTimer = -99.0;
			}


			if (curr_data.at_canSetAutoOn && !curr_data.at_wantSetAutoOff) {
				curr_data.veh_automation_state = AutomatedSystem;
			}
			else
			{
				curr_data.veh_automation_state = prev_data.veh_automation_state;
			}

		}

	}


};