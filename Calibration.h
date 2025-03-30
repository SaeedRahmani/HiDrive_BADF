#pragma once
	// All the parameters have been moved inside the modules for an easy parameter control basedn on scenarios
	 
	 
	///*========================= Car following =================================*/
	//const double desired_timegap = 1.6; // desired time gap to follower the leading vehicle in second [Lin: to handle cut-in, the desired_timegap has been made as a variable "veh_desired_time_gap" that could be relaxed]
	//const double a_max_5 = 4; // m/s², maximum acceleration at 5 m/s
	//const double a_max_20 = 2.0; // m/s², maximum acceleration at 20 m/s
	//const double a_min_5 = -6; // m/s², minimum acceleration at 5 m/s
	//const double a_min_20 = -5; // m/s²,  minimum acceleration at 20 m/s

	//// scenario-based parameters
	//const double dx_min = 5; //standstill distance [m]
	//const double senserDelay = 0; // sensor delay introduced in second
	//
	///*========================= speed limit control =================================*/
	//const double overtaking_speed_threshold = 60 / 3.6; // speed threshold to allow ovetakeing from right, defaut value 60 km/h [German rules]
	//const double setspeed_change_to_left_front = 0.2; // the changing rate of setspeed regarding the distance to left front leader
	//
	// scenario-based parameters
	//const double SpeedLimitControlDistance = 180.0; // distance range for speed limit control
	//
	///*========================= active gap search =================================*/
	//const double temporary_delta_setspeed = 2; // 2 m/s higher or lower than the setspeed for active gap searching

	///*========================= authority transition calibration parameters =================================*/
	//const double at_inactivatedTime = 5.0; // A cool down period to prevent automation is activated soon after a driver take-over
	//const double at_didc_reactionTime = 0.0; // driver's reaction time to driver-initiated take-over 
	//const double at_aidc_reactionTime = 1.5; // driver's reaction time to system-initiated take-over
	//const double rampTakeOverDistance = 500; // distance to the merging point at on-ramp for driver taking over control

	///*========================= lane change =================================*/
	//const double warming_up_period = 13.0; // simulation warming up period [s]

	//// mandartory LC
	////const double MLC_Distance = 500.0 ; // currently not used // a distance threshold to activate mandatory lane change [m]
	//const double min_distance_mandatory_LC = 5.0; // minimal space gap in target lane for mandatory lane change [m]
	//const double min_headway_mandatory_LC = 0.5; // minimal time gap in target lane for mandatory lane change [s] [Lin] should be 0.25 second? [Jie] L3Pilot uses 0.25, but even 0.5 might be too critical for VISSIM
	//const double min_TTC_mandatory_LC = 3; // adaption1, minimal TTC to lead vehicle in target lane for mandatory lane change [s]

	//// discretionary LC, need to check if values make sense
	//const double NoLaneChangeDistance = 150; // No discretionary lane change within this distance to the signal head or conflict area, urban scenario only
	//const double min_distance_discretionary_LC = 10.0; // minimal space gap in target lane for discretionary lane change [m]
	//const double min_headway_discretionary_LC = 1.0; // minimal time gap in target lane for discretionary lane change [s]
	//const double max_deceleration_discretionary_LC = -2.0; // maximal deceleration of rear vehicle for discretionary lane change [m/s2]
	//const double threshold_difference_to_set_speed = 10.8/3.6; // threshold of difference to BADF set speed to trigger discretionary lane change [km/h]
	//const double threshold_headway_to_lead_vehicle = 4.0;  // threshold of headway to leading vehicle to trigger discretionary lane change [s]
	//const double threshold_headway_to_adjacent_lead_vehicle = 6.0;  // threshold of headway to adjacent leading vehicle to trigger discretionary lane change [s]
	//const double threshold_distance_to_adjacent_lead_vehicle = 150.0;  // threshold of distance to adjacent leading vehicle to trigger discretionary lane change [s]
	//const double threshold_speed_congestion = 60.0 / 3.6;  // threshold speed for congestion situation [m/s]: 60 km/h for motorway, value for urban need to be checked
	//const double threshold_time_since_last_LC = 15.0; // threshold of time since the last LC to initiate a new LC [s]
	//const double threshold_current_braking = -1.5; // threshold of braking rate to cancel a discretionary LC[m/s2]
	
