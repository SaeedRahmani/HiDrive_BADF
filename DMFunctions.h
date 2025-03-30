#pragma once
#include "CarData.h"
#include "SimData.h"

struct DMFunctions {
	// database functions
	static void ResetCarData(car_data& curr_data);
	static void DelayDataBase(car_data& curr_data, car_data& prev_data, const int sensor_delay);
	
	// model functions
	static void UrbanScenarioParameters(const int scenario);
	static void AuthorityTransition(car_data& curr_data, car_data& prev_data);
	static void BaselineADF(car_data& curr_data, car_data& prev_data);
};

