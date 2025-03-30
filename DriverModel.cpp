/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that does nothing (uses Vissim's internal model).         */
/*                                                                          */
/*  Version of 2021-02-24                                   Lukas Kautzsch  */
/*  Data struction added by TNO 2023-10-03                  Lin Xiao        */
/*==========================================================================*/


/*
if ((*current_timestep_map)[vehicle_id()].timestep >= 140.0 && (*current_timestep_map)[vehicle_id()].veh_id == 10) {
    int debug = 1;
}

if (curr_data.timestep >= 93.3 && curr_data.veh_id == 30) {
    int debug = 1;
}
*/

#include <math.h>
#include <map>
#include <thread>
#include <mutex>
#include "DriverModel.h"

/*==================== Definition of data structures ===========================*/
#include "CarData.h"
#include "SimData.h"

/*=========================== MODEL functions =================================*/
#include "DMFunctions.h" // this contains paths to all functions

/*=================== Global running variables ==============================*/
double timestep = 0.0; // used to define current timestep
double ts_length = 0.0; // s

/*=================== Define the data map ==============================*/

// This data structure contains the data of all vehicles that are called within the dll.
// Data should only be accesed via this struct and not by using globals.

typedef std::map<long, car_data> car_data_map;

car_data_map timestep_1;
car_data_map timestep_2;
car_data_map* current_timestep_map = &timestep_1;
car_data_map* previous_timestep_map = &timestep_2;

/* =====================Enable multireading ==========================================*/

// this variable is lateron used to lock a certain part of the code,
// This is required so that permutions that should only occur once per timestep
// cannot be accesed by multi threads at the same time.
std::mutex mut;

// The thread_vehicle_map and the vehicle_id() function enable the code
// to call the thread local vehicle_id, which is the vehicle id for which
// the functions are currently called.
std::map<std::thread::id, long> thread_vehicle_map;
long vehicle_id() // This Function calls the vehicle_id used in the current thread
{
    return thread_vehicle_map[std::this_thread::get_id()]; // current vehicle
}

/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule,
                       DWORD   ul_reason_for_call,
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/
/* VISSIM --> DLL */
DRIVERMODEL_API  int  DriverModelSetValue (int    type,
                                           int    index1,
                                           int    index2,
                                           int    int_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <int_value>, <double_value> or             */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_TIMESTEP               :
        ts_length = double_value;
        return 1;
    case DRIVER_DATA_TIME                   :
        {// if this is the first thread, reaching this pioint in a new timestep:
          // create a previous timestep map from the current.
        std::lock_guard<std::mutex> lock(mut);
        if (timestep != double_value) { // if a new timestep swap pointers

            car_data_map* tmpmap = current_timestep_map;
            current_timestep_map = previous_timestep_map;
            previous_timestep_map = tmpmap;
            timestep = double_value;
        }
        }
        return 1;
    case DRIVER_DATA_USE_UDA                :
      return 1; /* doesn't use any UDAs */
                /* must return 1 for desired values of index1 if UDA values are to be sent from/to Vissim */
    case DRIVER_DATA_VEH_ID                 :
        // This is the first function called which makes it possible for a vehicle to save 
        // data on the correct position. 
        // for each vehicle the vehicle_id() function is activated,
        // simulation data about this timestep is saved and currently active
        // databases are copied

        // the vehicle_id() function is initialised and 
        thread_vehicle_map[std::this_thread::get_id()] = int_value;

        //sim data is saved
        (*current_timestep_map)[vehicle_id()].timestep = timestep;
        (*current_timestep_map)[vehicle_id()].veh_id = vehicle_id();

        /*=================RESET all near vehicle data =========================*/
        // since the data is based on a swap of the maps, it is possble that no 
        // near vehicle is present, in such a case data should not linger in the system

        DMFunctions::ResetCarData((*current_timestep_map)[vehicle_id()]);

        // Record the authority transition states
        (*current_timestep_map)[vehicle_id()].at_wantSetAutoOff = (*previous_timestep_map)[vehicle_id()].at_wantSetAutoOff;
        (*current_timestep_map)[vehicle_id()].at_canSetAutoOn = (*previous_timestep_map)[vehicle_id()].at_canSetAutoOn;
        (*current_timestep_map)[vehicle_id()].at_reactionTime = (*previous_timestep_map)[vehicle_id()].at_reactionTime;
        (*current_timestep_map)[vehicle_id()].veh_reactionTimer = (*previous_timestep_map)[vehicle_id()].veh_reactionTimer;
        (*current_timestep_map)[vehicle_id()].veh_toc_gap_search = (*previous_timestep_map)[vehicle_id()].veh_toc_gap_search;

        // Record the simulation time interval
        (*current_timestep_map)[vehicle_id()].ts_length = ts_length;

        return 1;
    case DRIVER_DATA_VEH_LANE               :
        (*current_timestep_map)[vehicle_id()].veh_current_lane = int_value;
        return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
        return 1;
    case DRIVER_DATA_VEH_VELOCITY           :
        (*current_timestep_map)[vehicle_id()].veh_velocity = double_value;
        return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
        (*current_timestep_map)[vehicle_id()].veh_acceleration = double_value;
        return 1;
    case DRIVER_DATA_VEH_LENGTH             :
        (*current_timestep_map)[vehicle_id()].veh_length = double_value;
        return 1;
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
        return 1;
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
        (*current_timestep_map)[vehicle_id()].veh_max_acceleration = double_value;
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
        (*current_timestep_map)[vehicle_id()].veh_turning_indicator = int_value;
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
        (*current_timestep_map)[vehicle_id()].veh_category = int_value;
        return 1;
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
        (*current_timestep_map)[vehicle_id()].veh_preferred_rel_lane = int_value;
        return 1;
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
        (*current_timestep_map)[vehicle_id()].veh_use_preferred_lane = int_value;
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
        (*current_timestep_map)[vehicle_id()].veh_desired_velocity = double_value;
        return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Z_COORDINATE  :
        return 1;
    case DRIVER_DATA_VEH_TYPE               :
        (*current_timestep_map)[vehicle_id()].veh_type = int_value;
        return 1;
    case DRIVER_DATA_VEH_COLOR              :
        (*current_timestep_map)[vehicle_id()].veh_color = int_value;
        return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
        (*current_timestep_map)[vehicle_id()].veh_current_link = int_value;
        return 1; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                  /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
		(*current_timestep_map)[vehicle_id()].next_link = int_value;
        return 1;
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
        (*current_timestep_map)[vehicle_id()].veh_active_lane_change = int_value;
        return 1;
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
    case DRIVER_DATA_VEH_INTAC_STATE        :
    case DRIVER_DATA_VEH_INTAC_TARGET_TYPE  :
    case DRIVER_DATA_VEH_INTAC_TARGET_ID    :
    case DRIVER_DATA_VEH_INTAC_HEADWAY      :
	    return 1;
    case DRIVER_DATA_VEH_UDA:
        // the routing info as UDA is only applicable for Highway scenario
        if (index1 == 100) {
            (*current_timestep_map)[vehicle_id()].urban_scenario = int_value;
        }
        
        if (index1 == 200) {
             strcpy((*current_timestep_map)[vehicle_id()].static_route, string_value);
             int stringLength = strlen(string_value);
             (*current_timestep_map)[vehicle_id()].static_route[stringLength] = '\0';
        }
        /*
        if (index1 == 201) {
            (*current_timestep_map)[vehicle_id()].NoLaneChangeRight = int_value;
        }

        if (index1 == 202) {
            (*current_timestep_map)[vehicle_id()].NoLaneChangeLeft = int_value;
        }
        */
        return 1;
    case DRIVER_DATA_NVEH_ID                :
        if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
            (*current_timestep_map)[vehicle_id()].lead_veh_id = int_value;
        }

        if (index1 == 1 && index2 == 1) { /* left lead vehicle */
            (*current_timestep_map)[vehicle_id()].left_lead_veh_id = int_value; 
        }
        if (index1 == 1 && index2 == -1) { /* left rear vehicle */
            (*current_timestep_map)[vehicle_id()].left_rear_veh_id = int_value;
        }

        if (index1 == -1 && index2 == 1) { /* right lead vehicle */
            (*current_timestep_map)[vehicle_id()].right_lead_veh_id = int_value;
        }

        if (index1 == -1 && index2 == -1) { /* right rear vehicle */
            (*current_timestep_map)[vehicle_id()].right_rear_veh_id = int_value;
        }
        return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
        return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
        if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
            (*current_timestep_map)[vehicle_id()].lead_veh_distance = double_value;
        }

        if (index1 == 1 && index2 == 1) { /* left lead vehicle */
            (*current_timestep_map)[vehicle_id()].left_lead_veh_distance_gross = double_value;
        }

        if (index1 == 1 && index2 == -1) { /* left rear vehicle */
            (*current_timestep_map)[vehicle_id()].left_rear_veh_distance_gross = double_value;
        }

        if (index1 == -1 && index2 == 1) { /* right lead vehicle */
            (*current_timestep_map)[vehicle_id()].right_lead_veh_distance_gross = double_value;
        }

        if (index1 == -1 && index2 == -1) { /* right rear vehicle */
            (*current_timestep_map)[vehicle_id()].right_rear_veh_distance_gross = double_value;
        }
        return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
        if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
            (*current_timestep_map)[vehicle_id()].lead_veh_rel_velocity = double_value; /* veh.speed - nveh.speed */
        }
        if (index1 == 1 && index2 == 1) { /* left lead vehicle */
            (*current_timestep_map)[vehicle_id()].left_lead_veh_rel_velocity = double_value; /* veh.speed - nveh.speed */
        }
        if (index1 == 1 && index2 == -1) { /* left rear vehicle */
            (*current_timestep_map)[vehicle_id()].left_rear_veh_rel_velocity = double_value; /* veh.speed - nveh.speed */
        }
        if (index1 == -1 && index2 == 1) { /* right lead vehicle */
            (*current_timestep_map)[vehicle_id()].right_lead_veh_rel_velocity = double_value; /* veh.speed - nveh.speed */
        }
        if (index1 == -1 && index2 == -1) { /* right rear vehicle */
            (*current_timestep_map)[vehicle_id()].right_rear_veh_rel_velocity = double_value; /* veh.speed - nveh.speed */
        }
        return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
        if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
            (*current_timestep_map)[vehicle_id()].lead_veh_acceleration = double_value;
        }
        return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
        if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
            (*current_timestep_map)[vehicle_id()].lead_veh_length = double_value;
        }
        if (index1 == 1 && index2 == 1) { /* left lead vehicle */
            (*current_timestep_map)[vehicle_id()].left_lead_veh_length = double_value;
        }
        if (index1 == -1 && index2 == 1) { /* right lead vehicle */
            (*current_timestep_map)[vehicle_id()].right_lead_veh_length = double_value;
        }
        return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
        return 1;
    case DRIVER_DATA_NVEH_TYPE              :
        if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
            (*current_timestep_map)[vehicle_id()].lead_veh_type = int_value;
        }
        return 1;
    case DRIVER_DATA_NVEH_UDA               :
        /*if (index1 == 1 && index2 == 1) {
            
        }
        return 1;*/
    case DRIVER_DATA_NVEH_X_COORDINATE      :
    case DRIVER_DATA_NVEH_Y_COORDINATE      :
    case DRIVER_DATA_NVEH_Z_COORDINATE      :
    case DRIVER_DATA_NVEH_REAR_X_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Y_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Z_COORDINATE :
        return 1;
    case DRIVER_DATA_NO_OF_LANES            :
        (*current_timestep_map)[vehicle_id()].veh_n_lanes = int_value;
        return 1;
    case DRIVER_DATA_LANE_WIDTH             :
        return 1;
    case DRIVER_DATA_LANE_END_DISTANCE      :
        if (index1 == 1) {
            (*current_timestep_map)[vehicle_id()].veh_lane_end_distance = double_value;
        }
    case DRIVER_DATA_CURRENT_LANE_POLY_N    :
    case DRIVER_DATA_CURRENT_LANE_POLY_X    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Y    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Z    :
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
        return 1;
    case DRIVER_DATA_SIGNAL_DISTANCE        :
        (*current_timestep_map)[vehicle_id()].veh_signal_distance = double_value;
        return 1;
    case DRIVER_DATA_SIGNAL_STATE           :
        (*current_timestep_map)[vehicle_id()].veh_signal_state = int_value;
        return 1;
    case DRIVER_DATA_SIGNAL_STATE_START     :
        return 1;
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
        (*current_timestep_map)[vehicle_id()].veh_speed_limit_distance = double_value;
        return 1;
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
        (*current_timestep_map)[vehicle_id()].veh_speed_limit_value = double_value;
        return 1;
    case DRIVER_DATA_PRIO_RULE_DISTANCE     :
    case DRIVER_DATA_PRIO_RULE_STATE        :
    case DRIVER_DATA_ROUTE_SIGNAL_DISTANCE  :
    case DRIVER_DATA_ROUTE_SIGNAL_STATE     :
    case DRIVER_DATA_ROUTE_SIGNAL_CYCLE     :
        return 1;
    case DRIVER_DATA_CONFL_AREAS_COUNT      :
        (*current_timestep_map)[vehicle_id()].conf_num = int_value;
		return 1;  /* (to avoid getting sent lots of conflict area data _ to be changed) */
    case DRIVER_DATA_CONFL_AREA_TYPE        :
        /*for (index1= 0; index1 < (*current_timestep_map)[vehicle_id()].conf_num; ++index1)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[index1] = int_value;
        }*/

        if (index1 == 0)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[0] = int_value;
        }
        if (index1 == 1)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[1] = int_value;
        }
        if (index1 == 2)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[2] = int_value;
        }
        if (index1 == 3)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[3] = int_value;
        }
        if (index1 == 4)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[4] = int_value;
        }
        if (index1 == 5)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[5] = int_value;
        }
        if (index1 == 6)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[6] = int_value;
        }
        if (index1 == 7)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[7] = int_value;
        }
        if (index1 == 8)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[8] = int_value;
        }
        if (index1 == 9)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[9] = int_value;
        }
        if (index1 == 10)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[10] = int_value;
        }
        if (index1 == 11)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[11] = int_value;
        }
        if (index1 == 12)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[12] = int_value;
        }
        if (index1 == 13)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[13] = int_value;
        }
        if (index1 == 14)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[14] = int_value;
        }
        if (index1 == 15)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[15] = int_value;
        }
        if (index1 == 16)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[16] = int_value;
        }
        if (index1 == 17)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[17] = int_value;
        }
        if (index1 == 18)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[18] = int_value;
        }
        if (index1 == 19)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[19] = int_value;
        }
        if (index1 == 20)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_type[20] = int_value;
        }
        

        return 1;
    case DRIVER_DATA_CONFL_AREA_YIELD       :

        if (index1 == 0)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[0] = int_value;
        }
        if (index1 == 1)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[1] = int_value;
        }
        if (index1 == 2)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[2] = int_value;
        }
        if (index1 == 3)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[3] = int_value;
        }
        if (index1 == 4)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[4] = int_value;
        }
        if (index1 == 5)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[5] = int_value;
        }
        if (index1 == 6)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[6] = int_value;
        }
        if (index1 == 7)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[7] = int_value;
        }
        if (index1 == 8)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[8] = int_value;
        }
        if (index1 == 9)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[9] = int_value;
        }
        if (index1 == 10)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[10] = int_value;
        }
        if (index1 == 11)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[11] = int_value;
        }
        if (index1 == 12)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[12] = int_value;
        }
        if (index1 == 13)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[13] = int_value;
        }
        if (index1 == 14)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[14] = int_value;
        }
        if (index1 == 15)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[15] = int_value;
        }
        if (index1 == 16)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[16] = int_value;
        }
        if (index1 == 17)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[17] = int_value;
        }
        if (index1 == 18)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[18] = int_value;
        }
        if (index1 == 19)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[19] = int_value;
        }
        if (index1 == 20)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_yield[20] = int_value;
        }
        return 1;
        
    case DRIVER_DATA_CONFL_AREA_DISTANCE    :
        /* if (index1 == 0)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist = double_value;
            
        }*/
        if (index1==0)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[0] = double_value;
        }
        if (index1 == 1)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[1] = double_value;
        }
        if (index1 == 2)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[2] = double_value;
        }
        if (index1 == 3)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[3] = double_value;
        }
        if (index1 == 4)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[4] = double_value;
        }
        if (index1 == 5)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[5] = double_value;
        }
        if (index1 == 6)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[6] = double_value;
        }
        if (index1 == 7)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[7] = double_value;
        }
        if (index1 == 8)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[8] = double_value;
        }
        if (index1 == 9)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[9] = double_value;
        }
        if (index1 == 10)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[10] = double_value;
        }
        if (index1 == 11)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[11] = double_value;
        }
        if (index1 == 12)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[12] = double_value;
        }
        if (index1 == 13)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[13] = double_value;
        }
        if (index1 == 14)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[14] = double_value;
        }
        if (index1 == 15)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[15] = double_value;
        }
        if (index1 == 16)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[16] = double_value;
        }
        if (index1 == 17)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[17] = double_value;
        }
        if (index1 == 18)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[18] = double_value;
        }
        if (index1 == 19)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[19] = double_value;
        }
        if (index1 == 20)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[20] = double_value;
        }
        /*for (index1 = 0; index1 < (*current_timestep_map)[vehicle_id()].conf_num; index1++) {
            (*current_timestep_map)[vehicle_id()].conf_area_dist[index1] = double_value;
        }*/

        return 1; /* (to be inserted) */

    case DRIVER_DATA_CONFL_AREA_LENGTH      :
        if (index1 == 0)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[0] = double_value;
        }
        if (index1 == 1)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[1] = double_value;
        }
        if (index1 == 2)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[2] = double_value;
        }
        if (index1 == 3)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[3] = double_value;
        }
        if (index1 == 4)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[4] = double_value;
        }
        if (index1 == 5)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[5] = double_value;
        }
        if (index1 == 6)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[6] = double_value;
        }
        if (index1 == 7)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[7] = double_value;
        }
        if (index1 == 8)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[8] = double_value;
        }
        if (index1 == 9)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[9] = double_value;
        }
        if (index1 == 10)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[10] = double_value;
        }
        if (index1 == 11)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[11] = double_value;
        }
        if (index1 == 12)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[12] = double_value;
        }
        if (index1 == 13)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[13] = double_value;
        }
        if (index1 == 14)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[14] = double_value;
        }
        if (index1 == 15)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[15] = double_value;
        }
        if (index1 == 16)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[16] = double_value;
        }
        if (index1 == 17)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[17] = double_value;
        }
        if (index1 == 18)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[18] = double_value;
        }
        if (index1 == 19)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[19] = double_value;
        }
        if (index1 == 20)
        {
            (*current_timestep_map)[vehicle_id()].conf_area_length[20] = double_value;
        }
        return 1; /* (to be inserted) */

    case DRIVER_DATA_CONFL_AREA_VEHICLES    :
    case DRIVER_DATA_CONFL_AREA_TIME_ENTER  :
        if (index1==0)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 1)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 2)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 3)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 4)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 5)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 6)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 7)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 8)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[8] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 9)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[9] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 10)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[10] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 11)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[11] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 12)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[12] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 13)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[13] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 14)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[14] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 15)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[15] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 16)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[16] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 17)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[17] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 18)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[18] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 19)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[19] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }

        if (index1 == 20)
        {
            if (index2 == 0)  /* index1= number conflict lane, INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_enter_unique[20] = int_value;
                (*current_timestep_map)[vehicle_id()].time_enter_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_enter_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        
        return 1; /* (to be inserted) */

    case DRIVER_DATA_CONFL_AREA_TIME_IN     :
    case DRIVER_DATA_CONFL_AREA_TIME_EXIT   :
        if (index1 == 0)
        {
            if (index2 == 0)  /* INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_exit_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_exit_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 1)
        {
            if (index2 == 0)  /* INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_exit_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_exit_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 2)
        {
            if (index2 == 0)  /* INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_exit_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_exit_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 3)
        {
            if (index2 == 0)  /* INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_exit_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_exit_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        if (index1 == 4)
        {
            if (index2 == 0)  /* INDEX2 = number of conflicting vehicle either in or will arrive(to be inserted) */
            {
                (*current_timestep_map)[vehicle_id()].time_exit_0[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
            if (index2 == 1)
            {
                (*current_timestep_map)[vehicle_id()].time_exit_1[index1] = double_value - (*current_timestep_map)[vehicle_id()].timestep;
            }
        }
        
        
        
        return 1; /* (to be inserted) */

     
    case DRIVER_DATA_DESIRED_ACCELERATION   :
        (*current_timestep_map)[vehicle_id()].veh_desired_acceleration = double_value;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE     :
        (*current_timestep_map)[vehicle_id()].veh_desired_lane_angle = double_value;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE     :
        (*current_timestep_map)[vehicle_id()].veh_desired_active_lane_change = int_value;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE        :
        (*current_timestep_map)[vehicle_id()].veh_rel_target_lane = int_value;
      return 1;
    default :
      return 0;
  }
}

/*--------------------------------------------------------------------------*/
/* VISSIM --> DLL */
DRIVERMODEL_API  int  DriverModelSetValue3 (int    type,
                                            int    index1,
                                            int    index2,
                                            int    index3,
                                            int    int_value,
                                            double double_value,
                                            char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1>, */
  /* <index2> and <index3>, to <int_value>, <double_value> or              */
  /* <*string_value> (object and value selection depending on <type>).     */
  /* Return value is 1 on success, otherwise 0.                            */
  /* DriverModelGetValue (DRIVER_DATA_MAX_NUM_INDICES, ...) needs to set   */
  /* *int_value to 3 or greater in order to activate this function!        */

  switch (type) {
    case DRIVER_DATA_ROUTE_SIGNAL_SWITCH:
      return 0; /* don't send any more switch values */
    default:
      return 0;
  }
}

/*--------------------------------------------------------------------------*/
/* DLL --> VISSIM */
DRIVERMODEL_API  int  DriverModelGetValue (int    type,
                                           int    index1,
                                           int    index2,
                                           int    *int_value,
                                           double *double_value,
                                           char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*int_value>,        */
  /* <*double_value> or <**string_value> (object and value selection      */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

    switch (type) {
    case DRIVER_DATA_STATUS:
        *int_value = 0;
        return 1;
    case DRIVER_DATA_WANTS_ALL_SIGNALS:
        *int_value = 1; /* needs to be set to zero if no global signal data is required */
        return 1;
    case DRIVER_DATA_MAX_NUM_INDICES:
		*int_value = 3; /* because DriverModelSetValue3() and DriverModelSetValue3() exist in this DLL */
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :

        // first function called in the Get function, therefore it is used to
        // do calculation when all data is retrieved to enrich the data [needs to be confirmed by PTV]

        /*==================== Calculate information from the recieved data =================*/
        // convert rel vel --> vel
        (*current_timestep_map)[vehicle_id()].lead_veh_velocity = (*current_timestep_map)[vehicle_id()].veh_velocity - (*current_timestep_map)[vehicle_id()].lead_veh_rel_velocity;

        // convert front to front distance to front to rear distance
        (*current_timestep_map)[vehicle_id()].veh_distance_headway = (*current_timestep_map)[vehicle_id()].lead_veh_distance - (*current_timestep_map)[vehicle_id()].lead_veh_length;

        // convert distance gap to time gap to the front leader
        (*current_timestep_map)[vehicle_id()].veh_time_headway = (*current_timestep_map)[vehicle_id()].veh_distance_headway / (*current_timestep_map)[vehicle_id()].veh_velocity;

        // compute distance gap and time gap to surrounding vehicles
        if ((*current_timestep_map)[vehicle_id()].left_lead_veh_id > 0)	// left lead
        {
            (*current_timestep_map)[vehicle_id()].left_lead_veh_distance = std::abs((*current_timestep_map)[vehicle_id()].left_lead_veh_distance_gross) - (*current_timestep_map)[vehicle_id()].left_lead_veh_length;
            (*current_timestep_map)[vehicle_id()].left_lead_veh_headway = (*current_timestep_map)[vehicle_id()].left_lead_veh_distance / (*current_timestep_map)[vehicle_id()].veh_velocity; // it is possible that time gap is negative due to negative distance gap
            if ((*current_timestep_map)[vehicle_id()].left_lead_veh_rel_velocity > 0 && (*current_timestep_map)[vehicle_id()].left_lead_veh_distance > 0) {
                (*current_timestep_map)[vehicle_id()].left_lead_veh_TTC = (*current_timestep_map)[vehicle_id()].left_lead_veh_distance / (*current_timestep_map)[vehicle_id()].left_lead_veh_rel_velocity;
            }
        }

        if ((*current_timestep_map)[vehicle_id()].left_rear_veh_id > 0)	// left rear
        {
            (*current_timestep_map)[vehicle_id()].left_rear_veh_distance = std::abs((*current_timestep_map)[vehicle_id()].left_rear_veh_distance_gross) - (*current_timestep_map)[vehicle_id()].veh_length;
            (*current_timestep_map)[vehicle_id()].left_rear_veh_headway = (*current_timestep_map)[vehicle_id()].left_rear_veh_distance / ((*current_timestep_map)[vehicle_id()].veh_velocity - (*current_timestep_map)[vehicle_id()].left_rear_veh_rel_velocity);
            if ((*current_timestep_map)[vehicle_id()].left_rear_veh_rel_velocity < 0) {
                (*current_timestep_map)[vehicle_id()].left_rear_veh_TTC = (*current_timestep_map)[vehicle_id()].left_rear_veh_distance / std::abs((*current_timestep_map)[vehicle_id()].left_rear_veh_rel_velocity);
            }
        }
        if ((*current_timestep_map)[vehicle_id()].right_lead_veh_id > 0)	// right lead
        {
            (*current_timestep_map)[vehicle_id()].right_lead_veh_distance = std::abs((*current_timestep_map)[vehicle_id()].right_lead_veh_distance_gross) - (*current_timestep_map)[vehicle_id()].right_lead_veh_length;
            (*current_timestep_map)[vehicle_id()].right_lead_veh_headway = (*current_timestep_map)[vehicle_id()].right_lead_veh_distance / (*current_timestep_map)[vehicle_id()].veh_velocity;
            if ((*current_timestep_map)[vehicle_id()].right_lead_veh_rel_velocity > 0) {
                (*current_timestep_map)[vehicle_id()].right_lead_veh_TTC = (*current_timestep_map)[vehicle_id()].right_lead_veh_distance / (*current_timestep_map)[vehicle_id()].right_lead_veh_rel_velocity;
            }
        }

        if ((*current_timestep_map)[vehicle_id()].right_rear_veh_id > 0)	// right rear
        {
            (*current_timestep_map)[vehicle_id()].right_rear_veh_distance = std::abs((*current_timestep_map)[vehicle_id()].right_rear_veh_distance_gross) - (*current_timestep_map)[vehicle_id()].veh_length;
            (*current_timestep_map)[vehicle_id()].right_rear_veh_headway = (*current_timestep_map)[vehicle_id()].right_rear_veh_distance / ((*current_timestep_map)[vehicle_id()].veh_velocity - (*current_timestep_map)[vehicle_id()].right_rear_veh_rel_velocity);
            if ((*current_timestep_map)[vehicle_id()].right_rear_veh_rel_velocity < 0) {
                (*current_timestep_map)[vehicle_id()].right_rear_veh_TTC = (*current_timestep_map)[vehicle_id()].right_rear_veh_distance / std::abs((*current_timestep_map)[vehicle_id()].right_rear_veh_rel_velocity);
            }
        }

        /* read scenario and get the model parameters */
        //DMFunctions::UrbanScenarioParameters((*current_timestep_map)[vehicle_id()].simulation_scenario);
        /*======================= Here the data flow starts ==================================*/
        // code for setting a breakpoint to check with a vehicle ID and timestep
        /*if ((*current_timestep_map)[vehicle_id()].timestep >= 140.0 && (*current_timestep_map)[vehicle_id()].veh_id == 10) {
            int debug = 1;
        }*/

        if (1 == 1) {
            DMFunctions::AuthorityTransition((*current_timestep_map)[vehicle_id()], (*previous_timestep_map)[vehicle_id()]);

            switch ((*current_timestep_map)[vehicle_id()].veh_automation_state) {

            case HumanTakeOver: {
                (*current_timestep_map)[vehicle_id()].veh_desired_acceleration = (*current_timestep_map)[vehicle_id()].veh_desired_acceleration;
                (*current_timestep_map)[vehicle_id()].veh_desired_active_lane_change = (*current_timestep_map)[vehicle_id()].veh_desired_active_lane_change;

                break;
            }
            case AutomatedSystem: {
                DMFunctions::BaselineADF((*current_timestep_map)[vehicle_id()], (*previous_timestep_map)[vehicle_id()]);
                (*current_timestep_map)[vehicle_id()].veh_desired_acceleration = (*current_timestep_map)[vehicle_id()].veh_desired_acceleration; // shall be improved by different names

                break;
            }
            }
            if ((*current_timestep_map)[vehicle_id()].veh_automation_state == AutomatedSystem) {

            } // When it is switched to human, some variable should remember the previous value, please check later!
        }


        /*======================= Here the functions starts ==================================*/


        /*==== Turning indicator function ====*/
        *int_value = (*current_timestep_map)[vehicle_id()].veh_turning_indicator;
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY:
        *double_value = (*current_timestep_map)[vehicle_id()].veh_desired_velocity;


        return 1;
    case DRIVER_DATA_VEH_COLOR:


        (*current_timestep_map)[vehicle_id()].veh_color = -255; // BADF Yellow

        if ((*current_timestep_map)[vehicle_id()].veh_automation_state == HumanTakeOver) {
            (*current_timestep_map)[vehicle_id()].veh_color = -218877; // orange
        }

        /* if ((*current_timestep_map)[vehicle_id()].visualize_LC_decision == 1)
        {
            (*current_timestep_map)[vehicle_id()].veh_color = -26215; // Red to visulize mandatory LC decision;

        if ((*current_timestep_map)[vehicle_id()].veh_automation_state == HumanTakeOver) {
            (*current_timestep_map)[vehicle_id()].veh_color = -255; //BADF HUMAN Yellow
        }
        else
        {

                (*current_timestep_map)[vehicle_id()].veh_color = -14799175; // BADF AUTOMATION Blue


        }
        if ((*current_timestep_map)[vehicle_id()].visualize_LC_decision == -1)
        {
            (*current_timestep_map)[vehicle_id()].veh_color = -9384735; // Blue to visulize discretionary LC decision;
        }
        */
		
		// color for unsignlized intersection 
		/*
        if ((*current_timestep_map)[vehicle_id()].veh_type == 700) {
            (*current_timestep_map)[vehicle_id()].veh_color = -255; //BADF Yellow
        }
        else
        {
            if ((*current_timestep_map)[vehicle_id()].veh_type == 800) {
                (*current_timestep_map)[vehicle_id()].veh_color = -14799175; // EADF Blue
            }
            else
            {
                (*current_timestep_map)[vehicle_id()].veh_color = -26215; // otherwise Red for warning
            }

        
        
            if ((*current_timestep_map)[vehicle_id()].veh_type == 100) {
                (*current_timestep_map)[vehicle_id()].veh_color = -6381661; // manual driven car
            
            }
        }
        */


        * int_value = (*current_timestep_map)[vehicle_id()].veh_color;

        /* Color example
            *long_value = -255; // yellow
            *long_value = -218877; // orange
            *long_value = -26215; // light red
            *long_value = -6684673;  // light blue-green
            *long_value = -9384735; // Light BLue
            *long_value = -14799175; //Dark BLue
            *long_value = -16020992; // Dark Green
            *long_value = -11814656; // Light Green
            *long_value = -6381661; // Gray
         */
        return 1;
    case DRIVER_DATA_VEH_UDA:

  
        if (1 == 1) // only set this false when it is applied to the unsignalized intersection
        {
            if (index1 == 100) {
                *int_value = (*current_timestep_map)[vehicle_id()].urban_scenario;
            }
            /* 
            if (index1 == 101) {
                *int_value = (*current_timestep_map)[vehicle_id()].veh_BADF_mandatory_lane_change_decision;
            }

            if (index1 == 102) {
                *int_value = (*current_timestep_map)[vehicle_id()].veh_BADF_discretionary_lane_change_decision;
            }

            if (index1 == 103) {
                *int_value = (*current_timestep_map)[vehicle_id()].veh_BADF_lane_change_decision;
            }

            if (index1 == 104) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_setspeed;
            }

            if (index1 == 105) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_desired_acceleration;
            }

            if (index1 == 106) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_acceleration_following;
            }
            
            if (index1 == 107) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_acceleration_speed_limit;
            }
            if (index1 == 108) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_aceleration_signalized_intersection;
            }
            if (index1 == 109) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_aceleration_unsignalized_intersection;
            }
            if (index1 == 110) {
                *int_value = (*current_timestep_map)[vehicle_id()].veh_BADF_SPL_control;
            }*/
            
        }
        else
        {
            if (index1 == 1)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_dist[0];

            }

            if (index1 == 2)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_dist[1];

            }

            if (index1 == 3)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[0];

            }

            if (index1 == 4)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[1];

            }

            if (index1 == 5)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[0];

            }

            if (index1 == 6)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[1];

            }

            if (index1 == 7)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[2];

            }

            if (index1 == 8)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[3];

            }

            if (index1 == 9)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].veh_velocity;

            }

            if (index1 == 10)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].veh_desired_acceleration;

            }

            if (index1 == 11)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[0];

            }

            if (index1 == 12)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[1];

            }

            if (index1 == 13)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].decided_to_yield;

            }

            if (index1 == 14)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].has_waiting_priority[0];

            }

            if (index1 == 15)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].lead_veh_distance;

            }

            if (index1 == 16)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_num;

            }

            if (index1 == 17)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[2];

            }

            if (index1 == 18)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[3];

            }

            if (index1 == 19)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_dist[2];

            }

            if (index1 == 20)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_dist[3];

            }

            if (index1 == 21)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_length[0];

            }

            if (index1 == 22)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_length[1];

            }

            if (index1 == 23)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_length[2];

            }

            if (index1 == 24)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].conf_area_length[3];

            }

            if (index1 == 25)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[0];

            }

            if (index1 == 26)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[1];

            }

            if (index1 == 27)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[2];

            }

            if (index1 == 28)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[3];

            }

            if (index1 == 29)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[2];

            }

            if (index1 == 30)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[3];

            }

            if (index1 == 31)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].timestep + (*current_timestep_map)[vehicle_id()].time_enter_0[0];

            }

            if (index1 == 32)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_exit_0[0];

            }

            if (index1 == 33)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_exit_0[1];

            }

            if (index1 == 34)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_exit_0[2];

            }

            if (index1 == 35)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_exit_0[3];

            }

            if (index1 == 36)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[4];

            }

            if (index1 == 37)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[4];

            }

            if (index1 == 38)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[4];

            }

            if (index1 == 39)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[4];

            }

            if (index1 == 40)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_aceleration_unsignalized_intersection;

            }

            if (index1 == 41)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[5];

            }

            if (index1 == 42)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[5];

            }

            if (index1 == 43)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[5];

            }

            if (index1 == 44)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[6];

            }

            if (index1 == 45)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[5];

            }

            if (index1 == 46)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[6];

            }

            if (index1 == 47)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[6];

            }

            if (index1 == 48)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_0[7];

            }

            if (index1 == 49)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_enter_1[2];

            }

            if (index1 == 50)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_first;

            }

            if (index1 == 51)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[6];

            }

            if (index1 == 52)

            {

                *double_value = (*current_timestep_map)[vehicle_id()].time_vehicle_pass[7];

            }

            if (index1 == 53)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_yield[7];

            }

            if (index1 == 54)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].conf_area_type[7];

            }

            if (index1 == 55)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[0];

            }

            if (index1 == 56)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[1];

            }

            if (index1 == 57)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[2];

            }

            if (index1 == 58)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[3];

            }

            if (index1 == 59)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[4];

            }

            if (index1 == 60)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[5];

            }

            if (index1 == 61)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[6];

            }

            if (index1 == 62)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].time_enter_unique[7];

            }

            if (index1 == 63)

            {

                *int_value = (*current_timestep_map)[vehicle_id()].veh_id;

            }

            if (index1 == 106) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_acceleration_following;
            }
            if (index1 == 107) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_acceleration_speed_limit;
            }
            if (index1 == 108) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_aceleration_signalized_intersection;
            }
            if (index1 == 109) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_desired_aceleration_unsignalized_intersection;
            }
            if (index1 == 104) {
                *double_value = (*current_timestep_map)[vehicle_id()].veh_BADF_setspeed;
            }
        }
        return 1; // doesn't set any UDA values 
    case DRIVER_DATA_WANTS_SUGGESTION:
        *int_value = 1;
        return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION:
        *double_value = (*current_timestep_map)[vehicle_id()].veh_desired_acceleration;
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE:
        //*double_value = (*current_timestep_map)[vehicle_id()].veh_desired_lane_angle;
        return 0;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE:
        *int_value = (*current_timestep_map)[vehicle_id()].veh_desired_active_lane_change;
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE:
        if ((*current_timestep_map)[vehicle_id()].veh_desired_active_lane_change == 0)
        {
            (*current_timestep_map)[vehicle_id()].veh_rel_target_lane = 0;
        }
        return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
      *int_value = 1;
      return 1;
    case DRIVER_DATA_USE_INTERNAL_MODEL:
      *int_value = 0; /* must be set to 0 if external model is to be applied */
      return 1;
    case DRIVER_DATA_WANTS_ALL_NVEHS:
      *int_value = 0; /* must be set to 1 if data for more than 2 nearby vehicles per lane and upstream/downstream is to be passed from Vissim */
      return 1;
    case DRIVER_DATA_ALLOW_MULTITHREADING:
      *int_value = 1; /* must be set to 1 to allow a simulation run to be started with multiple cores used in the simulation parameters */
      return 1;
    default:
      return 0;
  }
}

/*--------------------------------------------------------------------------*/
/* DLL --> VISSIM */
DRIVERMODEL_API  int  DriverModelGetValue3 (int    type,
                                            int    index1,
                                            int    index2,
                                            int    index3,
                                            int    *int_value,
                                            double *double_value,
                                            char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1>, */
  /* <index2> and <index3>, and writes that value to <*int_value>,         */
  /* <*double_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                 */
  /* Return value is 1 on success, otherwise 0.                            */
  /* DriverModelGetValue (DRIVER_DATA_MAX_NUM_INDICES, ...) needs to set   */
  /* *int_value to 3 or greater in order to activate this function!        */

  switch (type) {
    default:
      return 0;
    }
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (int  number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */

  switch (number) {
    case DRIVER_COMMAND_INIT :
      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
      return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
      return 1;
    default :
      return 0;
  }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
