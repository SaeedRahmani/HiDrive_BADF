#include "UnsignalizedIntersection.h"
#include "Calibration.h" 
#include <math.h>
#include <stdlib.h>


// function declaration
double time_pass(double dist, double accel, int type, double v_start) {
    if (type == 0) /*crossing*/
    {
        return (-v_start + sqrt(v_start * v_start + 2 * accel * dist)) / accel;
    }
    if (type == 1) /* merging - added 4 sec for left and right turn  */
    {
        return (-v_start + sqrt(v_start * v_start + 2 * accel * dist)) / accel + 2;
    }
}

double solve(double a, double b, double g)
{
    return ((-b + sqrt(b * b - 4 * a * g)) / (2 * a));
}

double solve2(double a, double b, double g)
{
    return ((-b - sqrt(b * b - 4 * a * g)) / (2 * a));
}

double accel(double V1, double V2, double s)
{
    return ((V1 * (V2 - V1) + (V2 - V1) * (V2 - V1) / 2) / s);
}

void ApproachingIntersection::UnsignalizedIntersection(car_data& curr_data, car_data& prev_data) {

    double max_acceleration = 6.0;
    double comfort_acceleration = 2.5;
    double comfort_accelerationnew = 1.5;
    double safety_gap = 1.0;
    double simu_time = curr_data.timestep;
    double safety_distance = 1.0;
    double target_time = 999.0;
    double time_enter_ego = 999.0;
    double time_exit_ego = 999.0;
    int lead_exists = 0;
    double target_entrance_speed = 10 / 3.6;
    int veh_id = curr_data.veh_id;
    //double accel_temp = 0.0;


    if (curr_data.veh_type == BADF)
    {
        

        if (((curr_data.conf_area_dist[0] - curr_data.conf_area_length[0]-safety_distance) <= ((curr_data.veh_velocity* curr_data.veh_velocity)/2/ comfort_accelerationnew)) && (curr_data.conf_area_dist[0]!=999.0)) /*IF Group A/ 40m = (50/3.6)*(50/3.6)/(2*2.5), the distance at which the vehicle can safely stop before the conflict with comfort decelaration */
        {
            for (int i = 0; i < curr_data.conf_num; i++) // check if there is a stopped priority vehicle
            {
                if ((curr_data.time_enter_0[i] + simu_time) < -0.01)
                {
                    curr_data.waiting_time_others[i] = prev_data.waiting_time_others[i] + curr_data.ts_length;
                }
            }

            for (int i = 0; i < curr_data.conf_num; i++) //check that there is no lead vehicle in ANY of the conflict areas ahead
            {
                if ((curr_data.lead_veh_distance - curr_data.lead_veh_length) < (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0]))  /*there is a lead vehcle*/
                {
                    lead_exists = 1;
                }
            }
            if (fabs(curr_data.veh_velocity - curr_data.lead_veh_rel_velocity) < 2.0)
            {
                lead_exists = 1;
            }

            if (lead_exists == 0)  /*its the first vehicle (sometimes in turns, there is a lead vehicle that is closer to the ego vehicle than the conf_area_dist)*/
            {
                if ((prev_data.decided_to_yield == 999)) /* ego vehicle hasnt taken any decision yet - decided_to_yield=999*/
                {
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        if (curr_data.conf_area_yield[i] == 1)/* BADF always decelerates to 10kph before checking*/
                        {
                            curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = accel(curr_data.veh_velocity, target_entrance_speed, (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance));
                            curr_data.decided_to_yield = 1;
                        }
                    }
                }
            }
        }

                if ((prev_data.decided_to_yield == 1) && (curr_data.conf_area_dist[0] != 999.0)) /* ego has already decided to yield*/
                {
                    curr_data.decided_to_yield = 1;
                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                    if ((curr_data.conf_area_dist[0] - curr_data.conf_area_length[0]) < safety_distance*2)// BADF CHECKS IF IT CAN CROSS OR NOT//
                    { 
                      for (int i = 0; i < curr_data.conf_num; i++)
                      {                       
                          //curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], curr_data.veh_BADF_desired_aceleration_unsignalized_intersection, curr_data.conf_area_type[i], prev_data.veh_velocity);
                          curr_data.time_vehicle_pass[i] = curr_data.conf_area_dist[i] / curr_data.veh_velocity;
                        if (curr_data.conf_area_yield[i] == 1) //ego has to give priority//                          
                        {
                            if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.001))  /*IF Group G,assumption: BADF checks only with the first conflicting vehicle, when the conflicting vehicle is too far away vissim sends time_enter=0. time_enter = -1 according to the manual refers to conflicting vehicles that will never enter the conflict. this is true for conflicting vehicles waiting just before the intersection but sometimes we get values -1 for vehicles too far away*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance); 
                                curr_data.decided_to_yield = 1;
                            }
                            if ((curr_data.time_enter_0[i] + simu_time) < -0.001)  /*IF priority vehicle is stopped, ego stops*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance); /* added a safety distance of 2.00m*/
                                curr_data.decided_to_yield = 1;
                            }
                        }
                      }
                    }
                    if (prev_data.veh_velocity < (10/3.6))  /* ego vehicle has already stopped and checks if it can cross with comfort acceleration */
                    {
                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = comfort_acceleration;
                        curr_data.decided_to_yield = 2; //initially intention to start unless one of the following applies, then it cancels and waits 
                        curr_data.waiting_time = 0.0;
                        curr_data.accel_temp = comfort_acceleration;

                        
                        for (int i = 0; i < curr_data.conf_num; i++)
                        {
 
                            curr_data.time_vehicle_pass[i] = time_pass (curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                            if (curr_data.conf_area_yield[i] == 0) // ego has priority
                            {
                                if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.01))// excludes cases where time is around 0 and -1. "o" doesnt exist, -1 stops 
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 1;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                }
                            }
                            if (curr_data.conf_area_yield[i] == 1)// ego must give priority
                            {
                                  if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (rand() % 101 < 70)) // when both non priority are stoppe, randomly cancel
                                  //if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (curr_data.veh_id > curr_data.time_enter_unique[i]))
                                  {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 1;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                  } 
                                if ((curr_data.time_enter_0[i] + simu_time) > 0.01)   /*priority conflicting vehicle present and not stopped*/
                                {
                                    if (curr_data.time_vehicle_pass[i] > (curr_data.time_enter_0[i]))
                                    {
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                        curr_data.decided_to_yield = 1;
                                        curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                    }
                                }
                            }
                            if ((curr_data.time_enter_0[i] > prev_data.time_enter_0[i]) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that slows down on a lane that has priority so ego must not start*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 1;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                            if ((curr_data.time_enter_0[i] > 0) && ((prev_data.time_enter_0[i] + simu_time) < -0.01) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that decided to start*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 1;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                        }
                    }
                } 

                if ((prev_data.decided_to_yield == 2) && (curr_data.conf_area_dist[0] <30.0)) // continue to accelerate*/
                {
                    curr_data.decided_to_yield = 2;
                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                        if ((curr_data.time_enter_0[i] + simu_time) > 0.01)   /*priority conflicting vehicle present and not stopped*/
                        {
                            if ((curr_data.time_vehicle_pass[i] > (curr_data.time_enter_0[i])) && (curr_data.conf_area_yield[i] == 1) && (curr_data.conf_area_dist[i]> curr_data.conf_area_length[i]))
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = - max_acceleration;
                                curr_data.decided_to_yield = 3;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                        }
                    }
                }
            
                if ((prev_data.decided_to_yield == 3) && (curr_data.conf_area_dist[0] <30.0)) /*IF Group D, ego has already decided to yield*/
                {
                    
                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = comfort_acceleration;
                        curr_data.decided_to_yield = 2; //initially intention to start unless one of the following applies, then it cancels and waits 
                        curr_data.waiting_time = 0.0;
                        for (int i = 0; i < curr_data.conf_num; i++)
                        {
                            curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                            if (curr_data.conf_area_yield[i] == 0) // ego has priority
                            {
                                if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.01))// excludes cases where time is around 0 and -1. "o" doesnt exist, -1 stops 
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 3;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                }
                            }
                            if (curr_data.conf_area_yield[i] == 1)// ego must give priority
                            {
                                if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (rand() % 101 < 70))
                                    //if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (curr_data.veh_id > curr_data.time_enter_unique[i]))
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 3;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                } 
                                if ((curr_data.time_enter_0[i] + simu_time) > 0.01)   /*priority conflicting vehicle present and not stopped*/
                                {
                                    if (curr_data.time_vehicle_pass[i] > (curr_data.time_enter_0[i]))
                                    {
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                        curr_data.decided_to_yield = 3;
                                        curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                    }
                                }
                            }
                            if ((curr_data.time_enter_0[i] > prev_data.time_enter_0[i]) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that slows down on a lane that has priority so ego must not start*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 3;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                            if ((curr_data.time_enter_0[i] > 0) && ((prev_data.time_enter_0[i] + simu_time) < -0.01) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that decided to start*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 3;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                        }
                   
                } 

            if ((prev_data.decided_to_yield == 1) && (curr_data.lead_veh_distance < 8))/* we have added this condition because we have noticed that sometimes the crossing vehicles appears as lead vehicle (lead vehicle distance changes changes from 80m to 10m (thewrei leading apo ekei kai meta ton conflicting kaimas pairnei decided_to _yield 999*/
            {
                curr_data.time_vehicle_pass[0] = curr_data.conf_area_dist[0] / curr_data.veh_velocity; /*just for check - its the initial checking value */
                curr_data.decided_to_yield = 1;
                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
            }
    }
    




    if (curr_data.veh_type == EADF)
    {
        if (((curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance) <= ((curr_data.veh_velocity * curr_data.veh_velocity) / 2 / comfort_acceleration)) && (curr_data.conf_area_dist[0] != 999.0)) /*IF Group A/ 40m = (2*50/3.6)*(2*50/3.6)/(2*2.5), the distance at which the vehicle can safely stop before the conflict with comfort decelaration */
        {
            for (int i = 0; i < curr_data.conf_num; i++) 
            {
                if ((curr_data.time_enter_0[i] + simu_time) < -0.01)
                {
                    curr_data.waiting_time_others[i] = prev_data.waiting_time_others[i] + curr_data.ts_length;
                }
            }

            for (int i = 0; i < curr_data.conf_num; i++) //check that there is no lead vehicle in ANY of the conflict areas ahead
            {
                if ((curr_data.lead_veh_distance - curr_data.lead_veh_length) < (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0]))  /*there is a lead vehcle*/
                {
                    lead_exists = 1;
                }
            }
            if (fabs(curr_data.veh_velocity - curr_data.lead_veh_rel_velocity) < 2.0)
            {
                lead_exists = 1;
            }
            if (lead_exists == 0)  // STOP CONDITION,its the first vehicle (sometimes in turns, there is a lead vehicle that is closer to the ego vehicle than the conf_area_dist)*/
            {
                if ((prev_data.decided_to_yield == 999)) /* ego vehicle hasnt taken any decision yet - decided_to_yield=999*/
                {
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        if (curr_data.conf_area_yield[i] == 1)/* IF it cantnot cross before an area with yield=1, it stops 1m before the "0" conflict*/
                        {
                            //curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], curr_data.veh_desired_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                            curr_data.time_vehicle_pass[i] = curr_data.conf_area_dist[i] / curr_data.veh_velocity;
                            if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.001))  /*BADF checks with the first conflicting vehicle*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance); /* added a safety distance of 2.00m*/
                                curr_data.decided_to_yield = 1;
                            }
                            if ((curr_data.time_enter_0[i] + simu_time) < -0.001)  /*IF priority vehicle is stopped, ego stops*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance); /* added a safety distance of 2.00m*/
                                curr_data.decided_to_yield = 1;
                            }
                        }
                    }
                } 
            }
        }

                if ((prev_data.decided_to_yield == 1) && (curr_data.conf_area_dist[0] != 999.0)) /* ego has already decided to yield*/
                {
                    curr_data.decided_to_yield = 1;
                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        curr_data.time_vehicle_pass[i] = curr_data.conf_area_dist[i] / curr_data.veh_velocity;
                    }
                    if (prev_data.veh_velocity < 0.1)  /*, BADF: ego vehicle has already stopped and checks if it can cross with comfort acceleration */
                    {
                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = comfort_acceleration;
                        curr_data.decided_to_yield = 2; //initially intention to start unless one of the following applies, then it cancels and waits 
                        curr_data.waiting_time = 0.0;
                        for (int i = 0; i < curr_data.conf_num; i++)
                        {
                            curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                            if (curr_data.conf_area_yield[i] == 0) // ego has priority
                            {

                                if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.01))// excludes cases where time is around 0 and -1. "o" doesnt exist, -1 stops 
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 1;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                }
                            }
                            if (curr_data.conf_area_yield[i] == 1)// ego must give priority
                            {
                                

                                if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (rand() % 101 < 70))
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 1;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                } 
                                if ((curr_data.time_enter_0[i] + simu_time) > 0.01)   /*priority conflicting vehicle present and not stopped*/
                                {
                                    if (curr_data.time_vehicle_pass[i] > (curr_data.time_enter_0[i]))
                                    {
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                        curr_data.decided_to_yield = 1;
                                        curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                    }
                                }
                            }
                            if ((curr_data.time_enter_0[i] > prev_data.time_enter_0[i]) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that slows down on a lane that has priority so ego must not start*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 1;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                            if ((curr_data.time_enter_0[i] > 0) && (prev_data.time_enter_0[i] + simu_time < -0.01) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that decided to start*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 1;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                        }

                    }
                } 

                        

                if ((prev_data.decided_to_yield == 2) && (curr_data.conf_area_dist[0] <30.0)) /*IF Group D, continue to accelerate*/
                {
                    curr_data.decided_to_yield = 2;
                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                        if ((curr_data.time_enter_0[i] + simu_time) > 0.01)   /*priority conflicting vehicle present and not stopped*/
                        {
                            if ((curr_data.time_vehicle_pass[i] > (curr_data.time_enter_0[i])) && (curr_data.conf_area_yield[i] == 1) && (curr_data.conf_area_dist[i] > curr_data.conf_area_length[i]))
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = - max_acceleration;
                                curr_data.decided_to_yield = 3;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                        }
                    }
                }
            
                if ((prev_data.decided_to_yield == 3) && (curr_data.conf_area_dist[0] <30.0)) /*IF Group D, ego has already decided to yield*/
                {

                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = comfort_acceleration;
                    curr_data.decided_to_yield = 2; //initially intention to start unless one of the following applies, then it cancels and waits 
                    curr_data.waiting_time = 0.0;
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                        if (curr_data.conf_area_yield[i] == 0) // ego has priority
                        {
                            if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.01))// excludes cases where time is around 0 and -1. "o" doesnt exist, -1 stops 
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 3;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                        }
                        if (curr_data.conf_area_yield[i] == 1)// ego must give priority
                        {
                            if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (rand() % 101 < 70))
                                //if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (curr_data.veh_id > curr_data.time_enter_unique[i]))
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 3;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            } 
                            if ((curr_data.time_enter_0[i] + simu_time) > 0.01)   /*priority conflicting vehicle present and not stopped*/
                            {
                                if (curr_data.time_vehicle_pass[i] > (curr_data.time_enter_0[i]))
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 3;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                }
                            }
                        }
                        if ((curr_data.time_enter_0[i] > prev_data.time_enter_0[i]) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that slows down on a lane that has priority so ego must not start*/
                        {
                            curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                            curr_data.decided_to_yield = 3;
                            curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                        }
                        if ((curr_data.time_enter_0[i] > 0) && ((prev_data.time_enter_0[i] + simu_time) < -0.01) && (curr_data.conf_area_yield[i] == 1))  /*there is a conflicting priority vehicle that decided to start*/
                        {
                            curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                            curr_data.decided_to_yield = 3;
                            curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                        }
                    }

                } 


            if ((prev_data.decided_to_yield == 1) && (curr_data.lead_veh_distance < 8))/* we have added this condition because we have noticed that sometimes the crossing vehicles appears as lead vehicle (lead vehicle distance changes changes from 80m to 10m (thewrei leading apo ekei kai meta ton conflicting kaimas pairnei decided_to _yield 999*/
            {
                curr_data.time_vehicle_pass[0] = curr_data.conf_area_dist[0] / curr_data.veh_velocity; /*just for check - its the initial checking value */
                curr_data.decided_to_yield = 1;
                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
            }
        
    }




    if (curr_data.veh_type == 620)
    {
        if (curr_data.conf_area_dist[0] < 40.0) /*IF Group A/ 40m = (2*50/3.6)*(2*50/3.6)/(2*2.5), the distance at which the vehicle can safely stop before the conflict with comfort decelaration */
        {
            for (int i = 0; i < curr_data.conf_num; i++) // check if there is a stopped priority vehicle
            {
                //if (curr_data.conf_area_yield[i] == 1) // no priority
                //{
                if ((curr_data.time_enter_0[i] + simu_time) < -0.01)
                {
                    curr_data.waiting_time_others[i] = prev_data.waiting_time_others[i] + curr_data.ts_length;
                }
                //}
            }


            for (int i = 0; i < curr_data.conf_num; i++) //check that there is no lead vehicle in ANY of the conflict areas ahead
            {
                if ((curr_data.lead_veh_distance - curr_data.lead_veh_length) < (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0]))  /*there is a lead vehcle*/
                {
                    lead_exists = 1;
                }
            }
            if (fabs(curr_data.veh_velocity - curr_data.lead_veh_rel_velocity) < 2.0)
            {
                lead_exists = 1;
            }


            if (lead_exists == 0)  /*IF Group B, */
            {
                if (prev_data.decided_to_yield == 999) /*IF Group D, ego vehicle hasnt taken any decision yet - decided_to_yield=999*/
                {
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        if (curr_data.conf_area_yield[i] == 1)
                        {
                            curr_data.conf_first = i; // first yielding conflict area, EADF takes clever decision only with the first one to decide if it can pass between the  1st and 2nd vehicle
                            break;
                        }
                    }
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        if (curr_data.conf_area_yield[i] == 1)
                        {
                            if (curr_data.decided_to_yield == 999)
                            {
                                curr_data.time_vehicle_pass[i] = curr_data.conf_area_dist[i] / curr_data.veh_velocity;
                            }
                            if (curr_data.decided_to_yield == 3) // has decided to adjust speed with target_acceleration to cross between 1st and 2nd conflict vehicle
                            {
                                curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i] - curr_data.conf_area_length[i], curr_data.target_acceleration, curr_data.conf_area_type[i], curr_data.veh_velocity);
                                time_enter_ego = time_pass(curr_data.conf_area_dist[i] - curr_data.conf_area_length[i], curr_data.target_acceleration, curr_data.conf_area_type[i], curr_data.veh_velocity);
                                time_exit_ego = time_pass(curr_data.conf_area_dist[i], curr_data.target_acceleration, curr_data.conf_area_type[i], curr_data.veh_velocity);
                            }
                            if (curr_data.decided_to_yield == 999) // if EADF hasnt decided yet
                            {
                                if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.001))   // if it cant cross befor the first it brakes
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance); /* added a safety distance of 2.00m*/
                                    curr_data.decided_to_yield = 1;

                                    if ((i == curr_data.conf_first) && (curr_data.time_enter_1[i] != 999.0)) //EADF inteligence only for tha first conflict area, if it can adjust its speed it crosses
                                    {
                                        target_time = (curr_data.time_exit_0[i] + curr_data.time_enter_1[i]) / 2;
                                    }
                                    if ((i == curr_data.conf_first) && (curr_data.time_enter_1[i] == 999.0)) //EADF inteligence if there is no 2nd conflictin vehicle/ cross 2*safety gaps after tha 1st conflicting vehicle
                                    {
                                        target_time = curr_data.time_exit_0[i] + 2 * safety_gap;
                                    }
                                    curr_data.target_acceleration = 2 * (curr_data.conf_area_dist[i] - curr_data.veh_velocity * target_time) / (target_time * target_time);
                                    if ((fabs(curr_data.target_acceleration) < comfort_acceleration) && ((curr_data.veh_velocity + curr_data.target_acceleration * solve(curr_data.target_acceleration / 2, curr_data.veh_velocity, -curr_data.conf_area_dist[curr_data.conf_num]))) > 3.5)// should not end in the end of conflicts areas with too low speed
                                    {
                                        curr_data.decided_to_yield = 3; //decides to adjust
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = curr_data.target_acceleration;
                                    }

                                    if ((curr_data.time_enter_0[i] + simu_time) < -0.001)  /*IF priority vehicle is stopped, ego stops*/

                                    {
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance); /* added a safety distance of 2.00m*/
                                        curr_data.decided_to_yield = 1;
                                    }


                                }
                            }

                            if (curr_data.decided_to_yield == 3) // EADF if it has decided to adjust but cannot safely cross the next conflict areas it breaks
                            {
                                if (i != curr_data.conf_first)
                                {
                                    if ((curr_data.time_enter_0[i] > time_enter_ego) && (curr_data.time_enter_0[i] < time_exit_ego))
                                    {
                                        curr_data.decided_to_yield = 1;
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance);
                                    }
                                    if ((curr_data.time_exit_0[i] > time_enter_ego) && (curr_data.time_exit_0[i] < time_exit_ego))
                                    {
                                        curr_data.decided_to_yield = 1;
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -(curr_data.veh_velocity * curr_data.veh_velocity) / 2 / (curr_data.conf_area_dist[0] - curr_data.conf_area_length[0] - safety_distance);
                                    }
                                }
                            }
                        }
                    }
                } /*/*IF Group D, ego hasnt taken any decision yet - decided_to_yield=999 - END*/


                if (prev_data.decided_to_yield == 1) /*IF Group D, ego has already decided to yield*/
                {
                    curr_data.decided_to_yield = 1;
                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        curr_data.time_vehicle_pass[i] = curr_data.conf_area_dist[i] / curr_data.veh_velocity;
                    }

                    if (prev_data.veh_velocity < 3.5)  //IF group E, EADF: ego does not need to completely stop and check if it can cross with comfort
                    {

                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = comfort_acceleration;
                        curr_data.decided_to_yield = 2; //initially intention to start unless one of the following applies, then it cancels and waits 
                        curr_data.waiting_time = 0.0;
                        for (int i = 0; i < curr_data.conf_num; i++)
                        {
                            curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);

                            if (curr_data.conf_area_yield[i] == 0) // ego has priority
                            {

                                if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.01))// excludes cases where time is around 0 and -1. "o" doesnt exist, -1 stops 
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 1;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                }

                            }

                            if (curr_data.conf_area_yield[i] == 1)// ego must give priority
                            {
                                ////if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (prev_data.waiting_time < prev_data.waiting_time_others[i]))  //priority conflicting vehicle stopped//
                                //if ((curr_data.time_enter_0[i] + simu_time) < -0.01)
                                //{
                                    //curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    //curr_data.decided_to_yield = 1;
                                    //curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                //}//TO svisame 361-366 stis 04 09 gia na min kolane oi duo aristeres strofes


                                if (((curr_data.time_enter_0[i] + simu_time) < -0.01) && (rand() % 101 < 70))
                                {
                                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                    curr_data.decided_to_yield = 1;
                                    curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                }////ayto to bazw instead the above me to rand gia na doume an den kolane pia sthn diastaurwsi. alliws to afairw kai ayto kai to proigoumeno kai exw crashes



                                if ((curr_data.time_enter_0[i] + simu_time) > 0.01)   /*priority conflicting vehicle present and not stopped*/
                                {

                                    if (curr_data.time_vehicle_pass[i] > (curr_data.time_enter_0[i]))
                                    {
                                        curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                        curr_data.decided_to_yield = 1;
                                        curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                                    }
                                }
                            }



                            if ((curr_data.time_enter_0[i] > prev_data.time_enter_0[i]) && (curr_data.conf_area_yield[i] == 0))  /*there is a conflicting priority vehicle that slows down on a lane that has priority so ego must not start*/
                            {
                                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                                curr_data.decided_to_yield = 1;
                                curr_data.waiting_time = prev_data.waiting_time + curr_data.ts_length;
                            }
                            //if ((curr_data.conf_area_yield[i] == 0) && (curr_data.time_enter_0[i] > 3.0))  /*there is a conflicting priority vehicle that is far away*/
                            //{
                                //curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = comfort_acceleration;
                                //curr_data.decided_to_yield = 2;
                            //}
                        //}


                        }

                    }/*IF Group E, already stopped and check when to start again */
                } /*IF Group D, decided_to_yield=1*/

                if (prev_data.decided_to_yield == 2) /*IF Group D, continue to accelerate*/
                {
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], prev_data.veh_BADF_desired_aceleration_unsignalized_intersection, curr_data.conf_area_type[i], prev_data.veh_velocity);
                    }
                    curr_data.decided_to_yield = 2;
                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;

                }
                if (prev_data.decided_to_yield == 3)/*IF Group D, ego has already decided to adjust*/
                {
                    for (int i = 0; i < curr_data.conf_num; i++)
                    {
                        curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], prev_data.veh_BADF_desired_aceleration_unsignalized_intersection, curr_data.conf_area_type[i], prev_data.veh_velocity);
                    }
                    curr_data.decided_to_yield = 3;
                    curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
                } /*IF Group D*/


            }/*IF Group B*/

            //if ((prev_data.decided_to_yield == 2) && (curr_data.veh_velocity<8.0)) 
            //{
                //for (int i = 0; i < curr_data.conf_num; i++)
                //{
                    //if (curr_data.conf_area_yield[i] == 1)

                    //{
                        //curr_data.time_vehicle_pass[i] = time_pass(curr_data.conf_area_dist[i], comfort_acceleration, curr_data.conf_area_type[i], prev_data.veh_velocity);
                        //if ((curr_data.time_vehicle_pass[i] > curr_data.time_enter_0[i]) && ((curr_data.time_enter_0[i] + simu_time) > 0.001))  /*IF Group G,assumption: BADF checks only with the first conflicting vehicle, when the conflicting vehicle is too far away vissim sends time_enter=0. time_enter = -1 according to the manual refers to conflicting vehicles that will never enter the conflict. this is true for conflicting vehicles waiting just before the intersection but sometimes we get values -1 for vehicles too far away*/
                        //{

                            //curr_data.decided_to_yield = 1;
                            //curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = -max_acceleration;
                        //}
                        //break;
                    //}
                //}

            //} 

            if ((prev_data.decided_to_yield == 1) && (curr_data.lead_veh_distance < 8))/* we have added this condition because we have noticed that sometimes the crossing vehicles appears as lead vehicle (lead vehicle distance changes changes from 80m to 10m (thewrei leading apo ekei kai meta ton conflicting kaimas pairnei decided_to _yield 999*/
            {
                curr_data.time_vehicle_pass[0] = curr_data.conf_area_dist[0] / curr_data.veh_velocity; /*just for check - its the initial checking value */
                curr_data.decided_to_yield = 1;
                curr_data.veh_BADF_desired_aceleration_unsignalized_intersection = prev_data.veh_BADF_desired_aceleration_unsignalized_intersection;
            }

        }/*IF Group A/*/

    }



}
