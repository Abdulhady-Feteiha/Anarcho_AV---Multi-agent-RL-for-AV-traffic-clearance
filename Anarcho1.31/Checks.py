import os
import sys
import optparse
import numpy as np
from sumolib import checkBinary
import traci
from Utils.measure_max_window import measure
from Config import *
from Utils.Vehicle import Vehicle
from Utils.helpers import *
from RL.SingleAgent import RLAlgorithm
from Environment.env import env
from Anarcho import *
import numpy as np





def are_we_ok(check_num,statistics, Proudhon ,current_step ,previous_action , previous_state ,current_state , environment ):

    count_non_requested_lane_change = 0
    count_acc_errors = 0
    count_dec_errors = 0
    count_no_acc_errors = 0
    count_non_requested_speed_change = 0
    count_change_left_errors = 0
    count_change_right_errors = 0


    if (check_num == 0):
        if (Proudhon is None):
            Proudhon = env(vehicles_list)
        if (vehicles_list[0].getL() != (Proudhon.emer_start_lane  )):

            raise ValueError(
                f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {vehicles_list[0].getL()} on step {current_step} . "
                f"\nAmbulance Should not change lane. Quitting.")

    elif (check_num == 1):
        rel_distance = current_state[4]
        if (enable_rl_dis_engagement & ( ~ ((rel_distance < environment.rel_amb_y_max) & (rel_distance > environment.rel_amb_y_min)))):

            # if  the agent is  outside ambulance window and rl _disenagement is enabled  , disable RL and as a result do not track its errors and consider them as zeros .
            pass
        else:
            if (current_step>4):



                if (previous_action  in ["acc" , "no_acc" , "dec"]):
                    if (previous_state[1]!=current_state[1]):
                            if(statistics):
                                count_non_requested_lane_change += 1
                            else:
                                print(f" previous_lane equals {previous_state[1]}, previous_action {previous_action} , current_lane equals {current_state[1]} " )
                                raise ValueError(
                                    "Lane change has happened although it is not required"
                                )

                    if ((previous_action == "acc") & (previous_state[0]!=5) & (current_state[0]<=previous_state[0])):     #5 is the agent_max speed , and it is hard_coded
                            if (statistics):
                                count_acc_errors += 1
                            else:
                                raise ValueError(
                                    "Acc action has been requested , However agent speed has not increased "
                                    f" agent_previous_Spd was {previous_state[0]} and agent_current_Spd was {current_state[0]} requested action is {previous_action}"
                                )
                    if ((previous_action == "dec") & (previous_state[0] !=0) & (current_state[0] >= previous_state[0])):

                            if (statistics):
                                count_dec_errors +=1
                            else :
                                raise ValueError(
                                    "dec action has been requested , However agent speed has not decreased "
                                     f" agent_previous_Spd was {previous_state[0]} and agent_current_Spd was {current_state[0]} requested action is {previous_action}"
                                 )

                    if ((previous_action== "no_acc") & (previous_state[0] != current_state[0])):

                            if (statistics):

                                count_no_acc_errors +=1
                            else:
                                raise ValueError (
                                    " Speed should not change , no_acc action has been requested "
                                    f" agent_previous_Spd was {previous_state[0]} and agent_current_Spd was {current_state[0]} requested action is {previous_action}"
                                )



                if (previous_action in  ["change_left" , "change_right"] ):

                    if (previous_state[0] != current_state[0]):

                        if (statistics):
                            count_non_requested_speed_change +=1
                        else:
                            difference = abs(current_state[0]-previous_state[0])
                            raise ValueError(
                                f" agent previous speed is {previous_state[0]} and agent current speed is {current_state[0]} "
                                f"previous_action={previous_action} agent speed has changed while no speed change action has requested and the difference is { difference } "
                                f" and current step is {current_step}"
                            )


                    if ( (previous_action == "change_left") & (current_state[1] != (previous_state[1]+1)) ):

                        if (statistics):
                            count_change_left_errors += 1
                        else:
                            raise ValueError(
                            f"let 's explore data types current_state type {type(current_state[1])} previous_state type {type(previous_state[1])}"
                            
                            f" Although {previous_action} action has been requested , agent_current lane equals {current_state[1]} "
                            f"while his previous lane was {previous_state[1]}  "
                        )
                    elif ( (previous_action == "change_right") & (current_state[1] != (previous_state[1]-1))):

                        if (statistics):
                            count_change_right_errors += 1
                        else:
                            raise  ValueError (
                                f" Although {previous_action} action has been requested , agent_current lane equals {current_state[1]} "
                                f"while his previous lane was {previous_state[1]}  "
                            )



    if (statistics):
        return np.asarray([count_non_requested_lane_change, count_acc_errors, count_dec_errors, count_no_acc_errors, count_non_requested_speed_change,
                 count_change_left_errors, count_change_right_errors])
    else:
        return None
