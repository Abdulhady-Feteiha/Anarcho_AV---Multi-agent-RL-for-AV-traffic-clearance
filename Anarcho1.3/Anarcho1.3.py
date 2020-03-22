#!/usr/bin/env python
import os
import sys
import optparse
from math import sqrt, ceil
from random import randrange
import numpy as np

from Utils.measure_max_window import measure
from Config import *
from Utils.Vehicle import Vehicle
from Utils.helpers import *
from RL.SingleAgent import RLAlgorithm
from Environment.env import env
# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci






#---------------------------------------------------------#




#---------------------------------------------------------#


def defGlobals():
    '''
    :function:  sets a few global variables
    Q                #Q_table. Multi-dimensional np.ndarray, each dimension: either state partial assignment or action (string action -> integer)
                      transformation is defined via action_to_string_dict
                     #Access order for the Q_table is [agent_vel][agent_lane][amb_vel][amb_lane][rel_amb_y][action]

                     #Values are kept as integers by rounding and casting as int. Values are clipped using np.clip() function
                     # agent_vel  (6): [0,1,2,3,4,5] #Clipped before applying velocity
                     # agent_lane (3): [0,1,2]
                     # amb_vel    (11): [0,1,2,3,4,5,6,7,8,9,10]
                     # amb_lane   (3): [0,1,2]
                     # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
                        #since window is designed to be:
                        #<--4 time steps *10 cells/step  = 40 steps  behind -- . agent . -- 3 steps * 5 cells/sec -->

                    # action     (5) : [Change left, Change right, acc +1, acc -1, acc 0]

                    Q_table size, is therefore = 6 * 3 * 11 *3 * 58 * 5 = 172260 ~ 170K .
                        Note that some Q(s,a) pairs will be infeasible and hence will not be trained/updated.


    :return:  None. Just defines global variables
    '''





def run():
    step = 0


    Proudhon = env(vehicles_list)  # [LH, RB]
    traci.simulationStep()  # apply_action
    # RB_ comment, why didn't you increase the step after applying simulationstep?
    getters(vehicles_list)
    #LH.chL(0) #No need for this now, automatic lane change is removed. Checkout Vehicle.__init__() : traci.vehicle.setLaneChangeMode(self.ID, 256)
    while traci.simulation.getMinExpectedNumber() > 0:

        amb_last_velocity = Proudhon.emer.spd

        traci.simulationStep() #apply_action
        step += 1

        getters(vehicles_list)

        Proudhon.measure_full_state()
        Proudhon.get_feasible_actions(vehicles_list[1])
        Proudhon.get_feasible_actions(vehicles_list[0])

        print(step,' : ','[agent_vel , agent_lane , amb_vel , amb_lane , rel_amb_y]')
        print(step,' :  ',Proudhon.observed_state)

        done = Proudhon.are_we_done(full_state=Proudhon.full_state, step_number=step)

        print("reward: ", Proudhon.calc_reward(amb_last_velocity, done, step) )

        if(done):
            if(done == 1):
                print("We are done here due to PASSING MAX STEPS NUMBER!")
            elif(done == 2):
                print("We are done here due to AMBULANCE PASSING THE GOAL!")
            elif(done == 3): #TODO: #TOFIX: What should be the state here?
                print("We are done here due to AGENT PASSING THE GOAL!")
            else:
                print("We are done here BUT I DON'T KNOW WHY!")

            break



    traci.close()
    sys.stdout.flush()


# main entry point
min_window = measure()

if GUI_on:
    sumoBinary = checkBinary('sumo-gui')
else:
    sumoBinary = checkBinary('sumo')

traci.start([sumoBinary, "-c", Sumocfg_DIR,
                         "--tripinfo-output", "tripinfo.xml"])
for vehc in vehicles_list:
    vehc.initialize()
run()
