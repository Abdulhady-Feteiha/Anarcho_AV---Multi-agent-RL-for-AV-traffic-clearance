#!/usr/bin/env python
import os
import sys
import optparse
from math import sqrt, ceil
from random import randrange
import numpy as np
import pandas as pd
from Config import Sumocfg_DIR,SimTime,vehicles_list
from Utils.Vehicle import Vehicle
# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci



def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


#---------------------------------------------------------#




#---------------------------------------------------------#

def getters(vs):
    for v in vs:
        v.getSpd()
        v.getRoute() #Note: getRoute must always be called before getPose
        v.getPose()
        v.getAcc()
        v.getL()

def defGlobals():
    '''
    global SimTime,LH,RB,track_len,fast,slow,Q,speed_range
    speed_range = np.arange(0,30,5)
    fast = 0
    slow = 1
    track_len = 4000
    SimTime = 1000
    '''





def measure():
    traci.start(['sumo', "-c", Sumocfg_DIR])
    for vehc in vehicles_list:
        vehc.initialize()
    defGlobals()
    step = 0


    #print("Hello world, I'm an AV, and I love carrots")
    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()
        vehicles_list[0].chL(0)
        vehicles_list[1].chL(0)
        traci.vehicle.setSpeed(vehicles_list[1].ID,0)
        if step ==2:
            getters(vehicles_list)
        elif step>2:
            lastStepSpd = vehicles_list[0].spd
            lastStepDist = vehicles_list[1].lane_pose-vehicles_list[0].lane_pose
            getters(vehicles_list)

            decel =vehicles_list[0].spd-lastStepSpd
            if decel<0:
                traci.close()
                sys.stdout.flush()
                os.system("clear")

                print("At setp: ",step-1)
                print("Vehicle expeienced a decel of: ",decel)
                print("Due to a min distance of: ",lastStepDist)
                return lastStepDist


        step += 1
