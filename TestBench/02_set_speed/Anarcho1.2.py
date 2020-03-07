#!/usr/bin/env python
import os
import sys
import optparse
from math import sqrt, ceil
from random import randrange
import numpy as np
import pandas as pd

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
class Vehicle:
    def __init__(self, ID):
        self.ID = ID
        self.base_route = 1 #Index of base route
        self.length_of_base_route = 100 #Length of base route

        traci.vehicle.setLaneChangeMode(self.ID, 256)
        '''To disable all autonomous changing but still handle safety checks in the simulation,
        either one of the modes 256 (collision avoidance) or 512 (collision avoidance and safety-gap enforcement) may be used.
        ref: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane_change_mode_0xb6'''
        self.type = traci.vehicle.getTypeID(self.ID)
        self.max_speed = traci.vehicle.getMaxSpeed(self.ID)

    def getSpd(self):
        '''
        :return: vehicle speed in m/sec over last step. This is the speed that will be continued with if no intereference
        occurs from setSpeed or slowDown functions.
        '''
        self.spd = traci.vehicle.getSpeed(self.ID)

    def getRoute(self):
        '''
        :return:  route of the car. Either r1 or r2 currently.
        '''
        try:
            self.route = int(traci.vehicle.getRoadID(self.ID)[1])
        except:
            pass #EDIT #Is this useful? I think it's a remenant. #Waleed
            #print("Warning,current route status: "+traci.vehicle.getRoadID(self.ID) )

    def getPose(self):
        '''
        :return: return the position of the vehicle's front tip in the lane (lane: 0,1 currently).
        Accounts for different routes.
        '''
        self.lane_pose = traci.vehicle.getLanePosition(self.ID)
        if(self.route > self.base_route):
            self.lane_pose +=  self.length_of_base_route

    def getAcc(self):
        '''
        :return: Returns the acceleration in m/s^2 of the named vehicle within the last step.
        '''
        self.accel = traci.vehicle.getAcceleration(self.ID)

    def getL(self):
        '''
        :return: None, but sets index of the lane in which the vehicle resides.
        '''
        self.lane = traci.vehicle.getLaneIndex(self.ID)

    def chL(self,L):
        '''
        :function: pefroms the lane change action
        :param L: Index of Lane to change lane to.
        :return:  None, but sets the ane changed to, and pefroms the lane change action
        '''
        traci.vehicle.changeLane(self.ID,L, SimTime)
        '''
        Forces a lane change to the lane with the given index; if successful,the lane will be chosen for the given amount of time (in s).
        SimtTime to keep the lane change till that the end of the simulation.
        '''
        self.lane = traci.vehicle.getLaneIndex(self.ID) #Force lane update right after to avoid lagging in information

    def acc(self,spd,t):
        '''
        :param spd: speed to reach after time (t)
        :param t: time after which to reach speed (spd)
        :return: None
        '''
        traci.vehicle.slowDown(self.ID,spd,t)

    def revertSpd(self):
        '''
        Sets speed instantaneously
        :return:
        '''
        traci.vehicle.setSpeed(self.ID,-1)

    def inst_acc(self, acc):
        ''' accelerate instantaneously'''
        self.getSpd() #get current speed
        traci.vehicle.setSpeed(self.ID, np.min(int(self.spd  + acc), int(self.max_speed)))



#---------------------------------------------------------#

def getters(vs):
    for v in vs:
        v.getSpd()
        v.getRoute() #Note: getRoute must always be called before getPose
        v.getPose()
        v.getAcc()
        v.getL()

def defGlobals():

    global SimTime,LH,RB,track_len,fast,slow,Q,speed_range
    speed_range = np.arange(0,30,5)
    fast = 0
    slow = 1
    track_len = 4000
    LH = Vehicle("LH")
    RB = Vehicle("RB")
    SimTime = 1000
    #Q = np.zeros((7,4))
    Q_ = np.zeros((6,3,11,3,58,5))


def run():
    step = 0

    LH.chL(0)
    traci.vehicle.setSpeed(LH.ID,3)
    #print("Hello world, I'm an AV, and I love carrots")
    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()

        vehicles_list = [LH, RB]
        if step>2:
            getters(vehicles_list)
            print("emer speed: ",LH.spd)
            print("agent speed: ",RB.spd)
            if step >=10:
                print("Trying to set emer speed to 20")
                traci.vehicle.setSpeed(LH.ID,20)
                if LH.spd<=3:
                    print('Negative, no response from the vehicle')
                elif LH.spd>3 and LH.spd<=11:
                    print('emer increased speed reaching the agent')
                elif LH.spd>11:
                    print('Positive, emer is on 20 m/s')



        step += 1

        """
        if step ==10:
            LH.chL(fast)

            Proudhon = Anarchia(LH,RB)
            Proudhon.pickAction()
            Proudhon.takeAction()
            Proudhon.memory()

        elif step%10==0 and step>10:

            Proudhon.evaluate()
            Proudhon.pickAction()
            Proudhon.takeAction()
            Proudhon.memory()


            #print("time ",getArrivTime(LH,RB))
        if step%1000==0 and step>=1000:
            print(step)
            print("Arrive time ", getArrivTime(LH,RB))
            print(Q)
            Q_Table = pd.DataFrame({'Change Lane': Q[:, 0], 'Accelerate': Q[:, 1],'Decelerate': Q[:, 2],'Do no thing': Q[:, 3]})

            Q_Table.to_csv('Results/Q_Table_step='+str(step)+'.csv', index=False)
        if step ==SimTime:
            break


        """



    traci.close()
    sys.stdout.flush()


# main entry point
if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "Anarcho1.2.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    defGlobals()
    defGlobals()
    run()
