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
        #traci.vehicle.setSpeed(self.ID, max( 0.0 , min(int(self.spd  + acc), int(self.max_speed))))
        traci.vehicle.setSpeed(self.ID, self.spd + acc)



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
    global SimTime,A,track_len,fast,slow,Q,speed_range
    speed_range = np.arange(0,30,5)
    fast = 0
    slow = 1
    track_len = 4000
    A = Vehicle("A")
    SimTime = 1000
    #Q = np.zeros((7,4))
    Q_ = np.zeros((6,3,11,3,58,5))

def CalcDist(emer,agent):
    #edit
    if emer.route>agent.route:
        dist = track_len-(emer.pose-agent.pose)
    elif emer.route<agent.route:
        dist = agent.pose-emer.pose
    else:
        if emer.pose>agent.pose:
            dist = track_len-(emer.pose-agent.pose)
        else:
            dist = agent.pose-emer.pose
    return dist

#timeOfArrival = quadSolv(0.5*LHacc,LHsp,-dist)

class RLAlgorithm():
    '''
    Access order for the Q_table is [agent_vel][agent_lane][amb_vel][amb_lane][rel_amb_y][action]
    '''

    def __init__(self, environment, name="Q-learning"):
        '''
        :param environment:  of class env, contains list of vehicles and observations.
        :param name:         string, currently not used except for display purposes
        '''
        self.name = name
        self.environment = environment

        self.QActions = self.environment.Actions
        self.action_to_string_dict = self.environment.action_to_string_dict

    def pickAction(self):
        self.Action = self.QActions[randrange(len(self.QActions))]

    def takeAction(self):
        if self.Action=="change_left":
            self.agent.chL(slow)
        elif self.Action=="change_right":
            self.agent.chL(fast)
        elif self.Action=="acc":
            self.agent.acc(40,10)
        elif self.Action=="dec":
            self.agent.acc(4,10)
        elif self.Action=="no_acc":
            pass
        else:
            raise("Error: Action not recognized. ")

    # def memory(self):
    #     self.initial_time = getArrivTime(self.emer,self.agent)

    def evaluate(self):
        ArrivTime = getArrivTime(self.emer,self.agent)
        if self.emer.lane == self.agent.lane:
            if ArrivTime<=speed_range[-1]:
                for i in range(1,len(speed_range)):
                    if speed_range[i-1]<=ArrivTime<speed_range[i]:
                        state = i-1
                if ArrivTime>self.initial_time:
                    self.reward = 5
                if ArrivTime<self.initial_time:
                    self.reward = -5
                Q[state,self.action_to_string_dict[self.Action]] += self.reward
                print("state ",state)
            else:
                self.reward = 1
                Q[5,self.action_to_string_dict[self.Action]] += self.reward
        else:
            self.reward = -1
            Q[6,self.action_to_string_dict[self.Action]] += self.reward

class env():

    def __init__(self, list_of_vehicles, name="SingleAgentEvn0.1"):

        self.name = name
        self.list_of_vehicles = list_of_vehicles

        self.agents = []
        self.emer = None

        self.hidden_state = None
        self.observed_state = None
        self.full_state = None

        self.count_emergency_vehicles = 0
        self.count_ego_vehicles = 0

        self.Actions = ["change_left", "change_right", "acc", "dec", "no_acc"]
        self.action_to_string_dict = {
            "change_left": 0,
            "change_right": 1,
            "acc": 2,
            "no_acc": 3,
            "dec": 4
        }  # Must maintain order in Actions



        for vhcl in self.list_of_vehicles:

            if (vhcl.type == "Emergency"):
                # ------------------------ checks - start -----------------------#
                self.count_emergency_vehicles += 1
                if (self.count_emergency_vehicles > 1):
                    raise ValueError(
                        f'WaleedError: Number of emergency vehicles = {self.count_emergency_vehicles}. Only a single emrgency vehicle is expected to be present.\n'
                        f'Please edit your code in Anarchia.observe() if you changed this in your problem formulation.'
                    )
                # ------------------------ checks - end ------------------------#
                self.emer = vhcl  # current vehicle is the emergency vehicle

            elif (vhcl.type == "Ego"):
                self.agents.append(vhcl)
                self.count_ego_vehicles += 1

            else:
                raise ValueError(
                    f'WaleedError: Value of the vehicle type {vhcl.type} is not recognized as neither "Emergency" nor "Ego".'
                    f'\n Please only choose one of those in your route XML file or edit the __init__ function inside env class to'
                    f'accommodate your new vehicle type.')



    def measure_full_state(self):

        '''
        :function: assign hidden and observed states.
        :param list_of_vehicles: list of objects of the type Vehicle, of which one ONLY is ambulance.
        :return: None. But assigns values below :
            :observed_state: [ [agent_vel , agent_lane , amb_vel , amb_lane , rel_amb_y] .. for vehicle in vehicles ]
            :hidden_state:   [ agent_abs_y ... for vehicle in vehicles , amb_abs_y] ---> note the amb_abs_y at the end
            :full_state:     [ observed_state, hidden_state ]
        '''

        count_emergency_vehicles = 0  # Must be 1 only !
        count_ego_vehicles = 0  # Currently 1 only
        observed_state = [[] for i in
                          range(len(self.list_of_vehicles))]  # Currently contains data for one ego vehicle only.
        # Shall contain all vehicles' data in the future
        # minus 1 since one vehicle is an emergency vehicle


        for vhcl in self.list_of_vehicles:


            agent_vel = vhcl.spd
            agent_lane = vhcl.lane
            agent_abs_y = vhcl.lane_pose
            #agent_abs_y

            observed_state[count_ego_vehicles] = [agent_vel, agent_lane, agent_abs_y]
            #hidden_state[count_ego_vehicles] = agent_abs_y
            count_ego_vehicles += 1




        #self.hidden_state = hidden_state
        self.observed_state = observed_state
        #self.full_state = [observed_state, hidden_state]

    def get_feasible_actions(self, agent):

        feasible_actions = self.Actions.copy()  # Initially, before checking

        left = 1
        right = -1
        change_left_possible  = traci.vehicle.couldChangeLane(agent.ID, left, state=None)
        change_right_possible = traci.vehicle.couldChangeLane(agent.ID, right, state=None)

        #TODO:
        #Acceleration and deceleration check.
        #Do Nothing should be always feasible since SUMO/Autonomous functionality will not allow vehicles to crash.



        '''
        couldChangeLane: Return bool indicating whether the vehicle could change lanes in the specified direction 
        (right: -1, left: 1. sublane-change within current lane: 0).
        #Check function here https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html 
        #NOTE: getLaneChangeState return much more details about who blocked, if blocking .. etc. 
            #Details: https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html#change_lane_information_0x13
        '''
        if(change_left_possible):
            print(f'Agent {agent.ID} can change lane to LEFT lane.')
        else:
            del feasible_actions[self.action_to_string_dict["change_left"]]
            #TODO: DO NOT INDEX .. will produce error if more than one action is removed
            print(f'Agent {agent.ID} //CAN NOT// change lane to LEFT lane.')

        if (change_right_possible):
            print(f'Agent {agent.ID} can change lane to RIGHT lane.')
        else:
            del feasible_actions[self.action_to_string_dict["change_right"]] #TODO: DO NOT INDEX .. will produce error if more than one action is removedd
            print(f'Agent {agent.ID} //CAN NOT// change lane to RIGHT lane.')


        print(f'Feasible actions: ', feasible_actions)




def run():
    step = 0

    A.chL(0)

    #print("Hello world, I'm an AV, and I love carrots")
    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()

        vehicles_list = [A]

        getters(vehicles_list)


        Proudhon = env(vehicles_list) #[LH, RB]
        Proudhon.measure_full_state()
        Proudhon.get_feasible_actions(A)

        print(step,' : ',' [agent_vel, agent_lane, agent_abs_y] ')
        print(step,' :  ',Proudhon.observed_state)
        step += 1

        if(step> -1):
            acc = 3.0
            A.inst_acc(acc)
            print(f'STEP {step}:-------------: Negative acceleration applied.')




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
    #defGlobals()
    run()
