#!/usr/bin/env python
import os
import sys
import optparse
from math import sqrt, ceil
from random import randrange
import numpy as np
import warnings; do_warn = False
import random

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
        self.max_accel = traci.vehicle.getAccel(self.ID)
        self.max_decel = traci.vehicle.getDecel(self.ID)

    def getSpd(self): #ROS
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

    def getPose(self): #ROS
        #TODO: Change this function to depend on  getDistanceRoad(self, edgeID1, pos1, edgeID2, pos2, isDriving=False)
        '''
        :return: return the position of the vehicle's front tip in the lane (lane: 0,1 currently).
        Accounts for different routes.
        '''
        self.lane_pose = traci.vehicle.getLanePosition(self.ID)
        if(self.route > self.base_route):
            self.lane_pose +=  self.length_of_base_route

    def getAcc(self): #ROS
        '''
        :return: Returns the acceleration in m/s^2 of the named vehicle within the last step.
        '''
        self.accel = traci.vehicle.getAcceleration(self.ID)

    def getL(self): #ROS
        '''
        :return: None, but sets index of the lane in which the vehicle resides.
        '''
        self.lane = traci.vehicle.getLaneIndex(self.ID)

    def chL(self,L): #ROS
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

    def inst_acc(self, acc): #ROS
        ''' accelerate instantaneously'''
        self.getSpd() #get current speed
        traci.vehicle.setSpeed(self.ID, max( 0.0 , min(int(self.spd  + acc), int(self.max_speed))))
        #minimum velocity: 0.0 -> Handled by Code, not SUMO. Sumo ignores command if velocity is negative.
        #maximum velocity: self.max_speed -> Handled by SUMO, so code here is redundant.



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
    global SimTime,LH,RB,track_len,fast,slow,speed_range
    speed_range = np.arange(0,30,5)
    fast = 0
    slow = 1
    track_len = 500
    LH = Vehicle("LH")
    RB = Vehicle("RB")
    SimTime = 1000
    #Q = np.zeros((7,4))

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
    Access order for the Q_table is [agent_vel][agent_lane][amb_vel][amb_lane][rel_amb_y][action].. remember to change the value rel_amb_y to be positive [0,58]
    '''

    def __init__(self, environment, name="Q-learning", algo_params = dict()):
        '''
        #Usage is: One RLAlgorithm object per training agent
        :param environment:  of class env, contains list of vehicles and observations.
        :param name:         string, currently not used except for display purposes
        '''
        self.name = name
        self.environment = environment

        self.QActions = self.environment.Actions
        self.action_to_string_dict = self.environment.action_to_string_dict


        #algo_params:
        if(name == "Q-learning"): #default case for now
            self.q_table = np.zeros((6, 3, 11, 3, 58, 5))
            self.exp_exp_tradeoff = algo_params['exp_exp_tradeoff']
            self.epsilon = algo_params['epsilon']
            self.gamma = algo_params['gamma']
            self.learning_rate = algo_params['learning_rate']
            self.max_epsilon = algo_params['max_epsilon']
            self.min_epsilon = algo_params['min_epsilon']
            self.decay_rate = algo_params['decay_rate']

    def pickAction(self):
        self.Action = self.QActions[randrange(len(self.QActions))]
        '''
        Edit as follows:
        ## First we randomize a number
        exp_exp_tradeoff = random.uniform(0,1)
        
        ## If this number > greater than epsilon --> exploitation (taking the biggest Q value for this state)
        if exp_exp_tradeoff > epsilon:
            action = np.argmax(qtable[state,:])
        
        # Else doing a random choice --> exploration
        else:
            action = env.action_space.sample()
            
        link: https://github.com/simoninithomas/Deep_reinforcement_learning_Course/blob/master/Q%20learning/Taxi-v2/Q%20Learning%20with%20OpenAI%20Taxi-v2%20video%20version.ipynb
        '''
        return self.Action

    def update_q_table(self, chosen_action, reward, new_observed_state_for_this_agent,last_observed_state_for_this_agent, feasible_actions_for_chosen_action,
                       rel_amb_y_min = -41, rel_amb_y_max = 16):
        '''

        :param chosen_action:
        :param observed_state:
        :return:

        :Notes:
        #Assumed Hierarchy of observed_state_for_this_agent:[agent_vel , agent_lane , amb_vel , amb_lane , rel_amb_y]

        '''

        #OLD STATE VARIABLES:
        agent_vel = last_observed_state_for_this_agent[0]
        agent_vel_index = int(np.round(agent_vel)) #[0,1,2,3,4,5]

        agent_lane_index = last_observed_state_for_this_agent[1]

        amb_vel = last_observed_state_for_this_agent[2]
        amb_vel_index = int(np.round(amb_vel)) #[0,1,2,3,4,5,6,7,8,9,10]

        amb_lane_index = last_observed_state_for_this_agent[3]

        rel_amb_y = np.clip(last_observed_state_for_this_agent[4], rel_amb_y_min, rel_amb_y_max)  # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
        rel_amb_y_index = int(np.round(rel_amb_y) + abs(rel_amb_y_min) )

        action_index = self.action_to_string_dict[chosen_action]



        #NEW STATE VARIABLES:
        new_agent_vel = new_observed_state_for_this_agent[0]
        new_agent_vel_index = int(np.round(new_agent_vel))  # [0,1,2,3,4,5]

        new_agent_lane_index = new_observed_state_for_this_agent[1]

        new_amb_vel = new_observed_state_for_this_agent[2]
        new_amb_vel_index = int(np.round(new_amb_vel))  # [0,1,2,3,4,5,6,7,8,9,10]

        new_amb_lane_index = new_observed_state_for_this_agent[3]

        new_rel_amb_y = np.clip(new_observed_state_for_this_agent[4], rel_amb_y_min,
                            rel_amb_y_max)  # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
        new_rel_amb_y_index = int(np.round(new_rel_amb_y) + abs(rel_amb_y_min))


        #Feasible Action Indices:
        feasible_action_indices = []
        for act in feasible_actions_for_chosen_action:
            feasible_action_indices.append( self.action_to_string_dict[act] )



        # Update Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
        q_of_s_a_value = self.q_table[agent_vel_index][agent_lane_index][amb_vel_index][amb_lane_index][rel_amb_y_index][action_index]
        max_q_of_s_value_new= np.max(self.q_table[new_agent_vel_index, new_agent_lane_index, new_amb_vel_index, new_amb_lane_index, new_rel_amb_y_index, feasible_action_indices])
        q_of_s_a_value = q_of_s_a_value + self.learning_rate * (reward + self.gamma * max_q_of_s_value_new - q_of_s_a_value)
        #actual update step:
        self.q_table[agent_vel_index][agent_lane_index][amb_vel_index][amb_lane_index][rel_amb_y_index][action_index] = q_of_s_a_value


    # def takeAction(self):
    #     if self.Action=="change_left":
    #         self.agent.chL(slow)
    #     elif self.Action=="change_right":
    #         self.agent.chL(fast)
    #     elif self.Action=="acc":
    #         self.agent.acc(40,10)
    #     elif self.Action=="dec":
    #         self.agent.acc(4,10)
    #     elif self.Action=="no_acc":
    #         pass
    #     else:
    #         raise("Error: Action not recognized. ")

    # def memory(self):
    #     self.initial_time = getArrivTime(self.emer,self.agent)

    # def evaluate(self):
    #     ArrivTime = getArrivTime(self.emer,self.agent)
    #     if self.emer.lane == self.agent.lane:
    #         if ArrivTime<=speed_range[-1]:
    #             for i in range(1,len(speed_range)):
    #                 if speed_range[i-1]<=ArrivTime<speed_range[i]:
    #                     state = i-1
    #             if ArrivTime>self.initial_time:
    #                 self.reward = 5
    #             if ArrivTime<self.initial_time:
    #                 self.reward = -5
    #             Q[state,self.action_to_string_dict[self.Action]] += self.reward
    #             print("state ",state)
    #         else:
    #             self.reward = 1
    #             Q[5,self.action_to_string_dict[self.Action]] += self.reward
    #     else:
    #         self.reward = -1
    #         Q[6,self.action_to_string_dict[self.Action]] += self.reward


class env():

    def __init__(self, list_of_vehicles, name="SingleAgentEvn0.1",  ambulance_goal_distance=500):

        self.name = name
        self.list_of_vehicles = list_of_vehicles
        self.amb_goal_dist = ambulance_goal_distance

        self.agents = [] #Stays as is in multiagent
        self.emer = None #Stays as is in multiagent

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

        # Before exiting, check:
        if (self.emer is None):
            raise ValueError(
                'WaleedError: Failed to initialize environment. No emergency vehicles were used in the evnironment.'
            )

        self.optimal_time = int(np.round(track_len/self.emer.max_speed))  #Optimal Number of time steps: number of time steps taken by ambulance at maximum speed
        self.max_steps = 20*self.optimal_time

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
                          range(len(self.list_of_vehicles) - 1)]  # Currently contains data for one ego vehicle only.
        # Shall contain all vehicles' data in the future
        # minus 1 since one vehicle is an emergency vehicle
        hidden_state = [0.0 for i in
                        range(len(self.list_of_vehicles))]  # Entry for each vehicle (review :hidden_state: up)

        for vhcl in self.list_of_vehicles:

            if (vhcl.type == "Emergency"): #Assumese that the ambulance is the first vehicle in the list of vehicles
                # ------------------------ checks - start -----------------------#
                count_emergency_vehicles += 1
                if (count_emergency_vehicles > 1):
                    raise ValueError(
                        f'WaleedError: Number of emergency vehicles = {count_emergency_vehicles}. Only a single emrgency vehicle is expected to be present.\n'
                        f'Please edit your code in Anarchia.observe() if you changed this in your problem formulation.')
                # ------------------------ checks - end ------------------------#
                amb_vel = vhcl.spd
                amb_lane = vhcl.lane
                amb_abs_y = vhcl.lane_pose

            elif (vhcl.type == "Ego"):
                agent_vel = vhcl.spd
                agent_lane = vhcl.lane
                agent_abs_y = vhcl.lane_pose
                rel_amb_y = amb_abs_y - agent_abs_y  # can be negative and that's ok. Will handle that during indexing.
                # Assumes that the ambulance is the first vehicle in the list of vehicles

                observed_state[count_ego_vehicles] = [agent_vel, agent_lane, amb_vel, amb_lane, rel_amb_y]
                hidden_state[count_ego_vehicles] = agent_abs_y
                count_ego_vehicles += 1

            else:
                raise ValueError(
                    f'WaleedError: Value of the vehicle type {vhcl.type} is not recognized as neither "Emergency" nor "Ego".'
                    f'\n Please only choose one of those in your route XML file or edit the observe function inside env class to'
                    f'accommodate your new vehicle type.')

        hidden_state[-1] = amb_abs_y


        self.hidden_state = hidden_state
        self.observed_state = observed_state
        self.full_state = [observed_state, hidden_state]

    def are_we_done(self, full_state, step_number):
        #full_state: currently not used since we have the ambulance object.

        amb_abs_y = self.full_state[-1][-1] #Please refer to shape of full_state list in env.measure_full_state()


        #1: steps == max_steps-1
        if(step_number == self.max_steps-1):
            return 1
        #2: goal reached
        elif(amb_abs_y > self.amb_goal_dist - self.emer.max_speed-1 ):
            # TODO: Change NET file to have total distance = 511. Then we can have the condition to compare with 500 directly.
            return 2 #GOAL IS NOW 500-10-1 = 489 cells ahead. To avoid ambulance car eacaping

        for agent_index in range( self.count_ego_vehicles ):
            agent_abs_y = self.full_state[-1][agent_index] # #Please refer to shape of full_state list in env.measure_full_state
                                                            # hidden_state shape: [ agent_abs_y ... for vehicle in vehicles , amb_abs_y]
            if (agent_abs_y > self.amb_goal_dist - self.emer.max_speed - 1):
                return 3



        # 0: not done
        return 0

    def get_vehicle_object_by_id(self, vehID):
        '''
        :param vehID: string, ID of the vehicle to get the Vehicle object for
        :return: Vehicle, object that has the ID equal to vehID
        '''
        for vhc in self.list_of_vehicles:
            if vhc.ID == vehID :
                return vhc
        return None #If vehID does not belong to any vehicle, None value is returned

    def get_follow_speed_by_id(self, vehID):

        agent = self.get_vehicle_object_by_id(vehID)
        leader_data = traci.vehicle.getLeader(LH.ID)

        if(leader_data is None): #If no leader found (i.e. there are no leading vehicles), return max_speed for vehicle with vehID
            return agent.max_speed
        #else:
        leader_id, distance_to_leader = leader_data #distance to leader (in our terms) = gap - leader_length - my_minGap
        leading_vehicle = self.get_vehicle_object_by_id(leader_id)

        follow_speed = traci.vehicle.getFollowSpeed(agent.ID, agent.max_speed, distance_to_leader, leading_vehicle.spd, agent.max_decel, leading_vehicle.ID)

        return follow_speed

    def get_feasible_actions(self, agent):
        '''
        :param agent: Vehicle object, vehicle for which we want to check the feasible actions. The set of possible actions
                        is assumed to be: ["change_left", "change_right", "acc", "dec", "no_acc"] (hard-coded)
        :return feasible_actions: list of strings, representing actions that are feasible to take in this step to apply over the
                next step.
        '''

        feasible_actions = self.Actions.copy()  # Initially, before checking

        left = 1
        right = -1
        change_left_possible = traci.vehicle.couldChangeLane(agent.ID, left, state=None)
        change_right_possible = traci.vehicle.couldChangeLane(agent.ID, right, state=None)
        decelrate_possible = True  # Always true because of how SUMO depends on the Car Follower model and thus avoids maximum decleration hits.
        # Review TestBench Test Case 06 results for more info.
        accelerate_possible = min(agent.max_speed, agent.spd + agent.max_accel/2) <= self.get_follow_speed_by_id(
            agent.ID)  # NOTE: Checks if maximum acceleration/2 is possible
        # Do Nothing should be always feasible since SUMO/Autonomous functionality will not allow vehicles to crash.

        if (agent.max_accel > 1.0 and do_warn): warnings.warn(
            f"Please note that agent {agent.ID} has maximum acceleration > 1.0. "
            f"Function env.get_feasible_actions(agent) checks if self.spd+self.max_accel is feasible.")

        '''
        couldChangeLane: Return bool indicating whether the vehicle could change lanes in the specified direction 
        (right: -1, left: 1. sublane-change within current lane: 0).
        #Check function here https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html 
        #NOTE: getLaneChangeState return much more details about who blocked, if blocking .. etc. 
            #Details: https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html#change_lane_information_0x13
        '''
        if (change_left_possible):
            #debug#print(f'Agent {agent.ID} can change lane to LEFT lane.')
            pass
        else:
            feasible_actions.remove("change_left")
            #debug#print(f'Agent {agent.ID} //CAN NOT// change lane to LEFT lane.')

        if (change_right_possible):
            #debug#print(f'Agent {agent.ID} can change lane to RIGHT lane.')
            pass
        else:
            feasible_actions.remove("change_right")
            #debug#print(f'Agent {agent.ID} //CAN NOT// change lane to RIGHT lane.')

        if (accelerate_possible):
            #debug#print(f'Agent {agent.ID} can ACCELERATE.. next expected velocity = {self.get_follow_speed_by_id(agent.ID)}')
            pass
        else:
            feasible_actions.remove("acc")
            #debug#print(f'Agent {agent.ID} can NOT ACCELERATE.. next expected velocity = {self.get_follow_speed_by_id(agent.ID)}')

        #debug#print(f'Feasible actions: ', feasible_actions)
        return feasible_actions

    def calc_reward(self, amb_last_velocity, done, number_of_steps, max_final_reward = 20, min_final_reward = -20, max_step_reward=0, min_step_reward = -1.25):
        #TODO: Fix reward logic to be this_step -> next_step (as opposed to prev_step -> this_step)
        #TODO: Fix final reward logic according last discussiion : if agent finishes first, assume the ambulance  will conitnue at its current
        #   velocity till the end.
        '''
        :logic: Calculate reward to agent from current state
        :param amb_last_velocity: float, previous velocity the ambulance (self.emer) had
        :param done: bool, whether this is the last step in the simulation or not (whether to calculate final reward or step rewrard)
        :param number_of_steps: number of steps in simulation so far. Used to calculate final reward but not step reward
        :param max_final_reward: reward for achieving end of simulation (done) with number_of_steps = self.optimal time
        :param min_final_reward: reward for achieving end of simulation (done) with number_of_steps = 20 * self.optimal time
        :param max_step_reward: reward for having an acceleration of value = self.emer.max_accel (=2) over last step
        :param min_step_reward: reward for having an acceleration of value = - self.emer.max_accel (=- 2) over last step
        :return: reward (either step reward or final reward)


        :Notes:
        #Simulation Time is not allowed to continue after 20*optimal_time (20* time steps with ambulance at its maximum speed)
        '''

        if(done): #Calculate a final reward
            #Linear reward. y= mx +c. y: reward, x: ration between time achieved and optimal time. m: slope. c: y-intercept
            m = ( (max_final_reward - min_final_reward) *20 ) /19 #Slope for straight line equation to calculate final reward
            c = max_final_reward - 1*m #c is y-intercept for the reward function equation #max_final_reward is the y for x = 1
            reward = m * (self.optimal_time/number_of_steps) + c
            #debug#print(f'c: {c}, m: {m}, steps: {number_of_steps}, optimal_time: {self.optimal_time}')
            return reward

        else: #Calcualate a step reward
            steps_needed_to_halt = 30
            ration_of_halt_steps_to_total_steps = steps_needed_to_halt/track_len
            m = (max_step_reward - min_step_reward)/(2 * self.emer.max_accel)  # Slope for straight line equation to calculate step reward
            #2 * self.emer.max_accel since: = self.emer.max_accel - * self.emer.max_decel
            c = max_step_reward - self.emer.max_accel * m  # c is y-intercept for the reward function equation #max_step_reward is the y for x = 2 (max acceleration)
            reward = m * (self.emer.spd - amb_last_velocity) + c
            #debug#print(f'c: {c}, m: {m}, accel: {(self.emer.spd - amb_last_velocity)}')

            if (abs(self.emer.spd - amb_last_velocity) <= 1e-10 and abs(amb_last_velocity-self.emer.max_speed) <= 1e-10):
            #since ambulance had maximum speed and speed did not change that much; unless we applied the code below.. the acceleration
            #   will be wrongly assumed to be zero. Although the ambulance probably could have accelerated more, but this is its maximum velocity.
                reward = max_step_reward #same reward as maximum acceleration (+2),
            return reward



def run():
    step = 0
    episode = 1 #TODO: Repeat over many episodes
    done = False


    q_learning_params = dict()
    q_learning_params['exp_exp_tradeoff'] = random.uniform(0, 1)
    q_learning_params['learning_rate'] = 0.7 # Learning rate
    q_learning_params['gamma'] = 0.618 # Discounting rate
    # Exploration parameters
    q_learning_params['epsilon'] = 1.0  # Exploration rate
    q_learning_params['max_epsilon'] = 1.0  # Exploration probability at start
    q_learning_params['min_epsilon'] = 0.01  # Minimum exploration probability
    q_learning_params['decay_rate'] = 0.01  # Exponential decay rate for exploration prob




    vehicles_list = [LH, RB]
    Proudhon = env(vehicles_list)  # [LH, RB]
    RB_RLAlgorithm = RLAlgorithm(Proudhon, algo_params= q_learning_params)

    traci.simulationStep()  # apply_action
    getters(vehicles_list)
    Proudhon.measure_full_state()

    while traci.simulation.getMinExpectedNumber() > 0:

        amb_last_velocity = Proudhon.emer.spd
        last_observed_state = Proudhon.observed_state #For rewarding purposes

        exp_exp_tradeoff = random.uniform(0, 1)
        traci.simulationStep() #apply_action
        step += 1

        getters(vehicles_list)
        Proudhon.measure_full_state()

        feasible_actions_for_chosen_action = Proudhon.get_feasible_actions(RB)
        Proudhon.get_feasible_actions(LH)

        chosen_action = RB_RLAlgorithm.pickAction()
        reward = Proudhon.calc_reward(amb_last_velocity, done, step)
        new_observed_state_for_this_agent = Proudhon.observed_state[0]
        last_observed_state_for_this_agent = last_observed_state[0]

        RB_RLAlgorithm.exp_exp_tradeoff = exp_exp_tradeoff

        RB_RLAlgorithm.update_q_table(chosen_action, reward, new_observed_state_for_this_agent,last_observed_state_for_this_agent, feasible_actions_for_chosen_action,
                       rel_amb_y_min = -41, rel_amb_y_max = 16)

        RB_RLAlgorithm.epsilon = RB_RLAlgorithm.min_epsilon + (RB_RLAlgorithm.max_epsilon - RB_RLAlgorithm.min_epsilon) * \
                                 np.exp(-RB_RLAlgorithm.decay_rate * episode)



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
if __name__ == "__main__":
    options = get_options()


    #no-gui is default now:
    # check binary
    #sumoBinary = checkBinary('sumo') #Uncomment this to have headless mode on
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
