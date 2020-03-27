import numpy as np
from Config import *
import random
import traci
import warnings; do_warn = False
import jinja2

class env():

    #TODO: Add env.reset(): ||:Changes .rou file to start: from different lane. ||:Changes network file to start from: different distance.

    def __init__(self, list_of_vehicles, name="SingleAgentEvn1.0",  ambulance_goal_distance=500 , rel_amb_y_min = -41, rel_amb_y_max = 16):

        self.name = name
        self.list_of_vehicles = list_of_vehicles
        self.amb_goal_dist = ambulance_goal_distance
        self.reward = 0.0
        self.emer_start_lane = None

        self.rel_amb_y_min = rel_amb_y_min
        self.rel_amb_y_max = rel_amb_y_max

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

    def reset(self):
        '''
        :function: Resets variables necessary to start next training episode, and reloads randomly initialized next episode XML files.
        :return: None, but resets environment

        :Notes:
        * Commented lines are tautologies (do not add new info), kept only for reference to what is inherited from last
        episode run and from initialization.

        :sources: https://www.eclipse.org/lists/sumo-user/msg03016.html (how to reset SUMO environent from code)
        '''

        # ------------------------------------------------------------------- #
        # 1 :        R E S E T       O L D       V A R I A B L E S
        # ------------------------------------------------------------------- #

        # self.name = self.name
        # self.list_of_vehicles = self.list_of_vehicles
        # self.amb_goal_dist = self.amb_goal_dist
        self.reward = 0.0
        self.emer_start_lane = None

        #self.rel_amb_y_min = self.rel_amb_y_min
        # self.rel_amb_y_max = self.rel_amb_y_max

        # self.agents = self.agents
        # self.emer = self.emer

        self.hidden_state = None
        self.observed_state = None
        self.full_state = None

        # self.count_emergency_vehicles = self.count_emergency_vehicles
        # self.count_ego_vehicles = self.count_ego_vehicles

        # self.Actions = self.Actions
        # self.action_to_string_dict = self.action_to_string_dict

        # Calculation for optimal time is kept in case the track_len is changed between episodes
        self.optimal_time = int(np.round(
            track_len / self.emer.max_speed))  # Optimal Number of time steps: number of time steps taken by ambulance at maximum speed
        self.max_steps = 20 * self.optimal_time

        # ---------------------------------------------------------------------------- #
        # 2 :        R A N D O M L Y      I N I T I A L I Z E       X M L s
        # ---------------------------------------------------------------------------- #
        new_stance = dict()
        # 2.1: Init Agent Start Lane
        new_stance['agent_start_lane'] = random.randint(0, 2)  # TODO: edit this to automatically retrieve the number of lanes

        # 2.2: Init Emergency Start Lane
        new_stance['ambulance_start_lane'] = random.randint(0, 2)  # TODO: edit this to automatically retrieve the number of lanes

        # 2.3: Init Agent Start Position
        new_stance['agent_start_position'] = random.randint(41, 250)  # TODO: Edit minimum position to depend on starting position
        new_stance['r1_new_length'] = new_stance['agent_start_position']
        new_stance['r2_new_length'] = 511 - new_stance['agent_start_position']

        # 2.4: Load Template to template
        templateLoader = jinja2.FileSystemLoader(searchpath = TEMPLATES_PATH)
        templateEnv = jinja2.Environment(loader=templateLoader)

        # 2.5: Put rou template to file
        rou_template = templateEnv.get_template("route_template.xml")
        with open(ROUTE_FILE_PATH, "w") as fp:
            fp.writelines(rou_template.render(data=new_stance))

        # 2.6: Put net template to file:
        net_template = templateEnv.get_template("net_template.xml")
        with open(NET_FILE_PATH, "w") as fp:
            fp.writelines(net_template.render(data=new_stance))


    def get_emer_start_lane(self):
        self.emer_start_lane = self.emer.getL()

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
        # TODO: Remove the below loop according to RB_comment. Do this in version 1.4 to avoid putting effort into it now
        #  since it is not a fault
        '''
        RB_ comment:
        repeated assignment, why don't use count_emergency_vehicles, count_ego_vehicles
        already existing in self?
        '''
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
                #RB_ comment, why isn't rel_amb_y the distance between agent and amb instead of amb to agent?
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
            # DONE: Change NET file to have total distance = 511. Then we can have the condition to compare with 500 directly.
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
        leader_data = traci.vehicle.getLeader(self.emer.ID)

        if(leader_data is None): #If no leader found (i.e. there are no leading vehicles), return max_speed for vehicle with vehID
            return agent.max_speed
        # else:
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
        accelerate_possible = min(agent.max_speed, agent.spd + agent.max_accel / 2) <= self.get_follow_speed_by_id(
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

        # debug#print(f'Feasible actions: ', feasible_actions)
        return feasible_actions

    def calc_reward(self, amb_last_velocity, done, number_of_steps, max_final_reward = 20, min_final_reward = -20, max_step_reward=0, min_step_reward = -1.25):
        # TODO: Fix reward logic to be this_step -> next_step (as opposed to prev_step -> this_step)
        # TODO: Fix final reward logic according last discussiion : if agent finishes first, assume the ambulance  will conitnue at its current
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

        if(done and give_final_reward): #Calculate a final reward
            #Linear reward. y= mx +c. y: reward, x: ration between time achieved and optimal time. m: slope. c: y-intercept
            m = ( (max_final_reward - min_final_reward) *20 ) /19 #Slope for straight line equation to calculate final reward
            c = max_final_reward - 1*m #c is y-intercept for the reward function equation #max_final_reward is the y for x = 1
            reward = m * (self.optimal_time/number_of_steps) + c
            #debug#print(f'c: {c}, m: {m}, steps: {number_of_steps}, optimal_time: {self.optimal_time}')
            self.reward = reward
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

            self.reward = reward
            return reward

    def __str__(self):  #:return: environment string representation
        return str(
            {"name": self.name, "reward": self.reward, "Vehicles": [vhc.ID for vhc in self.list_of_vehicles], "full_state": self.full_state})
