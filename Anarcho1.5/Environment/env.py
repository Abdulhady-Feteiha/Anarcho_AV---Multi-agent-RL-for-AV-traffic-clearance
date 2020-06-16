from Config import *
import numpy as np
import random
import traci
import warnings; do_warn = False
import copy
import jinja2




class env():

    def __init__(self, sumoBinary, name="MultiAgent1.0",  ambulance_goal_distance=500, rel_amb_y_min = -41, rel_amb_y_max = 16):


        self.name = name
        self.amb_goal_dist = ambulance_goal_distance
        self.reward = 0.0
        self.emer_start_lane = None
        self.emer_car_len = 2.0
        self.agent_car_len = 2.0

        self.rel_amb_y_min = rel_amb_y_min
        self.rel_amb_y_max = rel_amb_y_max

        self.agents = []  # Stays as is in multiagent
        self.emer = None  # Stays as is in multiagent

        self.hidden_state = None
        self.observed_state = None
        self.full_state = None

        self.Actions = ["change_left", "change_right", "acc", "no_acc", "dec"]
        self.action_to_string_dict = {
            "change_left": 0,
            "change_right": 1,
            "acc": 2,
            "no_acc": 3,
            "dec": 4
        }  # Must maintain order in Actions

        self.count_emergency_vehicles = 0  # Temporary assigned variable, reassigned in .reset()->recount_vehicles() to avoid calling a non-initialized vehicle
        self.count_ego_vehicles = 0  # Temporary assigned variable, reassigned in .reset()->recount_vehicles() to avoid calling a non-initialized vehicle
        # vehicles_data  # dict with: key = lane number, value = number of cars in lane
        self.max_possible_cars = None   # Maximum possible number of cars in lane give starting position

        self.optimal_time = 0   # Temporary assigned variable, reassigned in .reset() to avoid calling a non-initialized vehicle
        self.max_steps = 10000  # Temporary assigned variable, reassigned in .reset() to avoid calling a non-initialized vehicle

        # ---------------------------------------------------------------------------- #
        # 2 :        R A N D O M L Y      I N I T I A L I Z E       X M L s
        #                     and consequently vehicles data
        # ---------------------------------------------------------------------------- #
        self.templates_reset()

        # ---------------------------------------------------------------------------- #
        # 3 :          I N I T I A T E    S U M O     E N V I R O N M E N T
        #                             and vehicles list
        # ---------------------------------------------------------------------------- #
        traci.start([sumoBinary, "-c", Sumocfg_DIR,
                     "--tripinfo-output", "tripinfo.xml", "--seed", str(Sumo_random_seed)])  # SUMO starts

        vehicles_list = [Vehicle("LH")]  # NOTE: No error will be produced if some cars are not in this list.
        # An error will be produced only when in a not-present ID is requested , Vehicle("RB0"), Vehicle("RB1"), Vehicle("RB2"), Vehicle("RB3")

        # Create the real global vehicles list (temporary/fake: initialized one in Config.py with ambulance only):
        for lane, num_cars in vehicles_data.items():
            for agent_index in range(num_cars):
                vehicles_list.append(
                    Vehicle(env.create_vehicle_id(lane, agent_index))
                )

        for vehc in vehicles_list:  # vehicles initialized
            vehc.initialize()

        self.list_of_vehicles = copy.copy(vehicles_list)  # Note: to copy the list, keeping reference to the original vehicles (as opposed to deepcopy, which would copy vehicles)
        self.recount_vehicles()

    @staticmethod
    def create_vehicle_id(lane, agent_index, start_chars= "RB" ):
        return start_chars+"_" + "L" + str(lane) + "I" + str(agent_index)

    def templates_reset(self):
        """
        This function is called by env.reset() to reset the XML template contents based on Config.lanes_busyness and
        Config.lanes_busyness_mode

        :return: None. Just reset the templates.
        """
        # ---------------------------------------------------------------------------- #
        # 2 :        R A N D O M L Y      I N I T I A L I Z E       X M L s
        # ---------------------------------------------------------------------------- #

        new_stance = dict()  # This is the dict passed to jinja

        # 2.2: Init Emergency Start Lane
        new_stance['ambulance_start_lane'] = random.randint(0, num_lanes-1)

        new_stance['all_agents_start_position'] = random.randint(41, 250)  # TODO: Edit minimum position to depend on starting position
        new_stance['r1_new_length'] = new_stance['all_agents_start_position']
        new_stance['r2_new_length'] = 511 - new_stance['all_agents_start_position']

        new_stance['agent_min_gap'] = 2.5  # default
        new_stance['ambulance_min_gap'] = 2.5  # default

        new_stance['car_length'] = 2.0

        new_stance['agents_data_dicts'] = []

        distance_to_finish = self.amb_goal_dist - 10 - 1 - new_stance['all_agents_start_position']  # 10 = self.emer.max_speed

        max_cars_per_lane =  np.floor( distance_to_finish/         # According to this distance (changes in every reset)
                                       (new_stance['agent_min_gap'] + new_stance['car_length']
                                        + 5 ))  # 5 because it is the max agent velocity    #TODO: Do we have to add 5 ?
        self.max_possible_cars = max_cars_per_lane

        for lane in range(num_lanes):   # [0, 1, 2] for num_lanes = 3
            # ------------------------------------------------------------------- #
            # 1 :     C A R S    A T    E Q U A L    D I S T A N C E S
            # ------------------------------------------------------------------- #

            if lanes_busyness_mode == 0:  # i.e. place cars at equal distances
                cars_in_this_lane = int(max_cars_per_lane * lanes_busyness[lane])
                agent_index = 0
                distance_between_cars = distance_to_finish/cars_in_this_lane
                for car in range(cars_in_this_lane):

                    temp_dict = dict()  # agent_data temporary dict to be stored in new_stance['agents_data_dicts']

                    temp_dict['agent_id'] = self.create_vehicle_id(lane, agent_index) # "RB_" + "L" + str(lane) + "I" + str(agent_index)

                    # 2.1: Init Agent Start Lane
                    temp_dict['agent_start_lane'] = lane

                    # 2.3: Init Agent Start Position
                    temp_dict['departPos'] = distance_to_finish - agent_index * distance_between_cars
                    # random.randint(0, 511 - new_stance['all_agents_start_position'])  # distance infront of all_agents_start_position


                    new_stance['agents_data_dicts'].append(temp_dict)
                    agent_index += 1
                vehicles_data[lane] = agent_index  # number of cars in this lane

            # ------------------------------------------------------------------- #
            # 2 :  C A R S    P R O D U C E D   W I T H   P R O B A B I L I T Y
            # ------------------------------------------------------------------- #

            elif lanes_busyness_mode == 1:  # i.e. place cars at equal distances
                # cars_in_this_lane = int(max_cars_per_lane * lanes_busyness[lane])
                agent_index = 0
                real_agent_index = 0  # That gets updated only if we actually add the vehicle
                min_distance_between_cars =(new_stance['agent_min_gap']   # According to this distance (changes in every reset)
                                        + new_stance['car_length']
                                        + 5 )
                for car in range(int(max_cars_per_lane)):

                    temp_dict = dict()  # agent_data temporary dict to be stored in new_stance['agents_data_dicts']

                    temp_dict['agent_id'] = self.create_vehicle_id(lane, real_agent_index)  # "RB_" + "L" + str(lane) + "I" + str(real_agent_index)

                    # 2.1: Init Agent Start Lane
                    temp_dict['agent_start_lane'] = lane

                    # 2.3: Init Agent Start Position
                    temp_dict['departPos'] = distance_to_finish - agent_index * min_distance_between_cars
                    # random.randint(0, 511 - new_stance['all_agents_start_position'])  # distance infront of all_agents_start_position

                    # Do we add this car ?
                    add_this_car = random.random()
                    if add_this_car < lanes_busyness[lane]:
                        new_stance['agents_data_dicts'].append(temp_dict)
                        real_agent_index += 1

                    agent_index += 1

                vehicles_data[lane] = real_agent_index   # number of cars in this lane







        # for agent_index in range(int(max_cars_per_lane)):
        #     temp_dict = dict()  # agent_data temporary dict to be stored in new_stance['agents_data_dicts']
        #
        #     temp_dict['agent_id'] = agent_index
        #
        #     # 2.1: Init Agent Start Lane
        #     temp_dict['agent_start_lane'] = random.randint(0, num_lanes-1)
        #
        #     # 2.3: Init Agent Start Position
        #     temp_dict['departPos'] = random.randint(0, 511 - new_stance['all_agents_start_position'])  # distance infront of all_agents_start_position
        #
        #     new_stance['agents_data_dicts'].append(temp_dict)


        # 2.4: Load Template to template
        templateLoader = jinja2.FileSystemLoader(searchpath=TEMPLATES_PATH)
        templateEnv = jinja2.Environment(loader=templateLoader)

        # 2.5: Put rou template to file
        rou_template = templateEnv.get_template("route_template.xml")
        with open(ROUTE_FILE_PATH, "w") as fp:
            fp.writelines(rou_template.render(data=new_stance))

        # 2.6: Put net template to file:
        net_template = templateEnv.get_template("net_template.xml")
        with open(NET_FILE_PATH, "w") as fp:
            fp.writelines(net_template.render(data=new_stance))

    def reset(self, sumoBinary):
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
        # self.amb_goal_dist = self.amb_goal_dist
        self.reward = 0.0
        self.emer_start_lane = None

        # self.rel_amb_y_min = self.rel_amb_y_min
        # self.rel_amb_y_max = self.rel_amb_y_max


        # self.count_emergency_vehicles = self.count_emergency_vehicles
        # self.count_ego_vehicles = self.count_ego_vehicles
        # self.agents = self.agents
        # self.emer = self.emer

        self.hidden_state = None
        self.observed_state = None
        self.full_state = None



        # self.Actions = self.Actions
        # self.action_to_string_dict = self.action_to_string_dict

        # Calculation for optimal time is kept in case the track_len is changed between episodes
        self.optimal_time = int(np.round(
            track_len / self.emer.max_speed))  # Optimal Number of time steps: number of time steps taken by ambulance at maximum speed
        self.max_steps = 20 * self.optimal_time

        # ---------------------------------------------------------------------------- #
        # 2 :        R A N D O M L Y      I N I T I A L I Z E       X M L s
        # ---------------------------------------------------------------------------- #
        self.templates_reset()  # Vehicles list gets reset here

        # ---------------------------------------------------------------------------- #
        # 3 :          I N I T I A T E    S U M O     E N V I R O N M E N T
        #                             and vehicles list
        # ---------------------------------------------------------------------------- #
        traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml",
                    "--start", "--seed", str(Sumo_random_seed)])

        # Create the real global vehicles list (temporary/fake: initialized one in Config.py with ambulance only):

        vehicles_list = [Vehicle("LH")]  # NOTE: No error will be produced if some cars are not in this list.
        # An error will be produced only when in a not-present ID is requested , Vehicle("RB0"), Vehicle("RB1"), Vehicle("RB2"), Vehicle("RB3")


        for lane, num_cars in vehicles_data.items():
            for agent_index in range(num_cars):
                vehicles_list.append(
                    Vehicle(env.create_vehicle_id(lane, agent_index))
                )

        for vehc in vehicles_list:  # vehicles initialized
            vehc.initialize()

        self.list_of_vehicles = copy.copy(
            vehicles_list)  # Note: to copy the list, keeping reference to the original vehicles (as opposed to deepcopy, which would copy vehicles)
        self.recount_vehicles()




    def get_emer_start_lane(self):  # Called at beginning of episode only
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

        elements_to_remove = []
        for agent_index in range( self.count_ego_vehicles ):
            agent_abs_y = self.full_state[-1][agent_index] # #Please refer to shape of full_state list in env.measure_full_state
                                                            # hidden_state shape: [ agent_abs_y ... for vehicle in vehicles , amb_abs_y]
            if (agent_abs_y > self.amb_goal_dist - self.emer.max_speed - 1):
                # Delete element (so that it won't be referred to, again) and just continue
                # print(f'Removing {self.list_of_vehicles[agent_index+1].ID} ...')    # Why plus one (as opposed to just agent_index)?:: because ambulance is vehicles_list[0]
                elements_to_remove.append(self.list_of_vehicles[agent_index+1])
                # return 3

        if len(elements_to_remove) > 0:
            for agent in elements_to_remove:
                self.list_of_vehicles.remove(agent)
            self.recount_vehicles()

        # print("-------------------------------------------------------------------")
        # print("Local list:", [agent.ID for agent in  self.list_of_vehicles])
        # print("Global list:", [agent.ID for agent in vehicles_list])

        # 0: not done
        return 0

    def recount_vehicles(self):
        self.count_emergency_vehicles = 0
        self.count_ego_vehicles = 0

        for vhcl in self.list_of_vehicles:  # Count vehicles according to vehicle type

            if (vhcl.type == "Emergency"):
                # ------------------------ checks - start -----------------------#
                self.count_emergency_vehicles += 1
                if self.count_emergency_vehicles > 1:
                    raise ValueError(
                        f'WaleedError: Number of emergency vehicles = {self.count_emergency_vehicles}. Only a single emrgency vehicle is expected to be present.\n'
                        f'Please edit your code in env.__init__() if you changed this in your problem formulation.'
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

        self.optimal_time = int(np.round(
            track_len / self.emer.max_speed))  # Optimal Number of time steps: number of time steps taken by ambulance at maximum speed
        self.max_steps = 20 * self.optimal_time

    def get_vehicle_object_by_id(self, vehID):
        '''
        :param vehID: string, ID of the vehicle to get the Vehicle object for
        :return: Vehicle, object that has the ID equal to vehID
        '''
        for vhc in self.list_of_vehicles:
            if vhc.ID == vehID :
                return vhc
        return None  # If vehID does not belong to any vehicle, None value is returned

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

    def calc_reward(self, amb_last_velocity, done, number_of_steps, max_final_reward=20, min_final_reward=-20, max_step_reward=0, min_step_reward = -1.25):
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

            self.emer.getSpd()  # Make sure emergency vehicle's speed is up-to-date

            m = (max_step_reward - min_step_reward)/(2 * self.emer.max_accel)  # Slope for straight line equation to calculate step reward
            #2 * self.emer.max_accel since: = self.emer.max_accel - * self.emer.max_decel
            c = max_step_reward - self.emer.max_accel * m  # c is y-intercept for the reward function equation #max_step_reward is the y for x = 2 (max acceleration)
            reward = m * (self.emer.spd - amb_last_velocity) + c
            #debug#print(f'c: {c}, m: {m}, accel: {(self.emer.spd - amb_last_velocity)}')

            if ( abs(amb_last_velocity-self.emer.max_speed) <= 1e-10 ):
            #since ambulance had maximum speed and speed did not change that much; unless we applied the code below.. the acceleration
            #   will be wrongly assumed to be zero. Although the ambulance probably could have accelerated more, but this is its maximum velocity.
                reward = max_step_reward #same reward as maximum acceleration (+2),

            self.reward = reward
            return reward

    def __str__(self):  #:return: environment string representation
        return str(
            {"name": self.name, "reward": self.reward, "Vehicles": [vhc.ID for vhc in self.list_of_vehicles], "full_state": self.full_state})
