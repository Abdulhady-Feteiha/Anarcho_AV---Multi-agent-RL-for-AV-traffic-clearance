import numpy as np
from Config import *
import Config
import random


class ControlAlgo():
    """
    Class to handle Requesting actions and retrieving measurements from SUMO. In previous versions (prev to 1.41) the code had one RLALgorithm
    because it only had one agent (and hence the previous name of the file, SingleAgent.py) now the file name is changed to ControlAlgo.py and the
    class name is changed to ControlAlgo (As opposed to RLAlgrotihm) to reflect its nature as:
    - It could be Multi-Agent control algorithm
    - It could be an Autonomous control algorithm (Not RL: Reinforcement Learning control algorithm)

    The ideology of one Algorithm per agent is also used effectively in the since that: the Algorithm will be a property of the vehicle class,
    and hence each Vehicle will have an Algorithm.
    """
    '''
    Access order for the Q_table is [agent_vel][agent_lane][amb_vel][amb_lane][rel_amb_y][action].. remember to change the value rel_amb_y to be positive [0,58]
    '''

    def __init__(self, vehicle_params, name="SUMO_KRAUSS", algo_params=dict(), load_q_table=False, test_mode_on=False):
        """

        :param environment: environment (from env.py) that the Vehicle is in
        :param name:
        :param algo_params:
        :param load_q_table:
        :param test_mode_on:
        """
        self.name = name  # Used only since version 1.41K to judge for used Algorithm.
        # possible values: ["Q_LEARNING_SINGLE_AGENT", "SUMO_KRAUSS"]
        self.test_mode_on = test_mode_on


        self.vehicle_params = vehicle_params
        self.QActions = vehicle_params['Actions']
        self.action_to_string_dict = vehicle_params['action_to_string_dict']   # TODO: var_name_change action_to_string_dict to action_string_to_index_dict

        self.action_chosing_method = None  # To be asssigned: Exploration or Exploitation based on exp_exp_tradeoff and epsilon

        # algo_params:
        if name == "Q_LEARNING_SINGLE_AGENT":  # default case for now
            if load_q_table or self.test_mode_on:  # load_q_table if we are testing or we want to load it
                self.q_table = self.load_q_table()
            else:
                self.q_table = np.zeros((6, 3, 11, 3, 58, 5))
            self.exp_exp_tradeoff = algo_params['exp_exp_tradeoff']
            self.epsilon = algo_params['epsilon']

            self.gamma = algo_params['gamma']
            self.learning_rate = algo_params['learning_rate']
            self.max_epsilon = algo_params['max_epsilon']
            self.min_epsilon = algo_params['min_epsilon']
            self.decay_rate = algo_params['decay_rate']

    def pickAction(self, feasible_actions_for_chosen_action, new_observed_state_for_this_agent):
        '''
        :param feasible_actions_for_chosen_action: feasible actions for new_observed_state_for_this_agent to choose from
        :param new_observed_state_for_this_agent: current state to choose action for
        :return:
        '''

        if  self.name == "Q_LEARNING_SINGLE_AGENT":  # We will choose action via Q_TABLE only in this case
            # TODO: feasible_actions_for_chosen_action -->>> var_name_change ACTUALLY: feasible_actions_for_current_state
            feasible_action_indices = []
            for act in feasible_actions_for_chosen_action:
                feasible_action_indices.append(self.action_to_string_dict[act])

            rel_amb_y_min = self.vehicle_params['rel_amb_y_min']   # -41
            rel_amb_y_max = self.vehicle_params['rel_amb_y_max']  # +16 #TODO: Does it have a meaning ?

            new_agent_vel = new_observed_state_for_this_agent[0]
            new_agent_vel_index = int(np.round(new_agent_vel))  # [0,1,2,3,4,5]

            new_agent_lane_index = new_observed_state_for_this_agent[1]

            new_amb_vel = new_observed_state_for_this_agent[2]
            new_amb_vel_index = int(np.round(new_amb_vel))  # [0,1,2,3,4,5,6,7,8,9,10]

            new_amb_lane_index = new_observed_state_for_this_agent[3]

            new_rel_amb_y = np.clip(new_observed_state_for_this_agent[4], rel_amb_y_min,
                                    rel_amb_y_max)  # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
            new_rel_amb_y_index = int(np.round(new_rel_amb_y) + abs(rel_amb_y_min))


            self.exp_exp_tradeoff = random.uniform(0, 1)    # Note: this kept replicable by the unified random seed in Config.py

            if self.exp_exp_tradeoff > self.epsilon or self.test_mode_on:  # tes_mode_on will force the algorithm to choose exploitation.
                self.action_chosing_method = 'expLOIT'

                max_value_index = np.argmax(self.q_table[
                                                new_agent_vel_index, new_agent_lane_index, new_amb_vel_index, new_amb_lane_index, new_rel_amb_y_index, feasible_action_indices])
                action_index = feasible_action_indices[max_value_index]

                desired_action_string = self.QActions[action_index]
                self.Action = desired_action_string

                # debug#print("For this state, I am the max index : ",max_value_index)
                # debug# print ("I am the picked action for exploitation ",desired_action_string)
            else:
                self.action_chosing_method = 'expLORE'
                action_index = random.choice(feasible_action_indices)

                desired_action_string = self.QActions[action_index]
                # debug# print("I am the picked action for exploration ", desired_action_string)
                self.Action = desired_action_string

        elif self.name == "SUMO_KRAUSS":    # Leave control to SUMO, return NONE action
            self.Action = None

        else:
            raise ValueError(f"Unknown ALgorithm Name requested from vehicle {self.vehicle_params['ID']}, value of name: {self.name}")


        return self.Action

    def applyAction(self, action, agent):
        '''
        :function: Requests environment to apply an action on given agent
        :param action: string, action chosen by pick Action function
        :param agent: Vehicle object, our lovely RL agent
        :return: None but it apply action on the agent

        this function has a hard coded in acc , dec function as till know we have used +1,-1 to be only possible values
         for acc, dec.
        if it is not the case then :param action will be string with the value of acc and there will be a parasing func to do so
        '''
        if action == "change_left":
            agent.chLeft()
        elif action == "change_right":
            agent.chRight()
        elif action == "acc":
            agent.inst_acc(1)  # hard coded
            # NOTE: This will ask the agent to seek the maximum safe velocity closest to current_velocity + 1, because SUMO
            # clips the velocity automatically. Take care during #ROS implementation.
        elif action == "dec":
            agent.inst_acc(-1)  # hard coded
        elif action == "no_acc":
            pass    # TODO: IMPORTANT: ISSUE -- https://github.com/Abdulhady-Feteiha/Anarcho_AV---Multi-agent-RL-for-AV-traffic-clearance/issues/2
        elif action is None:
            pass  # Action produced by "SUMO_KRAUSS" is None. If given, we request no action from SUMO>
        else:  # Unrecognized action
            raise ValueError(f"Unrecgonized action requested from {agent.ID}.applyAction: {action}")

    def update_q_table(self, chosen_action, reward, new_observed_state_for_this_agent,
                       last_observed_state_for_this_agent, feasible_actions_for_new_observed_state):
        """"
        :param chosen_action: action chosen over last time step, the one for which we see the results in new_observed_state_for_this_agent
        :param reward: reward due to applying the chosen_action on last_observed_state_for_this_agent and getting new_observed_state_for_this_agent
        :param new_observed_state_for_this_agent: observed state due to applying chose_action on last_observed_state_for_this_agent
        :param feasible_actions_for_new_observed_state: feasible action for feasible_actions_for_new_observed_state

        :return: None, but updates q_table

        :Notes:
        #Assumed Hierarchy of observed_state_for_this_agent:[agent_vel , agent_lane , amb_vel , amb_lane , rel_amb_y]

        """
        if (self.test_mode_on):  # do not update q_table if test_mode is on ! just use it.
            pass
        else:
            if self.name == "SUMO_KRAUSS":  # DO NOT UPDATE Q_TABLE or train if not requested (actually, there will be no q-table then)
                pass

            elif self.name == "Q_LEARNING_SINGLE_AGENT":    # UPDATE Q_TABLE and so one
                rel_amb_y_min = self.vehicle_params['rel_amb_y_min']  # -41
                rel_amb_y_max = self.vehicle_params['rel_amb_y_max']  # +16 #TODO: Does it have a meaning ?

                # OLD STATE VARIABLES:
                agent_vel = last_observed_state_for_this_agent[0]
                agent_vel_index = int(np.round(agent_vel))  # [0,1,2,3,4,5]

                agent_lane_index = last_observed_state_for_this_agent[1]

                amb_vel = last_observed_state_for_this_agent[2]
                amb_vel_index = int(np.round(amb_vel))  # [0,1,2,3,4,5,6,7,8,9,10]

                amb_lane_index = last_observed_state_for_this_agent[3]

                rel_amb_y = np.clip(last_observed_state_for_this_agent[4], rel_amb_y_min,
                                    rel_amb_y_max)  # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
                rel_amb_y_index = int(np.round(rel_amb_y) + abs(rel_amb_y_min))

                action_index = self.action_to_string_dict[chosen_action]

                # NEW STATE VARIABLES:
                new_agent_vel = new_observed_state_for_this_agent[0]
                new_agent_vel_index = int(np.round(new_agent_vel))  # [0,1,2,3,4,5]

                new_agent_lane_index = new_observed_state_for_this_agent[1]

                new_amb_vel = new_observed_state_for_this_agent[2]
                new_amb_vel_index = int(np.round(new_amb_vel))  # [0,1,2,3,4,5,6,7,8,9,10]

                new_amb_lane_index = new_observed_state_for_this_agent[3]

                new_rel_amb_y = np.clip(new_observed_state_for_this_agent[4], rel_amb_y_min,
                                        rel_amb_y_max)  # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
                new_rel_amb_y_index = int(np.round(new_rel_amb_y) + abs(rel_amb_y_min))

                # Feasible Action Indices:
                feasible_action_indices = []
                for act in feasible_actions_for_new_observed_state:
                    feasible_action_indices.append(self.action_to_string_dict[act])

                # Update Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
                '''
                s: old state for which a was decided
                a: action taken in state s
                s': new state we are in because of action a (previous action)
                a': new action we are expected to choose if we exploit on s' (new state)
                '''
                # Q(s,a):
                q_of_s_a_value = \
                    self.q_table[agent_vel_index][agent_lane_index][amb_vel_index][amb_lane_index][rel_amb_y_index][
                        action_index]

                # max Q(s',a')
                max_q_of_s_value_new = np.max(self.q_table[
                                                  new_agent_vel_index, new_agent_lane_index, new_amb_vel_index, new_amb_lane_index, new_rel_amb_y_index, feasible_action_indices])

                # Actual Update:
                q_of_s_a_value = q_of_s_a_value + self.learning_rate * (
                            reward + self.gamma * max_q_of_s_value_new - q_of_s_a_value)

                # actual update step:
                self.q_table[agent_vel_index][agent_lane_index][amb_vel_index][amb_lane_index][rel_amb_y_index][
                    action_index] = q_of_s_a_value

    '''
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
        '''

    def save_q_table(self, variables_folder_path=VARIABLES_FOLDER):
        if (self.test_mode_on):
            pass  # do not save
        else:
            np.save(variables_folder_path + '/Q_TABLE.npy', self.q_table)

    def load_q_table(self, variables_folder_path=VARIABLES_FOLDER):
        if(Config.AGGREGATION_METHOD == "MAX"):
            #debug#print(f"Loaded Q_TABLE from {variables_folder_path + '/Q_TABLE_MAX.npy'}")
            return np.load(variables_folder_path + '/Q_TABLE_MAX.npy')

        else:
            #debug#print(f"Loaded Q_TABLE from {variables_folder_path + '/Q_TABLE_AVG.npy'}")
            return np.load(variables_folder_path + '/Q_TABLE_AVG.npy')
