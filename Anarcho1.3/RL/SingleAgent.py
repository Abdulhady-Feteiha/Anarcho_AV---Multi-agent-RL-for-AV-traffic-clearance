import numpy as np
import random
from Config import *

class RLAlgorithm():
    '''
    Access order for the Q_table is [agent_vel][agent_lane][amb_vel][amb_lane][rel_amb_y][action].. remember to change the value rel_amb_y to be positive [0,58]
    '''

    def __init__(self, environment, name="Q-learning", algo_params=dict()):
        '''
        #Usage is: One RLAlgorithm object per training agent
        :param environment:  of class env, contains list of vehicles and observations.
        :param name:         string, currently not used except for display purposes
        '''
        self.name = name
        self.environment = environment
        self.QActions = self.environment.Actions
        # self.feasible_actions = self.environment.ge
        self.action_to_string_dict = self.environment.action_to_string_dict

        self.action_chosing_method = None  # To be asssigned: Exploration or Exploitation based on exp_exp_tradeoff and epsilon

        # algo_params:
        if (name == "Q-learning"):  # default case for now
            if(load_q_table):
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
        #TODO: feasible_actions_for_chosen_action -->>> var_name_change ACTUALLY: feasible_actions_for_current_state
        feasible_action_indices = []
        for act in feasible_actions_for_chosen_action:
            feasible_action_indices.append(self.action_to_string_dict[act])

        rel_amb_y_min = self.environment.rel_amb_y_min
        rel_amb_y_max = self.environment.rel_amb_y_max

        new_agent_vel = new_observed_state_for_this_agent[0]
        new_agent_vel_index = int(np.round(new_agent_vel))  # [0,1,2,3,4,5]

        new_agent_lane_index = new_observed_state_for_this_agent[1]

        new_amb_vel = new_observed_state_for_this_agent[2]
        new_amb_vel_index = int(np.round(new_amb_vel))  # [0,1,2,3,4,5,6,7,8,9,10]

        new_amb_lane_index = new_observed_state_for_this_agent[3]

        new_rel_amb_y = np.clip(new_observed_state_for_this_agent[4], rel_amb_y_min,
                            rel_amb_y_max)  # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
        new_rel_amb_y_index = int(np.round(new_rel_amb_y) + abs(rel_amb_y_min))

        '''
        Previous comment by Waleed:
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


        #self.Action = self.QActions[randrange(len(self.QActions))]
        self.exp_exp_tradeoff = random.uniform(0,1)

        if self.exp_exp_tradeoff > self.epsilon:
            self.action_chosing_method = 'expLOIT'

            max_value_index= np.argmax(self.q_table[new_agent_vel_index, new_agent_lane_index, new_amb_vel_index, new_amb_lane_index, new_rel_amb_y_index, feasible_action_indices])
            action_index =feasible_action_indices [max_value_index]

            desired_action_string = self.QActions[action_index]
            self.Action = desired_action_string

            # debug#print("For this state, I am the max index : ",max_value_index)
            #debug# print ("I am the picked action for exploitation ",desired_action_string)
        else:
            self.action_chosing_method = 'expLORE'
            action_index = random.choice(feasible_action_indices)

            desired_action_string = self.QActions[action_index]
            #debug# print("I am the picked action for exploration ", desired_action_string)
            self.Action = desired_action_string

        return self.Action

    def applyAction(self, action, agent):
        '''

        :param action: action chosen by pick Action function
        :param agent: our lovely RL agent
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
            agent.inst_acc(1)   # hard coded
            # NOTE: This will ask the agent to seek the maximum safe velocity closest to current_velocity + 1, because SUMO
            # clips the velocity automatically. Take care during #ROS implementation.
        elif action == "dec":
            agent.inst_acc(-1)  # hard coded
        elif action == "no_acc":
            pass

    def update_q_table(self, chosen_action, reward, new_observed_state_for_this_agent,
                       last_observed_state_for_this_agent, feasible_actions_for_chosen_action):
        '''

        :param chosen_action:
        :param observed_state:
        :return:

        :Notes:
        #Assumed Hierarchy of observed_state_for_this_agent:[agent_vel , agent_lane , amb_vel , amb_lane , rel_amb_y]

        '''

        # OLD STATE VARIABLES:
        agent_vel = last_observed_state_for_this_agent[0]
        agent_vel_index = int(np.round(agent_vel))  # [0,1,2,3,4,5]

        rel_amb_y_min = self.environment.rel_amb_y_min
        rel_amb_y_max = self.environment.rel_amb_y_max

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
        for act in feasible_actions_for_chosen_action:
            feasible_action_indices.append(self.action_to_string_dict[act])

        # Update Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
        q_of_s_a_value = \
        self.q_table[agent_vel_index][agent_lane_index][amb_vel_index][amb_lane_index][rel_amb_y_index][action_index]
        max_q_of_s_value_new = np.max(self.q_table[
                                          new_agent_vel_index, new_agent_lane_index, new_amb_vel_index, new_amb_lane_index, new_rel_amb_y_index, feasible_action_indices])
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

    def save_q_table(self, variables_folder_path = VARIABLES_FOLDER):
        np.save(variables_folder_path+'/Q_TABLE.npy', self.q_table)
        print(f"Loaded Q_TABLE from {variables_folder_path+'/Q_TABLE.npy'}")

    def load_q_table(self, variables_folder_path = VARIABLES_FOLDER):
        return np.load(variables_folder_path+'/Q_TABLE.npy')
