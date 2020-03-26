import os
import numpy as np
import random

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
Sumocfg_DIR = os.path.join(BASE_PATH, "Sumo_files/Anarcho1.3.sumocfg")

track_len = 500
SimTime = 1000.0  # Maximum number of time steps per episode
max_num_episodes = 1000 # Number of training episodes

# Visual Update Parameters
vis_update_params = dict()
vis_update_params['every_n_episodes'] = 1  # Print Episode info every_n_episodes
vis_update_params['every_n_iters'] = 10  # Print Iteration info every_n_iters inside single episode

# History Variables:
total_reward_per_episode = []  # list of doubles
reward_history_per_episode = []  # list of lists

# Q-learning Parameters:
q_learning_params = dict()
q_learning_params['exp_exp_tradeoff'] = random.uniform(0,
                                                       1)  # TODO: Add random seed to enable replication. #Only keep the exp_exp_tradeoff here.
q_learning_params['learning_rate'] = 0.7  # Learning rate
q_learning_params['gamma'] = 0.618  # Discounting rate
# Exploration parameters
q_learning_params['epsilon'] = 1.0  # Exploration rate
q_learning_params['max_epsilon'] = 1.0  # Exploration probability at start
q_learning_params['min_epsilon'] = 0.01  # Minimum exploration probability
q_learning_params['decay_rate'] = 0.01  # Exponential decay rate for exploration prob

# Reward Parameters:
give_final_reward = False  # bool: if False, no final reward is given. Step by Step reward only is given.

#Don't forget to initialize SimTime before importing vehicle
from Utils.Vehicle import Vehicle
vehicles_list = [Vehicle("LH"), Vehicle("RB")]
