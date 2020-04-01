import random

# Reproducibility:  # Keep at the top
Sumo_random_seed = 5  # Any number, just fix it through the runs
Python_random_seed = 0  # Any number, just fix it through the runs
random.seed(Python_random_seed)

# ----------------------------------------------------- #

import os
import numpy as np

# Path Variables:
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
Sumocfg_DIR = os.path.join(BASE_PATH, r"Sumo_files/Anarcho1.3.sumocfg")
TEMPLATES_PATH = os.path.join(BASE_PATH, r"Templates")
NET_FILE_PATH = os.path.join(BASE_PATH, r"Sumo_files\Anarcho1.3.net.xml")
ROUTE_FILE_PATH = os.path.join(BASE_PATH, r"Sumo_files\Anarcho1.3.rou.xml")
VARIABLES_FOLDER = os.path.join(BASE_PATH, r"Saved Variables")


# Simulation Variables:
track_len = 500
SimTime = 1000.0  # Maximum number of time steps per episode
max_num_episodes = 50000  # Number of training episodes

# Visual Update Parameters
vis_update_params = dict()
vis_update_params['every_n_episodes'] = 1  # Print Episode info every_n_episodes
vis_update_params['every_n_iters'] = 1  # Print Iteration info every_n_iters inside single episode
vis_update_params['print_reward_every_episode'] = True
vis_update_params['test_mode_on'] = False  # If test mode is on, the q_table is loaded and not saved nor updated, gui is on, exploit is used always



# History Variables:
total_reward_per_episode = []  # list of doubles
reward_history_per_episode = []  # list of lists

# Q-learning Parameters:
q_learning_params = dict()
q_learning_params['exp_exp_tradeoff'] = random.uniform(0,
                                                       1)  # TODO: Add random seed to enable replication. #Only keep the exp_exp_tradeoff here.
q_learning_params['learning_rate'] = 0.7  # Learning rate
q_learning_params['gamma'] = 0.5  # Discounting rate
# Exploration parameters
q_learning_params['epsilon'] = 1.0  # Exploration rate
q_learning_params['max_epsilon'] = 1.0  # Exploration probability at start
q_learning_params['min_epsilon'] = 0.01  # Minimum exploration probability
q_learning_params['decay_rate'] = 0.0001  # Exponential decay rate for exploration prob
load_q_table = False

# Reward Parameters:
give_final_reward = False  # bool: if False, no final reward is given. Step by Step reward only is given.
enable_checks = True





#Don't forget to initialize SimTime before importing vehicle
from Utils.Vehicle import Vehicle
vehicles_list = [Vehicle("LH"), Vehicle("RB")]
