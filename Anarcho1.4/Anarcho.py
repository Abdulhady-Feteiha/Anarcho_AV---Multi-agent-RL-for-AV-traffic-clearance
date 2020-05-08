#!/usr/bin/env python
from Config import *  # Make sure this is the first one imported, to have random.seed() used before any actual randomness is assigned
import os
import sys
import optparse
import numpy as np
from sumolib import checkBinary
import traci
from Utils.helpers import *
from RL.SingleAgent import RLAlgorithm
from Environment.env import env
from Checks import *

from Utils.SimTools import SimTools



if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--Train", action="store_true",
                         default=False, help="Train and save a Q table")
    opt_parser.add_option("--Test", action="store_true",
                         default=False, help="Test a saved Q table")
    options, args = opt_parser.parse_args()
    return options


if __name__ == "__main__":

    #TODO: print at the end of every episode/iteration inside epsisode -- according to visualization paramters from Config.py
    #max_window = measure() #TODO: Make Q-table and assignments depend on max_window+10 from behind, and max_agent_velocity*3 forward

    options = get_options()

    if options.Test:
        vis_update_params['test_mode_on'] = True
        sumoBinary = checkBinary('sumo-gui')

    if options.Train:
        vis_update_params['test_mode_on'] = False
        sumoBinary = checkBinary('sumo')


    ## ----- ##
    Proudhon = env(vehicles_list)   # env.__init__ and template loading
    traci.start([sumoBinary, "-c", Sumocfg_DIR,
                             "--tripinfo-output", "tripinfo.xml", "--seed", str(Sumo_random_seed)])  # SUMO starts
    for vehc in vehicles_list:  # vehicles initialized
        vehc.initialize()

    # environment reset inside episode()
    ## ----- ##

    episode_num = 0

    Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list = SimTools.episode(Proudhon=Proudhon)
    # total_reward_per_episode.append(episode_reward)   #RLcomment
    # reward_history_per_episode.append(episode_reward_list) #RLcomment

    ## --- ##
    environment_for_next_episode.reset()
    traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml", "--start", "--seed", str(Sumo_random_seed)])
    for vehc in vehicles_list:
        vehc.initialize()  # Placed here to set lane change mode ! Important !
    ## --- ##

    while(episode_num < max_num_episodes):

        episode_num += 1

        Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list = SimTools.episode(Algorithm_for_RL, environment_for_next_episode, episode_num)
        # total_reward_per_episode.append(episode_reward) #RLcomment
        # reward_history_per_episode.append(episode_reward_list)  #RLcomment

    ##  --  ##
        if (enable_checks):
            are_we_ok(environment_for_next_episode)

        ## -- ##
        # 5: Reset environment in preparation for next episode
        environment_for_next_episode.reset()
        # Load XMLs:
        traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml", "--start", "--seed", str(Sumo_random_seed)])
        # TODO: https://stackoverflow.com/questions/59166732/how-to-disable-print-loading-configuration-done-in-sumo-traci and stop printing termination step number
        for vehc in vehicles_list:
            vehc.initialize()  # Placed here to set lane change mode ! Important !
        ## -- ##

    # Save Q-table after episodes ended:
    # Algorithm_for_RL.save_q_table() #RLcomment
