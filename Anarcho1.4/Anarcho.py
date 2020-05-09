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

from Checks import *
from Utils.SimTools import *
from Environment.env import *



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
    Proudhon = env(vehicles_list, sumoBinary)  # env.__init__ and template loading
    # environment reset inside episode()
    ## ----- ##

    episode_num = 0

    Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list = SimTools.episode(sumoBinary=sumoBinary, Proudhon=Proudhon)
    # total_reward_per_episode.append(episode_reward)   #RLcomment
    # reward_history_per_episode.append(episode_reward_list) #RLcomment

    ## --- ##
    environment_for_next_episode.reset(sumoBinary=sumoBinary)
    ## --- ##

    while(episode_num < max_num_episodes):

        episode_num += 1

        Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list = \
            SimTools.episode(sumoBinary=sumoBinary, RB_RLAlgorithm=Algorithm_for_RL, Proudhon=environment_for_next_episode, episode_num=episode_num)
        # total_reward_per_episode.append(episode_reward) #RLcomment
        # reward_history_per_episode.append(episode_reward_list)  #RLcomment

        ##  --  ##
        if (enable_checks):
            are_we_ok(environment_for_next_episode)

        ## -- ##
        # 5: Reset environment in preparation for next episode
        environment_for_next_episode.reset(sumoBinary)
        # TODO: https://stackoverflow.com/questions/59166732/how-to-disable-print-loading-configuration-done-in-sumo-traci and stop printing termination step number
        ## -- ##

    # Save Q-table after episodes ended:
    # Algorithm_for_RL.save_q_table() #RLcomment
