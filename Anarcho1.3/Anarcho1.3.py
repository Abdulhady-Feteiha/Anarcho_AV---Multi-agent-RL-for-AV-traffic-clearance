#!/usr/bin/env python
import os
import sys
import optparse
import numpy as np
from sumolib import checkBinary
import traci
from Utils.measure_max_window import measure
from Config import *
from Utils.Vehicle import Vehicle
from Utils.helpers import *
from RL.SingleAgent import RLAlgorithm
from Environment.env import env

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--GUI", action="store_true",
                         default=False, help="run the GUI version of sumo")
    options, args = opt_parser.parse_args()
    return options


def episode(RB_RLAlgorithm = None, Proudhon = None, episode_num = 0):
    ########################
    # 1 inits
    done = False  # are we done with the episode or not
    step = 1  # step number
    if(Proudhon is None): Proudhon = env(vehicles_list)  # vehicles_list = [LH, RB]
    if(RB_RLAlgorithm is None):
        algo_params = q_learning_params  # from Config.py
        RB_RLAlgorithm = RLAlgorithm(Proudhon, algo_params= algo_params)  # Algorithm for RB Agent
    ## ######################

    ########################
    # 2 init measurements
    traci.simulationStep()  # After stepping
    Proudhon.get_emer_start_lane()

    # (communication from environment):
    getters(vehicles_list)  # measure all values from environment
    # (communication to agent):
    Proudhon.measure_full_state()  # measure all values into our agents
    # (communication to algorithm):
    new_observed_state_for_this_agent = Proudhon.observed_state[0]

    # Chose Action from Feasible Actions:
    feasible_actions_for_current_state = Proudhon.get_feasible_actions(vehicles_list[1])
    chosen_action = RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent)
    RB_RLAlgorithm.applyAction(chosen_action, vehicles_list[1])  # Request Action on Agent

    episode_reward = 0
    episode_reward_list = []
    ########################

    # 3: MAIN LOOP
    while traci.simulation.getMinExpectedNumber() > 0:

        # 3.1: Store last states
        amb_last_velocity = Proudhon.emer.spd
        last_observed_state = Proudhon.observed_state
        last_observed_state_for_this_agent = last_observed_state[0]

        # ----------------------------------------------------------------- #
        # 3.2:   M O V E      O N E      S I M U L A T I O N       S T E P
        # ----------------------------------------------------------------- #
        traci.simulationStep()  # actual action applying
        step += 1

        #TODO: Turn this into are_we_ok function
        if(vehicles_list[0].getL() != Proudhon.emer_start_lane):
            raise ValueError(f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {vehicles_list[0].getL()} on step {step}. "
                             f"\nAmbulance Should not change lane. Quitting.")

        if (step % vis_update_params['every_n_iters'] == 0): # print step info
            print(f'E:{episode_num: <{6}}|S:{step: <{4}} | '
                  f'reward : {str(Proudhon.reward)[:min(5,len(str(Proudhon.reward)))]: <{5}}, '
                  f'lastAction: {chosen_action : <{12}} | '
                  f'cumReward: ' + str(episode_reward)[:6] + ' '*max(0, 6 - len(str(episode_reward))) +
                  f' | state: {[str(x)[:5]+" "*max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]}, ')
        # ----------------------------------------------------------------- #

        # 3.3: measurements and if we are done check
        getters(vehicles_list)
        Proudhon.measure_full_state()
        done = Proudhon.are_we_done(full_state=Proudhon.full_state, step_number=step)

        # 3.4: reward last step's chosen action
        reward = Proudhon.calc_reward(amb_last_velocity, done, step)
        episode_reward += reward  # for history
        episode_reward_list.append(reward)  # for history

        # 3.5: update q table using backward reward logic
        RB_RLAlgorithm.update_q_table(chosen_action, reward, new_observed_state_for_this_agent,
                                      last_observed_state_for_this_agent, feasible_actions_for_current_state)

        if (done): # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
            if(episode_num % vis_update_params['every_n_episodes'] == 0):
                if (done == 1):
                    episode_end_reason = "max steps"
                elif (done == 2):
                    episode_end_reason = "ambulance goal"
                elif (done == 3):  # TODO: #TOFIX: What should be the state here?
                    episode_end_reason = "agent goal"
                else:
                    raise ValueError(f"Episode: {episode_num} done  is True ={done} but reason not known !")

            print(f'E:{episode_num: <{6}}|S:{step: <{4}} : '
                  f'reward | {str(Proudhon.reward)[:min(5, len(str(Proudhon.reward)))]: <{5}}, '
                  f'lastAction: {chosen_action : <{12}} | '
                  f'cumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +
                  f' | state: {[str(x)[:5] + " " * max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]} | '
                  f'reason: {episode_end_reason: <{14}} ')
            print('-' * 134)
            break


        # 3.6: Feasibility check for current_state (for next step)
        feasible_actions_for_current_state = Proudhon.get_feasible_actions(vehicles_list[1])

        # 3.7: Actually Choose Action from feasible ones (for next step)
        chosen_action = RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent)

        # 3.8: Request environment to apply new action (Request action on Agent for next step)
        # Action is still not applied here, but on #3.2
        RB_RLAlgorithm.applyAction(chosen_action, vehicles_list[1])

    # Episode end
    sys.stdout.flush()


    # 4: Update Epsilon after episode is done
    RB_RLAlgorithm.epsilon = RB_RLAlgorithm.min_epsilon + (RB_RLAlgorithm.max_epsilon - RB_RLAlgorithm.min_epsilon) * \
                             np.exp(-RB_RLAlgorithm.decay_rate * episode_num)  # DONE: Change epsilone to update every episode not every iteration

    return RB_RLAlgorithm, Proudhon, episode_reward, episode_reward_list


if __name__ == "__main__":

    #TODO: print at the end of every episode/iteration inside epsisode -- according to visualization paramters from Config.py
    max_window = measure() #TODO: Make Q-table and assignments depend on max_window+10 from behind, and max_agent_velocity*3 forward

    options = get_options()

    if options.GUI:
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo')

    traci.start([sumoBinary, "-c", Sumocfg_DIR,
                             "--tripinfo-output", "tripinfo.xml"])

    for vehc in vehicles_list:
        vehc.initialize()

    episode_num = 0

    Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list = episode()
    total_reward_per_episode.append(episode_reward)
    reward_history_per_episode.append(episode_reward_list)


    traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml", "--start"])

    while(episode_num < max_num_episodes):
        episode_num += 1

        for vehc in vehicles_list:
            vehc.initialize()  # Placed here to set lane change mode ! Important !

        Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list = episode(Algorithm_for_RL, environment_for_next_episode, episode_num)
        total_reward_per_episode.append(episode_reward)
        reward_history_per_episode.append(episode_reward_list)

        # 5: Reset environment in preparation for next episode
        environment_for_next_episode.reset()

        # Load XMLs:
        traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml", "--start"])

