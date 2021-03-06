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
from Checks import *

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# to be moved to global variables or to be converted to class variable




def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--GUI", action="store_true",
                          default=False, help="run the GUI version of sumo")
    options, args = opt_parser.parse_args()
    return options


def episode(RB_RLAlgorithm=None, Proudhon=None, episode_num=0):
    ########################
    # 1 inits
    done = False  # are we done with the episode or not
    step = 0  # step number
    errors_arrays = np.zeros(7)    # for error checking capability

    if (Proudhon is None):
        Proudhon = env(vehicles_list)  # vehicles_list = [LH, RB]
        ## -- ##
        Proudhon.reset()
        traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml"])
        for vehc in vehicles_list:
            vehc.initialize()
        ## -- ##

    if (RB_RLAlgorithm is None):
        algo_params = q_learning_params  # from Config.py
        RB_RLAlgorithm = RLAlgorithm(Proudhon, algo_params=algo_params, load_q_table=load_q_table,
                                     test_mode_on=vis_update_params['test_mode_on'])  # Algorithm for RB Agent
    ## ######################

    ########################
    # 2 init measurements
    traci.simulationStep()  # After stepping
    Proudhon.get_emer_start_lane()

    # (communication from vehicle):
    getters(vehicles_list)  # measure all values from environment
    # (communication to environment):
    Proudhon.measure_full_state()  # measure all values into our agents
    # (communication to algorithm/agent):
    new_observed_state_for_this_agent = Proudhon.observed_state[0]

    # rl_disengagement
    # Chose Action from Feasible Actions:
    # chosen action must be none , feasible actions []
    feasible_actions_for_current_state = Proudhon.get_feasible_actions(vehicles_list[1], new_observed_state_for_this_agent)
    chosen_action = RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent)
    RB_RLAlgorithm.applyAction(chosen_action, vehicles_list[1],new_observed_state_for_this_agent)  # Request Action on Agent

    episode_reward = 0
    episode_reward_list = []
    ########################
    # rl_disengagement_end

    # 3: MAIN LOOP
    if (episode_num % vis_update_params['every_n_episodes'] == 0):
        print(f'E:{episode_num: <{6}}|S:{0: <{4}} | '
              f'epsilon: {RB_RLAlgorithm.epsilon: <{31}} | '
              f'state: {[str(x)[:5] + " " * max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]} |')

    while traci.simulation.getMinExpectedNumber() > 0:

        # 3.1: Store last states
        amb_last_velocity = Proudhon.emer.spd
        last_observed_state = Proudhon.observed_state
        last_observed_state_for_this_agent = last_observed_state[0]
        # ----------------------------------------------------------------#
        # 3.1.1: Store last picked action
        # rl_disengagement
        last_picked_action = chosen_action  # check_related_fcn #
        #

        # ----------------------------------------------------------------- #
        # 3.2:   M O V E      O N E      S I M U L A T I O N       S T E P
        # ----------------------------------------------------------------- #
        traci.simulationStep()  # actual action applying
        step += 1

        # TODO: Turn this into are_we_ok function
        if (vehicles_list[0].getL() != Proudhon.emer_start_lane):
            raise ValueError(
                f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {vehicles_list[0].getL()} on step {step}. "
                f"\nAmbulance Should not change lane. Quitting.")

        # ----------------------------------------------------------------- #

        # 3.3: measurements and if we are done check
        getters(vehicles_list)
        Proudhon.measure_full_state()
        new_observed_state_for_this_agent = Proudhon.observed_state[0]

        done = Proudhon.are_we_done(full_state=Proudhon.full_state, step_number=step)
        if (step % vis_update_params['every_n_iters'] == 0 and episode_num % vis_update_params[
            'every_n_episodes'] == 0):  # print step info

            print(f'E:{episode_num: <{6}}|S:{step: <{4}} | '
                  f'reward : {Proudhon.reward},'
                  f'lastAction: {chosen_action : <{12}} | '
                  #f'cumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +
                  f' | state: {[str(x)[:5] + " " * max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]}, '
                  f'actionMethod: {RB_RLAlgorithm.action_chosing_method : <{14}}')


        #rl_disengagement modify RB_RLAlgorithm.action_chosing_method to be null , Proudhon.reward should equal zero

        # 3.3.1: checking if we are ok
        # rl_disengagement  two conditions rl_enabled or not
        #
        if (enable_checks):
            errors_arrays += are_we_ok(1, True, Proudhon, step, last_picked_action, last_observed_state_for_this_agent,
                      new_observed_state_for_this_agent , Proudhon)

        # 3.4: reward last step's chosen action
        # rl_disengagement reward should be null , reward_list no append
        reward = Proudhon.calc_reward(amb_last_velocity, done, step,last_observed_state_for_this_agent,new_observed_state_for_this_agent)
        rel_distance = new_observed_state_for_this_agent[4]
        if (enable_rl_dis_engagement & (~ ((rel_distance < Proudhon.rel_amb_y_max) & (rel_distance > Proudhon.rel_amb_y_min)))):
             # if  the agent is  outside ambulance window and rl _disenagement is enabled  , do not add this step reward(equals None) to episode reward
            pass
        else:
            episode_reward += reward  # for history
            episode_reward_list.append(reward)  # for history

        # 3.6: Feasibility check for current_state (for next step)
        # rl_disengagement  empty list []
        feasible_actions_for_current_state = Proudhon.get_feasible_actions(vehicles_list[1],new_observed_state_for_this_agent)

        # 3.5: update q table using backward reward logic
        # rl_disengagement  should not be updated anyway
        RB_RLAlgorithm.update_q_table(chosen_action, reward, new_observed_state_for_this_agent,
                                      last_observed_state_for_this_agent, feasible_actions_for_current_state)

        if (done):  # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
            if (episode_num % vis_update_params['every_n_episodes'] == 0):
                if ( done == 1):  # TODO: Remove episode_end_reason outsisde the print check -- we might need it elsewehere
                    episode_end_reason = "max steps"
                elif (done == 2):
                    episode_end_reason = "ambulance goal"
                elif (done == 3):  # TODO: #TOFIX: What should be the state here?
                    episode_end_reason = "agent goal"
                else:
                    raise ValueError(f"Episode: {episode_num} done  is True ={done} but reason not known !")

                print(f'E:{episode_num: <{6}}|S:{step: <{4}} | '
                      f'reward : {Proudhon.reward},'
                      f'lastAction: {chosen_action : <{12}} | '
                      #f'cumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +
                      f' | state: {[str(x)[:5] + " " * max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]}, '
                      f'actionMethod: {RB_RLAlgorithm.action_chosing_method : <{14}}') ## rl_disengagement


                # rl_disengagement _modified
                if(enable_checks):


                    print( "non_requested_lane_change , acc_errors , dec_errors , _no_acc_errors , non_requested_speed_change ,change_left_errors , change_right_errors")

                    print(f"Error_happened_during_simulation {errors_arrays}")
            break

            # 3.7: Actually Choose Action from feasible ones (for next step)
            ## rl_disengagement  _modified
        chosen_action = RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent)

        # 3.8: Request environment to apply new action (Request action on Agent for next step)
        # Action is still not applied here, but on #3.2
        ## rl_disengagement  _modified
        RB_RLAlgorithm.applyAction(chosen_action, vehicles_list[1],new_observed_state_for_this_agent)

    # Episode end
    sys.stdout.flush()

    # 4: Update Epsilon after episode is done
    old_epsilon = RB_RLAlgorithm.epsilon
    RB_RLAlgorithm.epsilon = RB_RLAlgorithm.min_epsilon + (RB_RLAlgorithm.max_epsilon - RB_RLAlgorithm.min_epsilon) * \
                             np.exp(
                                 -RB_RLAlgorithm.decay_rate * episode_num)  # DONE: Change epsilon to update every episode not every iteration

    if (episode_num % vis_update_params['every_n_episodes'] == 0):
        print(f'\n\nE:{episode_num: <{6}}| END:{step: <{4}} | '
              f'finalCumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) + " | "
                                                                                                           f'reason: {episode_end_reason: <{15}} | '
                                                                                                           f'old_eps: {old_epsilon: <{10}}, '
                                                                                                           f'new_eps: {RB_RLAlgorithm.epsilon: <{10}}')
        print('-' * 157)
        print('=' * 157)
        print('\n')

    if (vis_update_params['print_reward_every_episode'] and episode_num % vis_update_params['every_n_episodes'] != 0):
        print(f'E:{episode_num: <{6}}| END:{step: <{4}} | '
              f'finalCumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) + " | ")

    return RB_RLAlgorithm, Proudhon, episode_reward, episode_reward_list, errors_arrays


if __name__ == "__main__":

    # TODO: print at the end of every episode/iteration inside epsisode -- according to visualization paramters from Config.py
    # max_window = measure() #TODO: Make Q-table and assignments depend on max_window+10 from behind, and max_agent_velocity*3 forward

    cumulative_errors_arrays = []

    options = get_options()

    if (options.GUI or vis_update_params['test_mode_on']):
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo')

    ## ----- ##
    traci.start([sumoBinary, "-c", Sumocfg_DIR,
                 "--tripinfo-output", "tripinfo.xml"])
    for vehc in vehicles_list:
        vehc.initialize()
    ## ----- ##

    episode_num = 0

    Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list, errors_arrays = episode()
    total_reward_per_episode.append(episode_reward) ### rl_disengagement should not be a concern empty list
    reward_history_per_episode.append(episode_reward_list)

    ## --- ##
    environment_for_next_episode.reset()
    traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml", "--start"])  # , "--start"
    for vehc in vehicles_list:
        vehc.initialize()  # Placed here to set lane change mode ! Important !
    ## --- ##

    while (episode_num < max_num_episodes):

        episode_num += 1

        Algorithm_for_RL, environment_for_next_episode, episode_reward, episode_reward_list, errors_arrays = episode(Algorithm_for_RL,
                                                                                                      environment_for_next_episode,
                                                                                                      episode_num)
        total_reward_per_episode.append(episode_reward)
        reward_history_per_episode.append(episode_reward_list)

        cumulative_errors_arrays.append(errors_arrays) ## rl_disengagement  _modified

        ##  --  ##

        ## -- ##
        # 5: Reset environment in preparation for next episode
        environment_for_next_episode.reset()
        # Load XMLs:
        traci.load(
            ["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml", "--start", "--message-log", "--no-step-log"])
        # TODO: https://stackoverflow.com/questions/59166732/how-to-disable-print-loading-configuration-done-in-sumo-traci and stop printing termination step number
        for vehc in vehicles_list:
            vehc.initialize()  # Placed here to set lane change mode ! Important !
        ## -- ##

    # Save Q-table after episodes ended:
    Algorithm_for_RL.save_q_table()
    total_errors = sum(cumulative_errors_arrays, 0) ## rl_disengagement  _modified
    print("Total errors ", total_errors)