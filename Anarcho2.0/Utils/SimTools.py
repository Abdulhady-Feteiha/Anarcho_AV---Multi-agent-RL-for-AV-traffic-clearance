from Config import *  # Make sure this is the first one imported, to have random.seed() used before any actual randomness is assigned
import traci
from Environment.env import *
from Utils.helpers import *
import sys




class SimTools():

    @staticmethod
    def episode(sumoBinary, Proudhon=None, episode_num=0):
        ########################
        # 1 inits
        done = False  # are we done with the episode or not
        step = 0  # step number
        ########################

        # 2 init measurements
        traci.simulationStep()  # After stepping
        Proudhon.get_emer_start_lane()

        # (communication from SUMO to vehicle):
        # getters(Proudhon.list_of_vehicles)  # measure all values from environment (local variable.. since some vehicles exited)
        # Commented because getters is called inside Proudhon.measure_full_state()
        # (communication from vehicle to environment):
        Proudhon.measure_full_state()  # measure all values into our agents
        # (communication to algorithm/agent):
        new_observed_state = Proudhon.observed_state   #RLcomment # Was: new_observed_state_for_this_agent

        # Chose Action from Feasible Actions:
        # It's ok that we get feasible actions and pick actions for ambulance vehicle because they will never be applied since it's Krauss model controlled
        feasible_actions_for_current_state = [Proudhon.get_feasible_actions(Proudhon.list_of_vehicles[i])
                                              for i in range(len(Proudhon.list_of_vehicles))] #RLcomment
        chosen_action = [Proudhon.list_of_vehicles[i+1].control_algorithm.pickAction(
                                                               feasible_actions_for_current_state[i+1], new_observed_state[i])
                         for i in range(len(Proudhon.list_of_vehicles)-1)]
                          #RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent) #RLcomment

        # Action is still not applied here, but on #3.2
        for i, vhcl in enumerate(Proudhon.list_of_vehicles):
            if (vhcl.type != "Emergency"):
                vhcl.control_algorithm.applyAction(chosen_action[i - 1], vhcl)
        # RB_RLAlgorithm.applyAction(chosen_action, Proudhon.list_of_vehicles[1])  # Request Action on Agent #RLcomment

        # episode_reward = 0    #RLcomment
        # episode_reward_list = []  #RLcomment
        ########################

        # 3: MAIN LOOP
        if (episode_num % vis_update_params['every_n_episodes'] == 0): #TempComment
            print(f'E:{episode_num: <{6}}|S:{0: <{4}} | '
                  f'MaxPossible: {Proudhon.max_possible_cars: <{4}} | '
                  f'ActualPerLane: { [ vehicles_data[i] for i in range(num_lanes) ] } |'
                  f'NumVehicles: {fill_str(str(len(Proudhon.list_of_vehicles)), 5)}')
        while traci.simulation.getMinExpectedNumber() > 0:

            # 3.1: Store last states
            amb_last_velocity = Proudhon.emer.spd
            # last_observed_state = Proudhon.observed_state #RLComment
            # last_observed_state_for_this_agent = last_observed_state #RLcomment

            # ----------------------------------------------------------------- #
            # 3.2:   M O V E      O N E      S I M U L A T I O N       S T E P
            # ----------------------------------------------------------------- #
            traci.simulationStep()  # actual action applying
            step += 1

            # TODO: Turn this into are_we_ok function
            if (Proudhon.list_of_vehicles[0].getL() != Proudhon.emer_start_lane and enable_checks):
                raise ValueError(
                    f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {Proudhon.list_of_vehicles[0].getL()} on step {step}. "
                    f"\nAmbulance Should not change lane. Quitting.")

            if (step % vis_update_params['every_n_iters'] == 0 and episode_num % vis_update_params['every_n_episodes'] == 0): # print step info   #TempComment
                print(f'E:{episode_num: <{6}}|S:{step: <{4}} |' #TempComment
                      f'EmerVel: {fill_str(str(Proudhon.emer.spd), 5)} |'
                      f'EmerGoalDist: {fill_str(str(Proudhon.amb_goal_dist-Proudhon.emer.lane_pose), 5)} |'
                      f'NumVehicles: {fill_str(str(len(Proudhon.list_of_vehicles)), 5)}')

            # ----------------------------------------------------------------- #

            # 3.3: measurements and if we are done check
            # getters(Proudhon.list_of_vehicles)  # Commented because now getters is inside Proudhon.measure_full_state
            Proudhon.measure_full_state()
            new_observed_state = Proudhon.observed_state  #RLcomment

            done = Proudhon.are_we_done(step_number=step)

            # 3.4: reward last step's chosen action
            # reward = Proudhon.calc_reward(amb_last_velocity, done, step)  #RLcomment
            # episode_reward += reward  # for history #RLcomment
            # episode_reward_list.append(reward)  # for history #RLcomment

            # 3.6: Feasibility check for current_state (for next step)
            feasible_actions_for_current_state = [Proudhon.get_feasible_actions(Proudhon.list_of_vehicles[i])
                                              for i in range(len(Proudhon.list_of_vehicles))]

            # 3.5: update q table using backward reward logic
            # RB_RLAlgorithm.update_q_table(chosen_action, reward, new_observed_state_for_this_agent,  #RLcomment
            #                               last_observed_state_for_this_agent, feasible_actions_for_current_state)  #RLcomment

            if (done):  # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
                if (episode_num % vis_update_params['every_n_episodes'] == 0):
                    if (done == 1):  # TODO: Remove episode_end_reason outsisde the print check -- we might need it elsewehere
                        episode_end_reason = "max steps"
                    elif (done == 2):
                        episode_end_reason = "ambulance goal"
                    else:
                        raise ValueError(f"Episode: {episode_num} done  is True ={done} but reason not known !")

                    print(f'E:{episode_num: <{6}}|EndStep:{step: <{4}}')
                break

                # 3.7: Actually Choose Action from feasible ones (for next step)
            chosen_action = [Proudhon.list_of_vehicles[i+1].control_algorithm.pickAction(
                                                               feasible_actions_for_current_state[i+1], new_observed_state[i])
                         for i in range(len(Proudhon.list_of_vehicles)-1)]

            # 3.8: Request environment to apply new action (Request action on Agent for next step)
            # Action is still not applied here, but on #3.2
            for i, vhcl in enumerate(Proudhon.list_of_vehicles):
                if (vhcl.type != "Emergency"):
                    vhcl.control_algorithm.applyAction(chosen_action[i-1], vhcl)

        # Episode end
        sys.stdout.flush()

        # 4: Update Epsilon after episode is done
        # old_epsilon = RB_RLAlgorithm.epsilon  #RLcomment
        # RB_RLAlgorithm.epsilon = RB_RLAlgorithm.min_epsilon + (RB_RLAlgorithm.max_epsilon - RB_RLAlgorithm.min_epsilon) * \   #RLcomment
        #                         np.exp(-RB_RLAlgorithm.decay_rate * episode_num)  # DONE: Change epsilon to update every episode not every iteration  #RLcomment

        if (episode_num % vis_update_params['every_n_episodes'] == 0):
            print(f'\n\nE:{episode_num: <{6}}| END:{step: <{4}} |'          
                  f'reason: {episode_end_reason: <{15}}')
            print('-'*157)
            print('=' * 157)
            print('\n')

        if (vis_update_params['print_reward_every_episode'] and episode_num % vis_update_params['every_n_episodes'] != 0):
            print(f'E:{episode_num: <{6}}| END:{step: <{4}} | ')
            #      f'finalCumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +" | ")   #RLcomment

        # return RB_RLAlgorithm, Proudhon, episode_reward, episode_reward_list  #RLcomment
        return  Proudhon, None, None


    @staticmethod
    def experiment_episode(Proudhon, episode_num=0):
        ''' A version of the episode function, but for running sample from experiments. One experiment episode is
        also known as one sample.'''
        ########################
        # 1 inits
        done = False  # are we done with the episode or not
        step = 0  # step number
        ########################

        # 2 init measurements
        traci.simulationStep()  # After stepping
        Proudhon.get_emer_start_lane()

        # (communication from SUMO to vehicle):
        # getters(Proudhon.list_of_vehicles)  # measure all values from environment (local variable.. since some vehicles exited)
        # Commented because getters is called inside Proudhon.measure_full_state()
        # (communication from vehicle to environment):
        Proudhon.measure_full_state()  # measure all values into our agents
        # (communication to algorithm/agent):
        new_observed_state = Proudhon.observed_state   #RLcomment # Was: new_observed_state_for_this_agent

        # Chose Action from Feasible Actions:
        # It's ok that we get feasible actions and pick actions for ambulance vehicle because they will never be applied since it's Krauss model controlled
        feasible_actions_for_current_state = [Proudhon.get_feasible_actions(Proudhon.list_of_vehicles[i])
                                              for i in range(len(Proudhon.list_of_vehicles))] #RLcomment
        chosen_action = [Proudhon.list_of_vehicles[i+1].control_algorithm.pickAction(
                                                               feasible_actions_for_current_state[i+1], new_observed_state[i])
                         for i in range(len(Proudhon.list_of_vehicles)-1)]
                          #RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent) #RLcomment

        # Action is still not applied here, but on #3.2
        for i, vhcl in enumerate(Proudhon.list_of_vehicles):
            if (vhcl.type != "Emergency"):
                vhcl.control_algorithm.applyAction(chosen_action[i - 1], vhcl)
        # RB_RLAlgorithm.applyAction(chosen_action, Proudhon.list_of_vehicles[1])  # Request Action on Agent #RLcomment

        # episode_reward = 0    #RLcomment
        # episode_reward_list = []  #RLcomment
        ########################

        # 3: MAIN LOOP
        if (episode_num % vis_update_params['every_n_episodes'] == 0): #TempComment
            print(f'E:{episode_num: <{6}}|S:{0: <{4}} | '
                  f'MaxPossible: {Proudhon.max_possible_cars: <{4}} | '
                  f'ActualPerLane: { [ vehicles_data[i] for i in range(num_lanes) ] } |'
                  f'NumVehicles: {fill_str(str(len(Proudhon.list_of_vehicles)), 5)}')
        while traci.simulation.getMinExpectedNumber() > 0:

            # 3.1: Store last states
            amb_last_velocity = Proudhon.emer.spd
            # last_observed_state = Proudhon.observed_state #RLComment
            # last_observed_state_for_this_agent = last_observed_state #RLcomment

            # ----------------------------------------------------------------- #
            # 3.2:   M O V E      O N E      S I M U L A T I O N       S T E P
            # ----------------------------------------------------------------- #
            traci.simulationStep()  # actual action applying
            step += 1

            # TODO: Turn this into are_we_ok function
            if (Proudhon.list_of_vehicles[0].getL() != Proudhon.emer_start_lane and enable_checks):
                raise ValueError(
                    f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {Proudhon.list_of_vehicles[0].getL()} on step {step}. "
                    f"\nAmbulance Should not change lane. Quitting.")

            if (step % vis_update_params['every_n_iters'] == 0 and episode_num % vis_update_params['every_n_episodes'] == 0): # print step info   #TempComment
                print(f'E:{episode_num: <{6}}|S:{step: <{4}} |' #TempComment
                      f'EmerVel: {fill_str(str(Proudhon.emer.spd), 5)} |'
                      f'EmerGoalDist: {fill_str(str(Proudhon.amb_goal_dist-Proudhon.emer.lane_pose), 5)} |'
                      f'NumVehicles: {fill_str(str(len(Proudhon.list_of_vehicles)), 5)}')

            # ----------------------------------------------------------------- #

            # 3.3: measurements and if we are done check
            # getters(Proudhon.list_of_vehicles)  # Commented because now getters is inside Proudhon.measure_full_state
            Proudhon.measure_full_state()
            new_observed_state = Proudhon.observed_state  #RLcomment

            done = Proudhon.are_we_done(step_number=step)

            # 3.4: reward last step's chosen action
            # reward = Proudhon.calc_reward(amb_last_velocity, done, step)  #RLcomment
            # episode_reward += reward  # for history #RLcomment
            # episode_reward_list.append(reward)  # for history #RLcomment

            # 3.6: Feasibility check for current_state (for next step)
            feasible_actions_for_current_state = [Proudhon.get_feasible_actions(Proudhon.list_of_vehicles[i])
                                              for i in range(len(Proudhon.list_of_vehicles))]

            # 3.5: update q table using backward reward logic
            # RB_RLAlgorithm.update_q_table(chosen_action, reward, new_observed_state_for_this_agent,  #RLcomment
            #                               last_observed_state_for_this_agent, feasible_actions_for_current_state)  #RLcomment

            if (done):  # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
                if (episode_num % vis_update_params['every_n_episodes'] == 0):
                    if (done == 1):  # TODO: Remove episode_end_reason outsisde the print check -- we might need it elsewehere
                        episode_end_reason = "max steps"
                    elif (done == 2):
                        episode_end_reason = "ambulance goal"
                    else:
                        raise ValueError(f"Episode: {episode_num} done  is True ={done} but reason not known !")

                    print(f'E:{episode_num: <{6}}|EndStep:{step: <{4}}')
                break

                # 3.7: Actually Choose Action from feasible ones (for next step)
            chosen_action = [Proudhon.list_of_vehicles[i+1].control_algorithm.pickAction(
                                                               feasible_actions_for_current_state[i+1], new_observed_state[i])
                         for i in range(len(Proudhon.list_of_vehicles)-1)]

            # 3.8: Request environment to apply new action (Request action on Agent for next step)
            # Action is still not applied here, but on #3.2
            for i, vhcl in enumerate(Proudhon.list_of_vehicles):
                if (vhcl.type != "Emergency"):
                    vhcl.control_algorithm.applyAction(chosen_action[i-1], vhcl)

        # Episode end
        sys.stdout.flush()

        # 4: Update Epsilon after episode is done
        # old_epsilon = RB_RLAlgorithm.epsilon  #RLcomment
        # RB_RLAlgorithm.epsilon = RB_RLAlgorithm.min_epsilon + (RB_RLAlgorithm.max_epsilon - RB_RLAlgorithm.min_epsilon) * \   #RLcomment
        #                         np.exp(-RB_RLAlgorithm.decay_rate * episode_num)  # DONE: Change epsilon to update every episode not every iteration  #RLcomment

        if (episode_num % vis_update_params['every_n_episodes'] == 0):
            print(f'\n\nE:{episode_num: <{6}}| END:{step: <{4}} |'          
                  f'reason: {episode_end_reason: <{15}}')
            print('-'*157)
            print('=' * 157)
            print('\n')

        if (vis_update_params['print_reward_every_episode'] and episode_num % vis_update_params['every_n_episodes'] != 0):
            print(f'E:{episode_num: <{6}}| END:{step: <{4}} | ')
            #      f'finalCumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +" | ")   #RLcomment

        # return RB_RLAlgorithm, Proudhon, episode_reward, episode_reward_list  #RLcomment
        return  Proudhon, step, None
