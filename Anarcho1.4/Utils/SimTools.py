from Config import *  # Make sure this is the first one imported, to have random.seed() used before any actual randomness is assigned
from Checks import *
import traci
from RL.SingleAgent import RLAlgorithm
import jinja2  # for templates

class SimTools():
    @staticmethod
    def templates_reset():
        # ---------------------------------------------------------------------------- #
        #         R A N D O M L Y      I N I T I A L I Z E       X M L s
        # ---------------------------------------------------------------------------- #
        new_stance = dict()
        # 2.1: Init Agent Start Lane
        new_stance['agent_start_lane'] = random.randint(0,
                                                        2)  # TODO: edit this to automatically retrieve the number of lanes

        # 2.2: Init Emergency Start Lane
        new_stance['ambulance_start_lane'] = random.randint(0,
                                                            2)  # TODO: edit this to automatically retrieve the number of lanes

        # 2.3: Init Agent Start Position
        new_stance['agent_start_position'] = random.randint(41,
                                                            250)  # TODO: Edit minimum position to depend on starting position
        new_stance['r1_new_length'] = new_stance['agent_start_position']
        new_stance['r2_new_length'] = 511 - new_stance['agent_start_position']

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

    @staticmethod
    def episode(RB_RLAlgorithm=None, Proudhon=None, episode_num=0):
        ########################
        # 1 inits
        done = False  # are we done with the episode or not
        step = 0  # step number
        if (Proudhon is None):
            Proudhon = env(vehicles_list)  # vehicles_list = [LH, RB]
            ## -- ##
            traci.load(["-c", Sumocfg_DIR, "--tripinfo-output", "tripinfo.xml", "--seed", str(Sumo_random_seed)])
            for vehc in vehicles_list:
                vehc.initialize()
            ## -- ##

        Proudhon.reset()

        # if(RB_RLAlgorithm is None): #RLcomment
        #     algo_params = q_learning_params  # from Config.py #RLcomment
        #     RB_RLAlgorithm = RLAlgorithm(Proudhon, algo_params= algo_params,
        #     load_q_table = load_q_table, test_mode_on = vis_update_params['test_mode_on'])  # Algorithm for RB Agent #RLcomment
        # ## ######################

        ########################
        # 2 init measurements
        traci.simulationStep()  # After stepping
        Proudhon.get_emer_start_lane()

        # (communication from vehicle):
        getters(Proudhon.list_of_vehicles)  # measure all values from environment
        # (communication to environment):
        Proudhon.measure_full_state()  # measure all values into our agents
        # (communication to algorithm/agent):
        # new_observed_state_for_this_agent = Proudhon.observed_state[0]    #RLcomment

        # Chose Action from Feasible Actions:
        # feasible_actions_for_current_state = Proudhon.get_feasible_actions(vehicles_list[1]) #RLcomment
        # chosen_action = RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent) #RLcomment
        # RB_RLAlgorithm.applyAction(chosen_action, vehicles_list[1])  # Request Action on Agent #RLcomment

        # episode_reward = 0    #RLcomment
        # episode_reward_list = []  #RLcomment
        ########################

        # 3: MAIN LOOP
        # if (episode_num % vis_update_params['every_n_episodes'] == 0): #TempComment
        #     print(f'E:{episode_num: <{6}}|S:{0: <{4}} | ' #TempComment
        #           f'epsilon: {RB_RLAlgorithm.epsilon: <{31}} | ' #TempComment
        #           f'state: {[str(x)[:5] + " " * max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]} |') #TempComment

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
            if (vehicles_list[0].getL() != Proudhon.emer_start_lane):
                raise ValueError(
                    f"Ambulance Changed lane from {Proudhon.emer_start_lane} to {vehicles_list[0].getL()} on step {step}. "
                    f"\nAmbulance Should not change lane. Quitting.")

            # if (step % vis_update_params['every_n_iters'] == 0 and episode_num % vis_update_params['every_n_episodes'] == 0): # print step info   #TempComment
            #     print(f'E:{episode_num: <{6}}|S:{step: <{4}} | ' #TempComment
            #           f'reward : {str(Proudhon.reward)[:min(5,len(str(Proudhon.reward)))]: <{5}}, ' #TempComment
            #           f'lastAction: {chosen_action : <{12}} | '   #TempComment
            #           f'cumReward: ' + str(episode_reward)[:6] + ' '*max(0, 6 - len(str(episode_reward))) + #TempComment
            #           f' | state: {[str(x)[:5]+" "*max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]}, '  #TempComment
            #           f'actionMethod: {RB_RLAlgorithm.action_chosing_method : <{14}}')  #TempComment
            # ----------------------------------------------------------------- #

            # 3.3: measurements and if we are done check
            getters(Proudhon.list_of_vehicles)
            Proudhon.measure_full_state()
            # new_observed_state_for_this_agent = Proudhon.observed_state[0]  #RLcomment

            done = Proudhon.are_we_done(full_state=Proudhon.full_state, step_number=step)

            # 3.4: reward last step's chosen action
            # reward = Proudhon.calc_reward(amb_last_velocity, done, step)  #RLcomment
            # episode_reward += reward  # for history #RLcomment
            # episode_reward_list.append(reward)  # for history #RLcomment

            # 3.6: Feasibility check for current_state (for next step)
            # feasible_actions_for_current_state = Proudhon.get_feasible_actions(vehicles_list[1]) #RLcomment

            # 3.5: update q table using backward reward logic
            # RB_RLAlgorithm.update_q_table(chosen_action, reward, new_observed_state_for_this_agent,  #RLcomment
            #                               last_observed_state_for_this_agent, feasible_actions_for_current_state)  #RLcomment

            if (done):  # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
                if (episode_num % vis_update_params['every_n_episodes'] == 0):
                    if (
                            done == 1):  # TODO: Remove episode_end_reason outsisde the print check -- we might need it elsewehere
                        episode_end_reason = "max steps"
                    elif (done == 2):
                        episode_end_reason = "ambulance goal"
                    elif (done == 3):  # TODO: #TOFIX: What should be the state here?
                        episode_end_reason = "agent goal"  # TempComment
                    else:
                        raise ValueError(f"Episode: {episode_num} done  is True ={done} but reason not known !")

                    # print(f'E:{episode_num: <{6}}|S:{step: <{4}} | '  #TempComment
                    #       f'reward : {str(Proudhon.reward)[:min(5, len(str(Proudhon.reward)))]: <{5}}, '  #TempComment
                    #       f'lastAction: {chosen_action : <{12}} | '  #TempComment
                    #       f'cumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +  #TempComment
                    #       f' | state: {[str(x)[:5] + " " * max(0, 5 - len(str(x))) for x in Proudhon.observed_state[0]]}, '  #TempComment
                    #       f'actionMethod: {RB_RLAlgorithm.action_chosing_method : <{14}}')  #TempComment
                break

                # 3.7: Actually Choose Action from feasible ones (for next step)
            # chosen_action = RB_RLAlgorithm.pickAction(feasible_actions_for_current_state, new_observed_state_for_this_agent)  #RLcomment

            # 3.8: Request environment to apply new action (Request action on Agent for next step)
            # Action is still not applied here, but on #3.2
            # RB_RLAlgorithm.applyAction(chosen_action, vehicles_list[1])   #RLcomment

        # Episode end
        sys.stdout.flush()

        # 4: Update Epsilon after episode is done
        # old_epsilon = RB_RLAlgorithm.epsilon  #RLcomment
        # RB_RLAlgorithm.epsilon = RB_RLAlgorithm.min_epsilon + (RB_RLAlgorithm.max_epsilon - RB_RLAlgorithm.min_epsilon) * \   #RLcomment
        #                         np.exp(-RB_RLAlgorithm.decay_rate * episode_num)  # DONE: Change epsilon to update every episode not every iteration  #RLcomment

        # if (episode_num % vis_update_params['every_n_episodes'] == 0):    #TempComment
        #     print(f'\n\nE:{episode_num: <{6}}| END:{step: <{4}} | '   #TempComment
        #           f'finalCumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +" | "   #TempComment
        #           f'reason: {episode_end_reason: <{15}} | '   #TempComment
        #           f'old_eps: {old_epsilon: <{10}}, '  #TempComment
        #           f'new_eps: {RB_RLAlgorithm.epsilon: <{10}}')    #TempComment
        #     print('-'*157)    #TempComment
        #     print('=' * 157)  #TempComment
        #     print('\n')   #TempComment

        if (vis_update_params['print_reward_every_episode'] and episode_num % vis_update_params[
            'every_n_episodes'] != 0):
            print(f'E:{episode_num: <{6}}| END:{step: <{4}} | ')
            #      f'finalCumReward: ' + str(episode_reward)[:6] + ' ' * max(0, 6 - len(str(episode_reward))) +" | ")   #RLcomment

        # return RB_RLAlgorithm, Proudhon, episode_reward, episode_reward_list  #RLcomment
        return None, Proudhon, None, None

