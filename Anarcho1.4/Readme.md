I (Waleed) used this document while working.

# Problems to solve:
- Insertion of cars -- where ? :: answer: probably after length + gap (mingap) from last vehicle?
- vehicles list must be updated after every step (and initialized?:: only if new vehicle)
- Agents must be able to leave the simulation without causing an error



# Things to take care of:
- naming of each car (just set a naming convention)

# Questions:
- Develop over which version? since no RL .. you know .. new version ?
- Does SUMO load the XML file once or every step?
- 

# Approach:
First run one vehicle without any RL -- then run many vehicles without RL (for this version) 
For next version: run many vehicles without RL first (given) -> then introduce RL to all cars (forced) -> Then introduce the option of RL/NO (or merge this step with the previous)

# GENERAL NOTES:
- 1  Maintain ambulance at __vehicles_list[0]__
- 2 Remember: After each traci.start() or traci.load() you must initialize vehicles (group in a function.. maybe?)
- 3 For version 1.5, make sure no_action sets next velocity to current velocity. Review the related github issue.

# 1: env.py
### 1.1: Vars
#### 1.1.2 Useful:
  * list_of_vehicles : list of all vehicles, agents and non-agents
  * agents: list of our agents, as opposed to emergency vehicle. (So, this should contain all the autonomous agents in version 1.4)
  * emer: emergency vehicle (Should be left as is)
  * count_emergency_vehicles: left as is
  * count_ego_vehicles: left as is 
#### 1.1.1 Useless: 
 name, amb_goal_dist, reward (current reward, regulalry updated), emer_start_lane, rel_amb_y_min, rel_amb_y_max, optimal_time (for ambulance to reach goal = track_len/emer.max_speed), max_steps (20*optimal_time)

#### 1.1.3 Useful in later versions:
Currently, these variables will be introduced in the __init__ function of the env class, but will not be updated in any other function.
* hidden_state: should be transformed to a list of hidden states per agent
* observed_state: should be transformed to a list of observed states per agent
* full_state: should be transformed to a list of full_states per agent 
* Actions: as is in next version
* action_to_string_dict: as is in next version

### 1.2 Functions
#### 1.2.1 Useful:
* reset: should change to initialize each agent in a new place  __~# MAIN FUNCTION TO CHANGE #~__
* measure_full_state: assigns hidden_state, observed_state, full_state for each agent (and measures the ambulance position in the process.. that's the only reason it is important in version 1.4 -- could possibly make the measurements for other agents as well .. but not sure if I should waste my time doing it in ver1.4)
* are_we_done: cases triggered by max_step (1) and ambulance (2) -- should be left as is. The case for the agent finishing should be edited to allow the agent to leave the simulation without causing an error  

#### 1.2.2 Useless:
get_emer_start_lane, get_vehicle_object_by_id, get_follow_speed_by_id, \_\_str\_\_

#### 1.2.3 Useful in later versions:
* get_feasible_actions: Should never be used in ver1.4
* calc_reward: Should never be used in ver1.4


# 2 SingleAgent.py:
This file could possibly be deleted from the code without causing any trouble. Please comment any line concerning it in the main simulation (by the comment #RLcomment .. but do not delete it)<br>
--- Check for lines that use RLAlgorithm and so one and comment them.

# 3 Config.py:
### 3.1: Vars
#### 3.1.1 Useful: 
* vehicles_list (__~# MAIN CHANGE !!!!! #~__) : should be set according to the logic in the XML file and env.reset() function
* Sumocfg_DIR: Directory for the sumo config file. Must change (5aleeha fe ela5er) (And the file contents must change)
* NET_FILE_PATH: Directory for the .net xml file. Must change (5aleeha fe ela5er) (And the file contents must change)
* ROUTE_FILE_PATH: Directory for the .rou xml file. Must change (5aleeha fe ela5er) (And the file contents must change)
* vis_update_params['test_mode_on']: should be forcibly put to True. If not, display a warning that it was set to False by the user and was changed to True since this is version 1.4 and no training exists, only Testing.
* total_reward_per_episode: should not be updated (#RLcomment)
* reward_history_per_episode: should not be updated (#RLcomment)
* q_learning_params: should not be updated (#RLcomment)
#### 3.1.2 Useless:
Sumo_random_seed, Python_random_seed, BASE_PATH, TEMPLATES_PATH, VARIABLES_FOLDER, track_len, SimTime (max simulation time), max_num_episodes, enable_checks
#### 3.1.3 Useful in later versions:
* give_final_reward

# 4 Checks.py:
__CHANGE__ are_we_ok logic to make sure it does not need to initialize Proudhon (the environment) again + change the name Proudhon inside the function. It just needs to check if the emergency car's lane is actually has changed --- make sure it is not called before an environment is initialized (i.e. not as are_we_ok() )

# 5  Anarcho.py :
This module has 3 functions. We explore them in the order they occur:
##### 5.1: get_options () : 
Leave as is. Useless.
##### 5.2 episode(RB_RLAlgorithm = None, Proudhon = None, episode_num = 0) :
* __#RLcomment__ (i.e. visit later in RL): Comment anything that has to do with RL_Algorithm/RB_RLAlgorithm
* __#RLcomment:__ Anything that has to do with rewarding, chosing action, feasibiltiy check ..etc 
* __#TempComment__: Observations, to be edited for our purposes. Possibly put: current ambulance velocity, lane, distance to goal ..etc 
###### 5.2.1 Useful in later versions:
* __new_observed_state_for_this_agent__ and similar variables: will be new observed state for multiple agents (goal from this variable was to save the new state at this step)

##### 5.3 \_\_main\_\_ :
#RLcomment what's necessary, which is not much (just saving, reward history stuff..etc)


# 6 Vehicle.py: 



# ---------------------------------------------------------- #
# Changed:
* in env.py: self.list_of_vehicles gets updated (i.e. vehicles get removed) when agents exit the lane... however, vehicles_list (the global variable), stays the same. Also added recount_vehicles in env.py 
* __IMPORANT__: Reorder the steps for done, reward, feasible actions ... etc __done check should be directly after simulation step__
* Moved SUMO-running-dependency from env.env.\_\_init\_\_ to env.reset ! More importantly: Added template filling to env.\_\_init\_\_ to have the .rou file edited before SUMO starts because vehicles initialization depends on it, so it happens as follows:
* * env.\_\_init\_\_ and template loading
* * SUMO starts
* * vehicles initialized
* * environment reset











 
  




