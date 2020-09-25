from Environment.env import env
from Utils.SimTools import SimTools
from sumolib import checkBinary
import numpy as np
from Config import *
import itertools
import traci


class Experiment:
    """
    How this works?
    1- An experiment sweeps over one variable, measuring another variable (one on the x, another on the y). Runs an episode with
    each x  value pair (x); N times, preparing for later averaging over the result over the N runs/episodes.
    2- An experiment consists of samples, a sample consists of an episode run for the specific (x) value. I.e. the
    result of the observation for an (x) value is calculated by averaging the results for the same (x) value over N samples
    (i.e. samples = runs = group of episodes till termination)
    3- The function define_experiment generates a list of samples-metadata (stored in dictionaries), for the function run_experiment
    to run them using run_sample(sample_metadata), N times (i.e. N calls for run_sample). (i.e.sample_metadata is stored in sample_dict, they are the same thing).
    #TODO let define_experiment return a **generator** of samples-metadata, instead of a **list** of samples-metadata
    4- The function run_experiment stores the result for each N iterations, averages the output observation after N sample of a certain sample
    metadata, and saves it to a csv file using the function save_to_csv.

    Notes:
        * Currently, the only obervation possible is the travel_time. Code is made ready to add later observations through the variable
        aggregate_yvarname_sum and following lines in function run_experiment.
    ----------

    How to use?
    0- Create an experiments list to loop over. (i.e. define experiment metadata)
    1- Create an experiment instance
    2- define an experiment via define_experiment
    3- run an experiment via run_experiment
    """

    def __init__(self, name="DefaultExperimentName", gui_on=False):
        self.current_sample = None
        self.exp_name = name
        self.sumoBinary = checkBinary('sumo') if not gui_on else checkBinary('sumo-gui')

    def run_sample(self, sample_dict, sample_num):
        self.current_sample = sample_dict
        sample_rl_p = sample_dict['rl_p']  # rl cars percentage
        sample_lane_busyness = sample_dict['lane_busyness']  # busyness of each lane
        sample_does_ambulance_change_lane_to_speed = sample_dict['does_amb_change']  # If True, ambulance changes lane to speed up.
        sample_start_position = sample_dict['start_pos']  # start position for vehicles. Either: "random", "middle", or a number from 40 to 250.

        sample_environment = env(self.sumoBinary, amb_to_change_lane=sample_does_ambulance_change_lane_to_speed,
                                 lane_busyness_list=sample_lane_busyness, rl_perecent_in=sample_rl_p,
                                 start_pos_for_agents=sample_start_position, name="ExperiementEnv1.0")

        sample_environment, amb_travel_time, _ = SimTools.experiment_episode(sample_environment, episode_num=sample_num)

        traci.close()

        return amb_travel_time


    def define_experiment(self, paramx, does_amb_change=False, start_pos="middle", default_rl_p=1.0, default_busyness=0.3):
        '''Helps user define an experiment by sweeping over two different paramters'''
        # xvarname = paramx[0]
        minx = paramx[1]
        maxx = paramx[2]
        stepx = paramx[3]
        paramx_list = list(np.arange(minx, maxx, stepx))

        experiment_samples_list = []

        for x in paramx_list:
            new_sample_metadata = dict()

            # First, put defaults
            new_sample_metadata['rl_p'] = default_rl_p  # rl cars percentage
            new_sample_metadata['lane_busyness'] = [default_busyness, default_busyness, default_busyness]  # busyness of each lane
            new_sample_metadata['does_amb_change'] = does_amb_change # If True, ambulance changes lane to speed up.
            new_sample_metadata['start_pos'] = start_pos # start position for vehicles. Either: "random", "middle", or a number from 40 to 250.

            # Make sure our input parameters are sweep-able
            assert (paramx[0] in new_sample_metadata.keys()), f"{paramx[0]}, x parameter, is a wrong parameter to sweep over. Check your spelling?." \
                                                         f"\n Possible keys are: {list(new_sample_metadata.keys())}."

            # Next, put what we want to loop on:
            new_sample_metadata[paramx[0]] = x
            if(paramx[0] == 'lane_busyness'): new_sample_metadata[paramx[0]] = [x]*num_lanes  # repeat for the number of lanes we have

            # Now, put the metadata in experiment_samples_list
            experiment_samples_list.append(new_sample_metadata)


        # return name of parameter x, name of parameter y, experiment_samples_list
        return paramx[0], experiment_samples_list


    def run_experiment(self, xvarname, experiment_samples_list, num_experiments_per_sample, yvarname="amb_travel_time"):
        ''' This function runs each sample N times, averages result over yvarname, and saves the output to csv file'''

        # Create empty csv file under experiment name (empty=column names only)
        with open(EXPERIMENT_RESULTS_FOLDER +'/'+self.exp_name+'.csv', 'w') as fd:
            fd.write(','.join([str(x) for x in experiment_samples_list[0].keys()]+[str(yvarname)]))
            fd.write('\n')


        for sample_num, sample_metadata_dict in enumerate(experiment_samples_list):
            print(f"Running dict data = {sample_metadata_dict}")

            aggregate_yvarname_sum = 0  # sum to be later divided  by N to get the average value
            for i in range(num_experiments_per_sample):
                amb_travel_time = self.run_sample(sample_dict=sample_metadata_dict, sample_num=sample_num)

                assert(yvarname=="amb_travel_time"), f"aggregating over {yvarname} is not yet implemented in Experiment.run_experiment()"
                aggregate_yvarname_sum+=amb_travel_time

            averaged_yvarname = aggregate_yvarname_sum/num_experiments_per_sample

            print(f"Saving dict data = {sample_metadata_dict}")
            self.save_to_csv(sample_metadata_dict, averaged_yvarname)   # Save every N=num_experiments_per_sample (after every averaged observation)
            print(f"Saved dict data = {sample_metadata_dict}")




    def save_to_csv(self, sample_dict, averaged_observation):

        # Append data row to csv files
        list1 = []
        with open(EXPERIMENT_RESULTS_FOLDER +'/'+self.exp_name+'.csv', 'a') as fd:
            fd.write(','.join([str(x) if (type(x) != list) else str(x[0]) for x in sample_dict.values() ]+[str(averaged_observation)]))
            fd.write('\n')









