import os
import sys
from Utils.Experiment import Experiment



if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")



if __name__ == "__main__":
    # 0 - Create an experiments list to loop over. (i.e. define experiment metadata)
    # You do not need to edit steps 1-3. Just copy an experiment dictionary
    # TODO: Generate an experiment name from experiment metadata
    N = 100  # Number of sample runs to average over.

    exp1 = {"name": "travel_time_vs_rl_p_amb_changes_0p1_busyness", "xvarname": "rl_p", "minx": 0.0,
            "maxx": 1.1, "stepx": 0.1, "default_rl_p": 1.0, "does_amb_change": True, "default_busyness": 0.1 }
    exp2 = {"name": "travel_time_vs_rl_p_amb_changes_0p3_busyness", "xvarname": "rl_p",
            "minx": 0.0, "maxx": 1.1, "stepx": 0.1, "default_rl_p": 0.0, "does_amb_change": True, "default_busyness": 0.3 }
    exp3 = {"name": "travel_time_vs_rl_p_amb_changes_0p5_busyness", "xvarname": "rl_p",
            "minx": 0.0, "maxx": 1.1, "stepx": 0.1, "default_rl_p": 0.0, "does_amb_change": True , "default_busyness": 0.5}
    exp3 = {"name": "travel_time_vs_rl_p_amb_changes_0p7_busyness", "xvarname": "rl_p",
            "minx": 0.0, "maxx": 1.1, "stepx": 0.1, "default_rl_p": 0.0, "does_amb_change": True , "default_busyness": 0.7}

    exp4 = {"name": "travel_time_vs_rl_p_amb_does_not_change_0p1_busyness", "xvarname": "rl_p", "minx": 0.0,
            "maxx": 1.1, "stepx": 0.1, "default_rl_p": 1.0, "does_amb_change": False, "default_busyness": 0.1}
    exp5 = {"name": "travel_time_vs_rl_p_amb_does_not_change_0p3_busyness", "xvarname": "rl_p",
            "minx": 0.0, "maxx": 1.1, "stepx": 0.1, "default_rl_p": 0.0, "does_amb_change": False,
            "default_busyness": 0.3}
    exp6 = {"name": "travel_time_vs_rl_p_amb_does_not_change_0p5_busyness", "xvarname": "rl_p",
            "minx": 0.0, "maxx": 1.1, "stepx": 0.1, "default_rl_p": 0.0, "does_amb_change": False,
            "default_busyness": 0.5}
    exp7 = {"name": "travel_time_vs_rl_p_amb_does_not_change_0p7_busyness", "xvarname": "rl_p",
            "minx": 0.0, "maxx": 1.1, "stepx": 0.1, "default_rl_p": 0.0, "does_amb_change": False,
            "default_busyness": 0.7}


    Experiments_list = [exp1, exp2, exp3, exp4, exp5, exp6, exp7]


    for experiment in Experiments_list:
        # 1- Create an experiment instance
        test_experiment = Experiment(name=experiment["name"], gui_on=False)

        # 2- define an experiment via define_experiment
        ''' Options for xvarname = [rl_p, lane_busyness, start_pos]'''
        xvarname = experiment["xvarname"]
        minx = experiment["minx"]
        maxx = experiment["maxx"]
        stepx = experiment["stepx"]
        paramx = [xvarname, minx, maxx, stepx]

        xvarname, experiment_samples_list = test_experiment.define_experiment(paramx=paramx,
                                                                              does_amb_change=experiment["does_amb_change"], start_pos="middle",
                                                                              default_rl_p=experiment["default_rl_p"],
                                                                              default_busyness=experiment['default_busyness'])

        # 3- run an experiment via run_experiment
        test_experiment.run_experiment(xvarname, experiment_samples_list, num_experiments_per_sample=N, yvarname="amb_travel_time")