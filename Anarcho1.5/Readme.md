This version (1.5) was copied from version (1.41). Design goal was to facilitate running experiments to measure the results of the independet learner single-agent. <br>


# Main changes:
* Now you run from main_runner.py; Anarcho.py was deleted.
* To run, open main_runner and define the list of experiments you would like to run.

# Current Models:
* SUMO_KRAUSS
* Q_LEARNING_SINGLE_AGENT

# How this works?
* 1- An experiment sweeps over one variable, measuring another variable (one on the x, another on the y). Runs an episode with
    each x  value pair (x); N times, preparing for later averaging over the result over the N runs/episodes.
* 2- An experiment consists of samples, a sample consists of an episode run for the specific (x) value. I.e. the
    result of the observation for an (x) value is calculated by averaging the results for the same (x) value over N samples
    (i.e. samples = runs = group of episodes till termination)
* 3- The function define_experiment generates a list of samples-metadata (stored in dictionaries), for the function run_experiment
    to run them using run_sample(sample_metadata), N times (i.e. N calls for run_sample). (i.e.sample_metadata is stored in sample_dict, they are the same thing).
    #TODO let define_experiment return a **generator** of samples-metadata, instead of a **list** of samples-metadata
* 4- The function run_experiment stores the result for each N iterations, averages the output observation after N sample of a certain sample
    metadata, and saves it to a csv file using the function save_to_csv.

#### Notes:
* Currently, the only obervation possible is the travel_time. Code is made ready to add later observations through the variable
        aggregate_yvarname_sum and following lines in function run_experiment.
   
# How to use?
* 0- Create an experiments list to loop over. (i.e. define experiment metadata)
* 1- Create an experiment instance
* 2- define an experiment via define_experiment
* 3- run an experiment via run_experimen
