import csv
import argparse
from hyperopt import fmin, tpe, hp
from hyperopt.mongoexp import MongoTrials
import hyperopt.pyll
from hyperopt.pyll import scope

# paths
path_results =      "/data/KalmanTuningLogs/"


## objective function
# for each trial this function is called and starts the trial runner script
def objective(params):
    # if we run parallel workers then only the objective function is called
    # thats why we can not use anything defined outside of this function :(
    import os
    import csv
    import yaml
    import math
    import time
    import random
    import subprocess
    from hyperopt import STATUS_OK, STATUS_FAIL
    from collections import deque


    # path definitions
    path_home = "/home/fabian"
    path_catkin_ws = path_home + "/catkin_ws"
    path_ros_package = path_catkin_ws + "/src/drive_ros_config/modules/drive_ros_localize_wheel_odometry"
    path_config_file = path_ros_package + "/config/ftm_rc_car_1.yaml"
    path_trial_runner = path_ros_package + "/scripts/hyperopt/runTrial.sh"
    path_bag_folder = "/data/Backup/Bags/FabianMA/odom_cali"
    path_launch_file = path_ros_package + "/launch/localize_wheel_odometry.launch"
    path_results = "/data/KalmanTuningLogs/"

    # TODO: make them bag specific (like: 1m error of 100m length)
    # normalize results by this values
    x_norm = 0.1 # meters
    y_norm = 0.1 # meters
    theta_norm = 0.05 # radiant

    # get trial number
    trial = random.getrandbits(128)

    # check if trial number already exists
    while os.path.exists(path_results + str(trial)):
        trial = random.getrandbits(128)

    # create directory for this trial
    os.makedirs(path_results + str(trial))
    print("  Start trial with number: "+str(trial))

    # config file path
    yaml_config = path_results + str(trial) + "/config.yaml"

    # load config
    config_file_in = path_config_file
    config_in = open(config_file_in, 'r')
    config = yaml.load(config_in)

    # update config
    config['b_actual'] = params['b']
    config['err_d'] = params['d']

    # save config
    config_out = open(yaml_config, 'w')
    yaml.dump(config, config_out)

    # start trial runner
    p = subprocess.run([path_trial_runner,
                        "--trial", str(trial),
                        "--logdir", path_results + str(trial),
                        "--bag", path_bag_folder,
                        "--config", yaml_config,
                        "--launch", path_launch_file,
                        "--catkin_ws", path_catkin_ws ], shell=False)

    # evaluate return code of trial runner
    if 0 == p.returncode:
        status = STATUS_OK
        print("  Trial successfull!")
    else:
        status = STATUS_FAIL
        print("  Trial failed!")


    # calculate loss
    # loop over all csv files
    loss = list()
    for filename in os.listdir(path_results + str(trial)):
        if filename.endswith(".csv"):
            # open csv file
            print(filename)
            with open(path_results + str(trial) + "/" + filename, 'r') as f:
                try:
                    lastrow = deque(csv.reader(f), 1)[0]
                except IndexError:  # empty file
                    lastrow = None

                x = float(lastrow[1])            # x position at the end
                y = float(lastrow[2])            # y position at the end
                theta = float(lastrow[6])        # theta at the end

                loss.append(math.sqrt( x/x_norm * x/x_norm
                                     + y/y_norm * y/y_norm
                                     + theta/theta_norm * theta/theta_norm ))
   
    avg_loss = sum(loss)/len(loss)             
    print("  Loss is: " + str(avg_loss))
    # return trial results
    return {
        'loss': avg_loss,
        'status': status,
        'eval_time': time.time(),
        'log_path': path_results + str(trial),
        'pose': { 
                 'x_end': x,
                 'y_end': y,
                 'theta_end': theta
                },
        'params': {
                   'd': params['d'],
                   'b': params['b']
                  },
        'norms': {
                    'x': x_norm,
                    'y': y_norm,
                    'theta': theta_norm
                 }
    }


## main function
if __name__ == "__main__":

    # parse arguments
    print("Parse arguments")
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Main Param Tuning script. Creates all trials and connects to the MongoDB where results are stored.')
    parser.add_argument('--mongohost', help='database hostname or ip', required=True)
    parser.add_argument('--mongoport', help='database port', type=int, required=True)
    parser.add_argument('--experiment', help='name of experiment', required=True)
    parser.add_argument('--max_evals', help='how many evaluations should be tried', type=int, required=True)
    args = parser.parse_args()

    # create search space
    print("Create search space")
    space = {
        'b': hp.uniform('b', 0.19, 0.23),
        'd': hp.uniform('d', 0.99, 1.01)
    }

    # some settings
    algo = tpe.suggest
    max_evals = args.max_evals
    mongo_host = args.mongohost
    mongo_port = args.mongoport
    mongo_db = args.experiment

    trials = MongoTrials("mongo://" + mongo_host + ":" + str(mongo_port) + "/" + mongo_db + "/jobs",
                         exp_key="param")


    # find the hyperparameters
    print("Start fmin ...")
    best = fmin(objective, space, algo, max_evals, trials)
    print("Finished fmin :=)")
