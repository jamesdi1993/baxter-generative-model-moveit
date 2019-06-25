try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og

#!/usr/bin/python
from src.env.environments import empty_environment, one_box_environment
from src.state_validity_check.state_validity_checker import MoveitStateValidityChecker
from src.env.space import initialize_space
from src.utils.utils import get_joint_names, sample_loc, construct_robot_state
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

import copy as cp
import csv
import math
import os
import rospy
import sys
import time

DIRECTORY = "./data/sampled_data/self_and_environment_collision"
FILE_EXTENSION = '.csv'
FILE_DELIMETER = '_'
SELF_COLLISION_KEY = 'self_collision_free'
COLLISION_KEY = 'collision_free'
WORLD_OBJECT = 1
ROBOT_LINK = 0
BATCH_SIZE  = 1000

def get_file_name(limb_name, num_joints, num_points, time):
    return limb_name + FILE_DELIMETER + str(num_joints) + FILE_DELIMETER + str(num_points) + FILE_DELIMETER + str(time) + FILE_EXTENSION

def generate_configs(num_samples, state_validity_checker, limb_name):
    """
    Generate configurations, that would be later written to the csv.
    :param num_samples: The number of configs to generate each time.
    :param stateValidityChecker: The collisionChecker used for determining state validity.
    :param space: the space for constructing the waypoint
    :return: A set of configurations.
    """
    # Sample locations
    samples = sample_loc(num_samples=num_samples, name=limb_name)

    # Transform locations into arrays
    joint_value_array = zip(*samples.values())  # Convert to list, and then transform.
    joint_array = map(lambda joint: dict(zip(samples.keys(), joint)), joint_value_array)  # Add key to each config

    configs = []
    for index in range(len(joint_array)):
        sample = joint_array[index]
        valid = check_config(sample, state_validity_checker, limb_name + "_arm")
        sample[SELF_COLLISION_KEY] = int(valid)
        configs.append(sample)
    return configs

def check_config(config, state_validity_checker, limb_name):
    """
    Check the validity of a configuration
    :param : The configurations to check against.
    :param state_validity_checker: The validity checker
    :param space: The space to check for
    :param limb_name: The name of the limb.
    :return: The validty of the configurations.
    """
    # Construct a robotstate object from a dictionary
    rs = RobotState()
    js = JointState()
    js.name = get_joint_names('right')
    js.position = [config[joint] for joint in js.name]
    rs.joint_state = js
    result = state_validity_checker.getStateValidity(rs, group_name=limb_name + '_arm')
    return result.valid

def generate_self_collision_dataset(start, num_joints, num_points):
    """
    Generate the base self-collision dataset labels.
    :return: The file that corresponds to the dataset.
    """
    total_number_points = 0

    # import environment
    empty_environment()

    # initialize validity check
    space = initialize_space()
    ss = og.SimpleSetup(space)
    state_validity_checker = MoveitStateValidityChecker(ss.getSpaceInformation())

    # header for the dataset
    limb_name = 'right'
    header_written = False
    header = get_joint_names(limb_name) + [SELF_COLLISION_KEY]

    if not os.path.exists(DIRECTORY):
        os.makedirs(DIRECTORY)

    file_name = os.path.join(DIRECTORY, get_file_name(limb_name, num_joints, num_points, start))

    # statistics for tracking generation
    batch_time = []

    for i in range(0, int(math.ceil(float(num_points) / BATCH_SIZE))):
        print("Elasped time: %s" % (time.time() - start))
        print("Start generating data for batch: %s" % (i + 1))
        batch_start = i * BATCH_SIZE
        batch_end = batch_start + BATCH_SIZE
        size = BATCH_SIZE

        if batch_end >= num_points:
            # We've come to the last batch
            size = num_points - batch_start

        batch_time_start = time.time()

        samples = generate_configs(size, state_validity_checker, limb_name)

        with open(file_name, mode='a') as csv_file:
            # Quote header such that it is easier for reader to parse data
            writer = csv.DictWriter(csv_file, fieldnames=header, quoting=csv.QUOTE_NONNUMERIC)
            if not header_written:
                writer.writeheader()
                header_written = True
            writer.writerows(samples)
        total_number_points += size

        # Save time to generate batch for later statistics
        batch_time_end = time.time()
        batch_time.append(batch_time_end - batch_time_start)

    # output statistics;
    end = time.time()
    total_time = end - start
    print("Finished generating self-collision dataset. " +
          "Total time elapsed: %s; Total number of points: %s" % (total_time, total_number_points))
    print("Average time to produce a batch of size %s: %s" % (
    BATCH_SIZE, reduce(lambda x, y: x + y, batch_time) / len(batch_time)))
    return file_name

def get_label_name(env_name):
    return  env_name + '_' + COLLISION_KEY

def augment_collision_dataset(file_name, env_name):
    """
    Augment the dataset with additional labels, based on the environment.
    :param file_name: The file name
    :param env_name: The environment for generating the additional label.
    :return: The file that corresponds to the new dataset.
    """
    print("Generating environment collision label for: %s" % (file_name))
    start = time.time()
    if env_name == 'one_box_environment':
        one_box_environment() # initialize env

    # initialize validity check
    space = initialize_space()
    ss = og.SimpleSetup(space)
    state_validity_checker = MoveitStateValidityChecker(ss.getSpaceInformation())

    file_prefix = os.path.abspath(file_name).split('.')[0]
    output_file_name = file_prefix + "_augmented" + ".csv"
    label = get_label_name(env_name)

    with open(file_name, mode='rb') as input_file:
        csv_reader = csv.DictReader(input_file, quoting=csv.QUOTE_NONNUMERIC)
        lines_read = 0
        headers = cp.deepcopy(csv_reader.fieldnames)
        headers.append(label)

        print("Writing to output file: %s" % output_file_name)

        # Write headers with the fields in the input
        with open(output_file_name, mode='w') as output_file:
            # Quote header such that it is easier for reader to parse data
            writer = csv.DictWriter(output_file, fieldnames=headers, quoting=csv.QUOTE_NONNUMERIC)
            writer.writeheader()
            points = []
            batch = 0

            for waypoint in csv_reader:
                if waypoint[SELF_COLLISION_KEY] == 0:
                    waypoint[label] = 0
                else:
                    env_collision_free = check_config(waypoint, state_validity_checker, 'right')
                    waypoint[label] = int(env_collision_free)
                points.append(waypoint)
                lines_read += 1
                if lines_read % BATCH_SIZE == 0:
                    writer.writerows(points)
                    points = []
                    batch += 1
                    print("Finished generating data for batch: %s" % (batch))
            writer.writerows(points)
    print("Finished generating augmented dataset. Time Elasped: %s" % (time.time() - start))
    return output_file

if __name__ == '__main__':
    rospy.init_node('BaxterEnvCollisionDatasetGeneration')
    env = "one_box_environment"

    # Configurations for the dataset generation
    num_joints = int(sys.argv[1])
    num_points = int(sys.argv[2])

    start = int(time.time())
    file = generate_self_collision_dataset(start, num_joints, num_points)
    dataset = augment_collision_dataset(file_name=file, env_name=env) # generate the new dataset;
    print("Finished generating the entire dataset for environment: %s" % env)
    print("Time elapsed: %s" % (time.time() - start))