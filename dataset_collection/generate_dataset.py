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
from baxter_interfaces.env.environments import empty_environment
from baxter_interfaces.state_validity_check.state_validity_checker import MoveitOMPLStateValidityChecker
from baxter_interfaces.env.space import initialize_space
from baxter_interfaces.dataset_collection.label_generators import SelfCollisionLabelGenerator, EnvironmentCollisionLabelGenerator, EndEffectorPositionGenerator, get_collision_label_name
from baxter_interfaces.dataset_collection.common import check_config, augment_dataset
from baxter_interfaces.env.self_collision_free_sampling import plan
from baxter_interfaces.utils.utils import get_joint_names, sample_loc, construct_robot_state
from baxter_interfaces.visualization.workspace_plot_rviz import WaypointPublisher
from baxter_interfaces.utils.positions import nominal_pos

from moveit_msgs.msg import RobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

import argparse
import copy as cp
import csv
import math
import os
import rospy
import sys
import time

DIRECTORY = "./data/%s/input"
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
    state_validity_checker = MoveitOMPLStateValidityChecker(ss.getSpaceInformation())

    # header for the dataset
    limb_name = 'right'
    header_written = False
    header = get_joint_names(limb_name) + [SELF_COLLISION_KEY]

    directory = DIRECTORY % "empty_environment"
    if not os.path.exists(directory):
        os.makedirs(directory)

    file_name = os.path.join(directory, get_file_name(limb_name, num_joints, num_points, start))

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

def main(args):
    rospy.init_node('BaxterEnvCollisionDatasetGeneration')
    file = args.file

    start = int(time.time())

    if file == None:
        num_joints = args.num_joints
        num_points = args.num_points
        output = generate_self_collision_dataset(start, num_joints, num_points)
        print("Finished generating dataset. Output file: %s" % (output))
    else:
        label = args.label
        env = args.env

        label_generator = None
        if label == "selfCollision":
            label_generator = SelfCollisionLabelGenerator(env_name=env)
        elif label == "envCollision":
            label_generator = EnvironmentCollisionLabelGenerator(env_name=env)
        elif label == "endPosition":
            label_generator = EndEffectorPositionGenerator(env_name=env)
        else:
            raise RuntimeError("Unknown label to augment for the dataset: %s" % label)
        file_prefix = os.path.abspath(file).split('.')[0]
        output_file = file_prefix + "_%s.csv" % label_generator.label

        augment_dataset(input_file=file, output_file=output_file, env_name=env, label_generator=label_generator)  # generate the new dataset;
        print("Finished augmenting dataset for environment: %s; Output file: %s" % (env, output_file))
    print("Time elapsed: %s" % (time.time() - start))

def test_augment_end_effector_pos():
    rospy.init_node('End_effector_pos_test')
    # for publishing trajectory;
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory,
                                                   queue_size=0)
    rospy.sleep(0.5)

    space = initialize_space()
    ss = og.SimpleSetup(space)
    state_validity_checker = MoveitOMPLStateValidityChecker(ss.getSpaceInformation())
    ss.setStateValidityChecker(state_validity_checker)

    start = nominal_pos(space)

    file = "/home/nikhildas/ros_ws/baxter_interfaces/baxter_moveit_config/data/sampled_data/self_and_environment_collision/right_7_test.csv"
    publisher = WaypointPublisher()

    label = get_collision_label_name("one_box_environment")
    with open(file, mode='rb') as input_file:
        csv_reader = csv.DictReader(input_file,  quoting=csv.QUOTE_NONNUMERIC)
        for waypoint in csv_reader:
            if waypoint[label] == 1:
                point = {}
                for key in get_joint_names('right'):
                    point[key] = waypoint[key]
                goal = construct_robot_state(space, point)
                plan(0, start, goal, ss, space, display_trajectory_publisher)
                position = Point()
                position.x = waypoint['x']
                position.y = waypoint['y']
                position.z = waypoint['z']
                # print("The position of the data point is: %s" % position)
                publisher.publish_waypoints([position], scale=[0.05, 0.05, 0.05])
                time.sleep(5.0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    # dataset configurations
    parser.add_argument('--file') # if file is presented, then augment label;
    parser.add_argument('--env', choices=['empty_environment','one_box_environment'])
    parser.add_argument('--label', choices=['selfCollision', 'envCollision', 'endPosition'])
    parser.add_argument('--num-joints', type=int, default=7)
    parser.add_argument('--num-points', type=int, default=10000)
    args, _ = parser.parse_known_args()
    main(args)
    # test_augment_end_effector_pos()