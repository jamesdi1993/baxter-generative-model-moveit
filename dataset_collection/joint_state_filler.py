#!/usr/bin/python
from moveit_msgs.msg import DisplayRobotState
from src.demo import StateValidity
from joint_state_generator import COLLISION_KEY, FILE_DELIMETER, initialize_environment, get_current_state, fill_waypoint
from os.path import isfile, join

import argparse
import baxter_interface
import copy as cp
import csv
import os
import rospy
import time

LIMIT = 10000000
COLLISION_KEY = 'collisionFree'
BATCH_SIZE = 1000
INPUT_BASE_PATH = "../data/generated_data/input/"
OUTPUT_BASE_PATH = "../data/generated_data/validated/"
PATH_ARGS = ["num_joints", "beta"]

# TODO: Refactor these methods with the Baxter_vae_self_collision pkg;
def find_data_file(path, num_joints):
    # Find the file to read data from;
    file_name = ''
    files = [f for f in os.listdir(path) if isfile(join(path, f))]
    for f in files:
        f_name = f.replace('.csv', '')
        joints = int(f_name.split('_')[1])
        num_samples = int(f_name.split('_')[2])
        if joints == num_joints and num_samples < LIMIT:
            file_name = join(path, f)
            break
        elif num_samples > LIMIT:
            raise IOError(
                "Number of samples is greater than limit. Num of samples: " + str(num_samples))
    print("Loading data from file: %s" % (file_name,))
    return file_name


def get_path(base_path, path_args):
    # Get the path for the output dir
    relative_path = base_path
    for (arg, value) in path_args:
        relative_path = relative_path + str(value) + "_" + arg + "/"
    dirname = os.path.dirname(__file__)
    # return relative_path
    return os.path.join(dirname, relative_path)


def parse_args(args):
    """
    Parse out the args according to the order in PATH_ARGS, if they exist
    :param args: A namespace object.
    :return: A dictionary containing the file args.
    """
    path_args = []
    for arg in PATH_ARGS:
        if arg in args.__dict__.keys():
            path_args.append((arg, args.__dict__[arg]))
    return path_args


def fillCollisionFlag(sv, limb_name, waypoint):
    # Deep copy the waypoint first, then pop the key
    point = cp.deepcopy(waypoint)
    current_state = get_current_state()
    rs = fill_waypoint(point, current_state)
    try: 
        collisionflag = sv.getStateValidity(rs, group_name=limb_name + '_arm').valid
        point[COLLISION_KEY] = int(collisionflag)
    except:
        print("An IOError was encourtered. Waypoint: " + str(waypoint))
        point[COLLISION_KEY] = 0
    # print("Collision flag: %s" % bool(collisionflag))
    return point

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--num-joints', type=int, default=7)
    parser.add_argument('--beta', type=float, default=1.0)
    args, _ = parser.parse_known_args()
    num_joints = args.num_joints

    path_args = parse_args(args)
    input_path = get_path(INPUT_BASE_PATH, path_args)
    print("The input directory is: %s" % input_path)

    input_name = find_data_file(input_path, num_joints)
    file_name = input_name.split('/')[-1] # file_name
    print("Reading from input file: %s" % input_name)

    output_path = get_path(OUTPUT_BASE_PATH, path_args)

    if not os.path.exists(output_path):
        os.makedirs(output_path)
    output_name = output_path + file_name.replace('.csv', '') + '_filled' + '.csv'
    print("Writing to output file: %s" % output_name)

    scene, robot = initialize_environment()
    rospy.sleep(2)
    robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=0)
    sv = StateValidity()

    limb_name = file_name.split(FILE_DELIMETER)[0]
    limb = baxter_interface.Limb(limb_name)
    sv = StateValidity()
    print("Reading from input file: %s" % input_name)

    start = time.time()
    with open(input_name, mode='rb') as input_file:
        csv_reader = csv.DictReader(input_file, quoting=csv.QUOTE_NONNUMERIC)
        lines_read = 0
        headers = cp.deepcopy(csv_reader.fieldnames)
        headers.append(COLLISION_KEY)

        print("Writing to output file: %s" % output_name)

        # Write headers with the fields in the input
        with open(output_name, mode='w') as output_file:
            # Quote header such that it is easier for reader to parse data                                                                                                                                                                                   
            writer = csv.DictWriter(output_file, fieldnames=headers, quoting=csv.QUOTE_NONNUMERIC)
            writer.writeheader()
            points = []
            batch = 1

            for waypoint in csv_reader:
                # print("Filling waypoint %d" % (lines_read + 1))
                # print(waypoint)
                point = fillCollisionFlag(sv, limb_name, waypoint)
                points.append(point)
                lines_read += 1
                if lines_read % BATCH_SIZE == 0:
                    writer.writerows(points)
                    points = []
                    print("Finish filling data for batch: %s" % (batch))
                    batch += 1
            writer.writerows(points)
        elapsed_time = time.time() - start
        print("Finished processing. The number of lines read is: %s. Elapsed_time: %s" % (lines_read, elapsed_time))

            # visualize_waypoint(limb, waypoint, False, robot_state_collision_pub)
            # Sleep 5 seconds for wavepoint to finish.
            # time.sleep(5)
