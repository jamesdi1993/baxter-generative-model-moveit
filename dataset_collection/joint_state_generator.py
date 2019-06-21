#!/usr/bin/python
from moveit_commander import PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, RobotState
from src.demo import StateValidity, sample_loc

import baxter_interface
import copy as cp
import csv
import math
import rospy
import sys
import time

FILE_EXTENSION = '.csv'
FILE_DELIMETER = '_'
COLLISION_KEY = 'collisionFree'
BATCH_SIZE  = 1000

def initialize_environment():
    # Initialize environment
    rospy.init_node('Baxter_Env')
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    scene._scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=0)
    rospy.sleep(2)
    return scene, robot

def get_current_state():
    # Prepare a new state object for validity check
    rs = RobotState()
    robot = RobotCommander()
    robot_state = robot.get_current_state()
    rs.joint_state.name = robot_state.joint_state.name
    rs.joint_state.position = list(robot_state.joint_state.position) # filler for rest of the joint angles not found in waypoint
    return rs

def fill_waypoint(point, state):
    # Fill the joints into state
    rs = cp.copy(state)
    joint_name_indices = [rs.joint_state.name.index(n) for n in point.keys()]
    for i, idx in enumerate(joint_name_indices):
        rs.joint_state.position[idx] = point.values()[i]
    return rs

def generate_configs(state_validity, waypoints):
    # current_state = get_current_state()
    # states = map(lambda point: fill_waypoint(point, current_state), waypoints)
    # configs = []
    # for index in range(0, len(states)):
    #     rs = states[index]
    #     point = waypoints[index]
    #     collision_flag = state_validity.getStateValidity(rs, group_name=limb_name+'_arm')
    #     point[COLLISION_KEY] = int(collision_flag)
    #     configs.append(point)
    configs = []
    for index in range(0, len(waypoints)):
        current_state = get_current_state()
        point = waypoints[index]
        rs = fill_waypoint(point, current_state)
        collision_flag = state_validity.getStateValidity(rs, group_name=limb_name+'_arm').valid
        point[COLLISION_KEY] = int(collision_flag)
        configs.append(point)
    return configs

def get_file_name(limb_name, num_joints, num_points, time):
    return limb_name + FILE_DELIMETER + str(num_joints) + FILE_DELIMETER + str(num_points) + FILE_DELIMETER + str(time) + FILE_EXTENSION

if __name__ == '__main__':
    total_number_points = 0
    start = time.time()

    # Configurations for the dataset generation
    num_joints = int(sys.argv[1])
    num_points = int(sys.argv[2])

    scene, robot = initialize_environment()

    """ Preliminaries, initialize limb, set up publisher for collision checking """
    limb_name = 'right'
    limb = baxter_interface.Limb(limb_name)
    # robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=0)
    sv = StateValidity()

    file_name = get_file_name(limb_name, num_joints, num_points, start)
    header_written = False
    batch_time = []

    for i in range(0, int(math.ceil(float(num_points)/BATCH_SIZE))):
        print("Elasped time: %s" % (time.time() - start))
        print("Start generating data for batch: %s" % (i + 1))
        batch_start = i * BATCH_SIZE
        batch_end = batch_start + BATCH_SIZE
        size = BATCH_SIZE

        if batch_end >= num_points:
            # We've come to the last batch
            size = num_points - batch_start

        # Sample locations
        samples = sample_loc(num_samples=size, num_joints=num_joints, name=limb_name)

        # Transform locations into arrays
        joint_value_array = zip(*samples.values()) # Convert to list, and then transform.
        joint_array = map(lambda joint: dict(zip(samples.keys(), joint)), joint_value_array) # Add key to each config

        batch_time_start = time.time()

        points = generate_configs(sv, joint_array)

        # Save time to generate batch for later statistics
        batch_time_end = time.time()
        batch_time.append(batch_time_end - batch_time_start)

        # Write to csv file
        names = samples.keys() + [COLLISION_KEY]

        with open(file_name, mode='a') as csv_file:
            # Quote header such that it is easier for reader to parse data                                                                                                                                                                                   
            writer = csv.DictWriter(csv_file, fieldnames=names, quoting=csv.QUOTE_NONNUMERIC)
            if not header_written:
                writer.writeheader()
                header_written = True
            writer.writerows(points)
        total_number_points += size

    end = time.time()
    total_time = end - start
    print("Total time elapsed: %s; Total number of points: %s" % (total_time, total_number_points))
    print("Average time to produce a batch of size %s: %s" % (BATCH_SIZE, reduce(lambda x,y: x + y, batch_time)/len(batch_time)))
