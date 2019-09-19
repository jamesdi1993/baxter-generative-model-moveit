#!/usr/bin/env python
# Author: Yubai Di
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

from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from baxter_interfaces.env.environments import empty_environment, one_box_environment, one_pillar_environment
from baxter_interfaces.sampler.collision_free_sampler import allocSelfCollisionFreeStateSampler
from baxter_interfaces.state_validity_check.state_validity_checker import TimedMoveitOMPLStateValidityChecker
from baxter_interfaces.utils.utils import convertStateToRobotState, generate_random_state, get_joint_names, JOINT_LIMITS, JOINT_INDEX
from baxter_interfaces.utils.positions import nominal_pos, neutral_position
from time import sleep

import argparse
import numpy as np
import rospy
import time

# TODO: Use RobotState loaded from URDF to construct this information;
ATTEMPT_LIMIT = 10

def isStateValid(state):
    # Dummy function that always returns true for computing validity of a state; 
    sleep(.001)
    return True

def convertPlanToTrajectory(path):
    """
    Convert a path in OMPL to a trajectory in Moveit that could be used for display;
    :param plan: A path in OMPL;
    :return: a RobotTrajectory message in Moveit;
    """
    trajectory = RobotTrajectory()
    states = path.getStates()
    points = []
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = get_joint_names('right')
    for state in states:
        point = JointTrajectoryPoint()
        point.positions = convertStateToRobotState(state).joint_state.position
        points.append(point)
    joint_trajectory.points = points
    trajectory.joint_trajectory = joint_trajectory
    return trajectory

def initialize_space():
    dof = 7
    space = ob.RealVectorStateSpace(dof)

    # set the bounds
    bounds = ob.RealVectorBounds(dof)
    for key, value in JOINT_LIMITS.items():
        i = JOINT_INDEX[key]
        bounds.setLow(i, value[0])
        bounds.setHigh(i, value[1])
        print("Setting bound for the %sth joint: %s, bound: %s" % (i, key, value))
    space.setBounds(bounds)
    return space

def benchmark(goal_config='fixed'):
    # for publishing trajectory;
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=0)

    space = initialize_space()
    ss = og.SimpleSetup(space)

    state_validity_checker = TimedMoveitOMPLStateValidityChecker(ss.getSpaceInformation())
    ss.setStateValidityChecker(state_validity_checker)

    states = [None, None] # start and goal states
    if goal_config == 'fixed':
        states[0] = nominal_pos(space) # fixed position
        states[1] = neutral_position(space)
    elif goal_config == 'random':
        validity = np.zeros(2)
        for i in range(len(states)):
            states[i] = generate_random_state(space, 'right')
            validity[i] = state_validity_checker.isValid(states[i])

        indx = np.where(validity == 0)[0]

        attempts = 0

        if indx.size != 0:
            for ind in np.nditer(indx):
                while not state_validity_checker.isValid(states[int(ind)]) and attempts < ATTEMPT_LIMIT:
                    print("State is not valid, regenerating a new state %s" % states[int(ind)])
                    states[int(ind)] = generate_random_state(space, 'right')
                attempts += 1

        if attempts >=  ATTEMPT_LIMIT:
            print("Cannot find a valid goal after %s attempts. Exiting" % attempts)
            exit(0)

    state_validity_checker.resetTimer()
    t0, s0 = plan(0, states[0], states[1], ss, space, display_trajectory_publisher)
    c0, counts0 = state_validity_checker.getTime()
    state_validity_checker.resetTimer()
    t1, s1 = plan(1, states[0], states[1], ss, space, display_trajectory_publisher)
    c1, counts1= state_validity_checker.getTime()
    return (t0, t1, s0, s1, c0, c1, counts0, counts1)

def plan(samplerIndex, start, goal, ss, space, display_trajectory_publisher):
    si = ss.getSpaceInformation()

    if samplerIndex == 1:
        # use obstacle-based sampling
        space.setStateSamplerAllocator(ob.StateSamplerAllocator(allocSelfCollisionFreeStateSampler))

    ss.setStartAndGoalStates(start, goal)

    # create a planner for the defined space
    planner = og.FMT(si) # change this to FMT;
    if samplerIndex == 1:
        planner.setVAEFMT(1) # This flag is for turning on sampling with the VAE generator;

    # set parameter;
    planner.setExtendedFMT(False) # do not extend if the planner does not terminate;
    planner.setNumSamples(100)
    planner.setNearestK(False);
    planner.setCacheCC(True);
    planner.setHeuristics(True);


    # planner.setNearestK(1) # Disable K nearest neighbor implementation;
    ss.setPlanner(planner)

    start_time = time.time()
    solved = ss.solve(40.0)
    elapsed_time = time.time() - start_time
    if solved:
        print("Found solution after %s seconds:" % elapsed_time)
        # print the path to screen
        path = ss.getSolutionPath()
        # ("The solution is: %s" % path)

        # Visualization
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = convertStateToRobotState(start)
        trajectory = convertPlanToTrajectory(path)
        display_trajectory.trajectory.append(trajectory)
        display_trajectory_publisher.publish(display_trajectory)
        print("Visualizing trajectory...")
        sleep(0.5)

    else:
        print("No solution found after %s seconds: " % elapsed_time)
    return elapsed_time, float((int(solved)))

"""
Sample Usage:
python -m baxter_interfaces.experiments.vae_motion_planning --env one_pillar_environment --n 10 --mode random
"""
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--env')
    parser.add_argument('--n', type=int, default=10)
    parser.add_argument('--mode', default='fixed')
    args, _ = parser.parse_known_args()
    env = args.env
    n = args.n
    mode = args.mode

    rospy.init_node('VAE_Motion_Planning_Main')
    if env == "empty_environment":
        empty_environment()
    elif env == "one_box_environment":
        one_box_environment()
    elif env == "one_pillar_environment":
        one_pillar_environment()
    total = [0] * 8
    for i in range(n):
        result = benchmark(mode)
        for i in range(len(total)):
            total[i] += result[i]

    # Print statistics
    print("Average planning time for %s goals using FMT is: %s" % (n, total[0] / n))
    print("Average planning time for %s goals using VAEFMT is: %s" % (n, total[1] / n))
    print("Average success rate using FMT is: %s"% (total[2] / n))
    print("Average success rate using VAEFMT is: %s" % (total[3] / n))
    print("Average collision checking time for %s goals using FMT is: %s" % (n, total[4] / n))
    print("Average collision checking time for %s goals using VAEFMT is: %s" % (n, total[5] / n))
    print("Number of collision checks using FMT is: %s" % (total[6] / n))
    print("Number of collision checks using VAEFMT is: %s" % (total[7] / n))


    with open(mode + '_' + str(int(time.time())) + '.log', mode='w') as output_file:
        output_file.write("Average collision checking time for %s goals using FMT is:%s\n" % (n, total[4] / n))
        output_file.write("Average collision checking time for %s goals using VAEFMT is:%s\n" % (n, total[5] / n))
        output_file.write("Number of collision checks using FMT is:%s\n" % (float(total[6]) / n))
        output_file.write("Number of collision checks using VAEFMT is:%s" % (float(total[7]) / n))
