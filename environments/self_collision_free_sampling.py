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

from moveit_commander import RobotCommander
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from ..sampler.self_collision_free_sampler import allocSelfCollisionFreeStateSampler
from ..state_validity_check.state_validity_checker import MoveitStateValidityChecker
from ..utils.utils import convertStateToRobotState, generate_random_state, get_joint_names, JOINT_LIMITS, JOINT_INDEX
from ..utils.positions import nominal_pos, neutral_position
from time import sleep
from math import fabs

import moveit_commander
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
    rospy.init_node('Baxter_Env')
    # for publishing trajectory;
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=0)

    space = initialize_space()
    ss = og.SimpleSetup(space)

    state_validity_checker = MoveitStateValidityChecker(ss.getSpaceInformation())
    # TODO: Implement a state validity checker
    ss.setStateValidityChecker(state_validity_checker)

    start = nominal_pos(space)
    goal = None
    if goal_config == 'fixed':
        goal = neutral_position(space)
    elif goal_config == 'random':
        goal = generate_random_state(space, 'right')
        attempts = 0
        while not state_validity_checker.isValid(goal) and attempts < ATTEMPT_LIMIT:
            print("Goal is not valid, regenerating a new goal for the environment. %s" % goal)
            goal = generate_random_state(space, 'right')
            attempts += 1

        print("The goal is: %s" % goal)

        if attempts >=  ATTEMPT_LIMIT:
            print("Cannot find a valid goal after %s attempts. Exiting" % attempts)
            exit(0)
    t0, s0 = plan(0, start, goal, ss, space, display_trajectory_publisher)
    t1, s1 = plan(1, start, goal, ss, space, display_trajectory_publisher)
    return (t0, t1, s0, s1)

def plan(samplerIndex, start, goal, ss, space, display_trajectory_publisher):
    # # construct the state space we are planning in
    # rospy.init_node('Baxter_Env')
    # # for publishing trajectory;
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory,
    #                                                queue_size=0)
    # dof = 7
    # space = ob.RealVectorStateSpace(dof)

    # # set the bounds
    # bounds = ob.RealVectorBounds(dof)
    # for key, value in JOINT_LIMITS.items():
    #     i = JOINT_INDEX[key]
    #     bounds.setLow(i, value[0])
    #     bounds.setHigh(i, value[1])
    #     print("Setting bound for the %sth joint: %s, bound: %s" % (i, key, value))
    # space.setBounds(bounds)

    # ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    if samplerIndex == 1:
        # use obstacle-based sampling
        space.setStateSamplerAllocator(ob.StateSamplerAllocator(allocSelfCollisionFreeStateSampler))
    # define a simple setup class

    # ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # Initialize movegroup; 
    # robot = moveit_commander.RobotCommander()
    # r_arm_group = moveit_commander.MoveGroupCommander('right_arm')
    # set the start and goal states;
    ss.setStartAndGoalStates(start, goal)

    # create a planner for the defined space
    planner = og.FMT(si)

    if samplerIndex == 1:
        planner.setVAEFMT(1) # This flag is for turning on sampling with the VAE generator;

    # planner.setNearestK(1) # Disable K nearest neighbor implementation;
    ss.setPlanner(planner)

    start_time = time.time()
    solved = ss.solve(40.0)
    elapsed_time = time.time() - start_time
    if solved:
        print("Found solution after %s seconds:" % elapsed_time)
        # print the path to screen
        path = ss.getSolutionPath()
        print("The solution is: %s" % path)

        display_trajectory = DisplayTrajectory()

        display_trajectory.trajectory_start = convertStateToRobotState(start)
        trajectory = convertPlanToTrajectory(path)
        display_trajectory.trajectory.append(trajectory)
        display_trajectory_publisher.publish(display_trajectory);

        print("Visualizing trajectory...")
        sleep(0.5)
        # Visualization

    else:
        print("No solution found after %s seconds: " % elapsed_time)
    return elapsed_time, float((int(solved)))

if __name__ == '__main__':
    n = 10
    total_t0 = 0
    total_t1 = 0
    total_s0 = 0
    total_s1 = 0
    total = [total_t0, total_t1, total_s0, total_s1]
    for i in range(n):
        result = benchmark('random')
        for i in range(len(total)):
            total[i] += result[i]
    print("The average planning time for %s goals using FMT is: %s" % (n, total[0] / n))
    print("The average planning time for %s goals using VAEFMT is: %s" % (n, total[1] / n))
    print("The average success rate using FMT is: %s"% (total[2] / n))
    print("The average success rate using VAEFMT is: %s" % (total[3] / n))
    # print("\nUsing self_collision_free_sampler")
    # plan(1)