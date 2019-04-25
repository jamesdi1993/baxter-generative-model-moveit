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
from time import sleep
from math import fabs
from src.utils import sample_loc

class SelfCollisionFreeStateSampler(ob.StateSampler):
    def __init__(self, si):
        super(SelfCollisionFreeStateSampler, self).__init__(si)
        self.name_ = "self_collision_free_sampler"
        # self.rng_ = ou.RNG()

        # TODO: Load model here;
    def sampleUniform(self, state):
        waypoint = sample_loc(1, "right") # sample a way point;
        # TODO: Make sure that the order of RobotState is correct;
        for i, key in enumerate(waypoint):
            state[i] = waypoint[key]
        return state

# This function is needed, even when we can write a sampler like the one
# above, because we need to check path segments for validity
def isStateValid(state):
    rs = RobotState()
    robot = RobotCommander()

    # TODO: Figure out a better way to make this efficient; 
    robot_state = robot.get_current_state()
    rs.joint_state.name = robot_state.joint_state.name
    rs.joint_state.position = list(robot_state.joint_state.position) # filler for rest of the joint angles not found in waypoint

    for i in range(len(state)):
        rs.joint_state.position[i] = state[i]
    # joint_name_indices = [rs.joint_state.name.index(n) for n in waypoint.keys()]
    # for i, idx in enumerate(joint_name_indices):
    #     rs.joint_state.position[idx] = waypoint.values()[i]

    isStateValid = sv.getStateValidity(rs, group_name=limb_name+'_arm')
    return isStateValid

# return an obstacle-based sampler
def allocOBValidStateSampler(si):
    # we can perform any additional setup / configuration of a sampler here,
    # but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return ob.ObstacleBasedValidStateSampler(si)

# return an instance of my sampler
def allocSelfCollisionFreeStateSampler(si):
    return SelfCollisionFreeStateSampler(si)

def initialize_start(space):
    start = ob.State(space)
    start[0] = 0
    start[1] = -0.55
    start[2] = 0
    start[3] = 0.75
    start[4] = 0
    start[5] = 1.26
    start[6] = 0
    # Neutral position is defined as: [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
    return start

def generate_random_goal(space):
    goal = ob.State(space)
    waypoint = sample_loc(1, "right") # sample a way point;
    for i, key in enumerate(waypoint):
        goal[i] = waypoint[key]
    return goal

def plan(samplerIndex):
    # construct the state space we are planning in
    dof = 7
    space = ob.RealVectorStateSpace(dof)

    # set the bounds
    bounds = ob.RealVectorBounds(dof)
    for i in range(dof):
        bounds.setLow(i, -1)
        bounds.setHigh(i, 1)
    space.setBounds(bounds)

    if samplerIndex == 1:
        # use obstacle-based sampling
        space.setStateSamplerAllocator(ob.StateSamplerAllocator(allocSelfCollisionFreeStateSampler))
    # define a simple setup class
    ss = og.SimpleSetup(space)

    # TODO: Implement a state validity checker
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    start = initialize_start(space)
    goal = generate_random_goal(space)

    # set the start and goal states;
    ss.setStartAndGoalStates(start, goal)

    # set sampler (optional; the default is uniform sampling)
    si = ss.getSpaceInformation()

    # create a planner for the defined space
    planner = og.FMT(si)
    ss.setPlanner(planner)

    # attempt to solve the problem within ten seconds of planning time
    solved = ss.solve(10.0)
    if solved:
        print("Found solution:")
        # print the path to screen
        print(ss.getSolutionPath())
    else:
        print("No solution found")

if __name__ == '__main__':
    print("Using default uniform sampler:")
    plan(0)
    print("\nUsing self_collision_free_sampler")
    plan(1)