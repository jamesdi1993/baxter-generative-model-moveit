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

