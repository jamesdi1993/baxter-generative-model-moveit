#!/usr/bin/python
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_commander import RobotCommander
from moveit_msgs.msg import RobotState, DisplayRobotState
from src.environments.set_environment import main as set_environment

import baxter_interface
import numpy as np
import rospy

DEFAULT_SV_SERVICE = "/check_state_validity"

JOINT_NAMES = ['s0','s1','e0','e1','w0','w1','w2']
JOINT_LIMITS = {
    's0': (-1.7016, 1.7016),
    's1': (-2.147, 1.047),
    'e0': (-3.0541, 3.0541),
    'e1': (-0.05, 2.618),
    'w0': (-3.059, 3.059),
    'w1': (-1.5707, 2.094),
    'w2': (-3.059, 3.059)
}

class StateValidity():

    # Initialize a StateValidity class
    def __init__(self):
        rospy.loginfo("Initializing stateValidity class")
        self.sv_srv = rospy.ServiceProxy(DEFAULT_SV_SERVICE, GetStateValidity)
        rospy.loginfo("Connecting to State Validity service")
        try:
            self.sv_srv.wait_for_service()
            rospy.wait_for_service("check_state_validity")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

        if rospy.has_param('/play_motion/approach_planner/planning_groups'):
            list_planning_groups = rospy.get_param('/play_motion/approach_planner/planning_groups')
        else:
            rospy.logwarn("Param '/play_motion/approach_planner/planning_groups' not set. We can't guess controllers")
        rospy.loginfo("Ready for making Validity calls")

    def close_SV(self):
        self.sv_srv.close()

    def getStateValidity(self, robot_state, group_name='both_arms_torso', constraints=None):
        """Given a RobotState and a group name and an optional Constraints
        return the validity of the State"""
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result.valid

#TODO: Refactor this function using the constants
def sample_loc(num_samples, name):
    joint = {}
    # ref hardware specs for baxter joint limits
    joint[name+'_s0'] = float(np.random.uniform(-1.7016, 1.7016, num_samples)) # s0
    joint[name+'_s1'] = float(np.random.uniform(-2.147, 1.047, num_samples))  # s1
    joint[name+'_e0'] = float(np.random.uniform(-3.0541, 3.0541, num_samples)) # e0
    joint[name+'_e1'] = float(np.random.uniform(-0.05, 2.618, num_samples)) # e1
    joint[name+'_w0'] = float(np.random.uniform(-3.059, 3.059, num_samples)) # w0
    joint[name+'_w1'] = float(np.random.uniform(-1.5707, 2.094, num_samples)) # w1
    joint[name+'_w2'] = float(np.random.uniform(-3.059, 3.059, num_samples)) # w2
    return joint

def sample_loc(num_samples, num_joints, name):
    joint = {}

    # Initialize the joint config
    for joint_name in JOINT_NAMES:
        joint[name + '_' + joint_name] = np.zeros(num_samples)

    # Sample values from the configs provided
    for i in range(0, num_joints):
        joint_name = JOINT_NAMES[i]
        limit = JOINT_LIMITS[joint_name]
        joint[name + '_' + joint_name] = np.random.uniform(limit[0], limit[1], num_samples)
    return joint

if __name__ == '__main__':

    set_environment() # initializes rospy node, creates a test environment with collision objects

    """ Preliminaries, initialize limb, set up publisher for collision checking """
    limb_name = 'right'
    limb = baxter_interface.Limb(limb_name)
    robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=0)
    sv = StateValidity()

    # Sample a waypoint
    waypoint = sample_loc(num_samples=1, name=limb_name)

    # CONSTRUCT A ROBOTSTATE MSG FROM SAMPLED JOINT FOR COLLISION CHECKING
    rs = RobotState()
    robot = RobotCommander()
    robot_state = robot.get_current_state()
    rs.joint_state.name = robot_state.joint_state.name
    rs.joint_state.position = list(robot_state.joint_state.position) # filler for rest of the joint angles not found in waypoint

    joint_name_indices = [rs.joint_state.name.index(n) for n in waypoint.keys()]
    for i, idx in enumerate(joint_name_indices):
        rs.joint_state.position[idx] = waypoint.values()[i]

    collision_flag = sv.getStateValidity(rs, group_name=limb_name+'_arm')
    print(collision_flag) # Boolean
    limb.move_to_joint_positions(waypoint) # moves to waypoint for visual confirmation

    # Publish collision information
    drs = DisplayRobotState()
    drs.state = rs
    robot_state_collision_pub.publish(drs)