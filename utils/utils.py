from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from ompl import base as ob
import numpy as np
import time

JOINT_NAMES = ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']
LIMB_NAMES = ['left', 'right']

JOINT_LIMITS = {
    's0': (-1.7016, 1.7016),
    's1': (-2.147, 1.047),
    'e0': (-3.0541, 3.0541),
    'e1': (-0.05, 2.618),
    'w0': (-3.059, 3.059),
    'w1': (-1.5707, 2.094),
    'w2': (-3.059, 3.059)
}

JOINT_INDEX = {
    's0': 0,
    's1': 1,
    'e0': 2,
    'e1': 3,
    'w0': 4,
    'w1': 5,
    'w2': 6
}

def tic():
    return time.time()

def toc(tstart):
    print("It took %s secs." % (time.time() - tstart))

def get_joint_names(limb):
    joint_name = []
    if limb in LIMB_NAMES:
        for joint in JOINT_NAMES:
            joint_name.append(limb + joint)
    elif limb is 'both_arms':
        for limb_name in LIMB_NAMES:
            for joint in JOINT_NAMES:
                joint_name.append(limb_name + joint)
    else:
        raise RuntimeError("Invalid move group encountered.")
    return joint_name

def convertStateToWaypoint(state, name):
    joint = {}
    # ref hardware specs for baxter joint limits
    joint[name + '_s0'] = float(state[0]) # s0
    joint[name + '_s1'] = float(state[1])  # s1
    joint[name + '_e0'] = float(state[2])  # e0
    joint[name + '_e1'] = float(state[3])  # e1
    joint[name + '_w0'] = float(state[4])  # w0
    joint[name + '_w1'] = float(state[5])  # w1
    joint[name + '_w2'] = float(state[6])  # w2
    return joint

def sample_loc(num_samples, name):
    joint = {}
    # ref hardware specs for baxter joint limits
    joint[name+'_s0'] = np.random.uniform(-1.7016, 1.7016, num_samples) # s0
    joint[name+'_s1'] = np.random.uniform(-2.147, 1.047, num_samples)  # s1
    joint[name+'_e0'] = np.random.uniform(-3.0541, 3.0541, num_samples) # e0
    joint[name+'_e1'] = np.random.uniform(-0.05, 2.618, num_samples) # e1
    joint[name+'_w0'] = np.random.uniform(-3.059, 3.059, num_samples) # w0
    joint[name+'_w1'] = np.random.uniform(-1.5707, 2.094, num_samples) # w1
    joint[name+'_w2'] = np.random.uniform(-3.059, 3.059, num_samples) # w2
    return joint

def convertStateToRobotState(state):
    # Map a AbstractState object in OMPL to a RobotState object in Moveit!
    rs = RobotState()
    # robot_state = robot.get_current_state()
    rs.joint_state = convertStateToJointState(state)
    return rs

def convertStateToJointState(state):
    # Map an AbstractState object in OMPL to a RobotState object in Moveit!
    js = JointState()
    # robot_state = robot.get_current_state()
    js.name = get_joint_names('right')
    js.position = [state[0], state[1], state[2], state[3], state[4], state[5], state[6]]
    return js

def construct_robot_state(space, waypoint):
    """
    Map a waypoint to an AbstractState object in OMPL
    """
    state = ob.State(space)
    for joint, value in waypoint.items():
        joint_name = joint.split('_')[1] # example waypoint: {'right_e0': 0, 'right_e1': 0, ...}
        index = JOINT_INDEX[joint_name]
        state[index] = value
    return state

def generate_random_state(space, limb_name):
    waypoint = sample_loc(1, limb_name)
    return construct_robot_state(space, waypoint)

def test_convert_state_to_robot_state():
    space = ob.RealVectorStateSpace(7)
    state = ob.State(space)
    state[0] = 0
    state[1] = -0.55
    state[2] = 0
    state[3] = 0.75
    state[4] = 0
    state[5] = 1.26
    state[6] = 0

    # print("The list of states is: %s" % state)

    robot_state = convertStateToRobotState(state)
    expected_joint_names =  ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    expected_joint_values = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
    assert robot_state.joint_state.name ==  ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'],\
        "joint names do not match! Expected: %s, Actual %s" % (expected_joint_names, robot_state.joint_state.name)
    assert robot_state.joint_state.position == expected_joint_values, \
        "joint values do not match! Expected: %s, Actual %s" % (expected_joint_values, robot_state.joint_state.position)

    print("Test for converting AbstractState to RobotState passed!")


def test_get_joint_name():
    limb_name = 'right'
    joint_names = get_joint_names(limb_name)
    expected_joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    assert joint_names == expected_joint_names, "Joint names do not match. Expected: %s, Actual: %s" % \
                                                (expected_joint_names, joint_names)

if __name__ is "__main__":
    test_get_joint_name()
    test_convert_state_to_robot_state()