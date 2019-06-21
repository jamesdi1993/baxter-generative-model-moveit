try:
    from ompl import base as ob
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from ompl.base import State
from ..utils.utils import get_joint_names, convertStateToRobotState

import rospy

DEFAULT_SV_SERVICE = "/check_state_validity"

class MoveitStateValidityChecker(ob.StateValidityChecker):
    """
    This is the base class for StateValidity Checking;
    """
    # Initialize a StateValidity class
    def __init__(self, si):
        super(MoveitStateValidityChecker, self).__init__(si)
        rospy.loginfo("Initializing MoveitStateValidityChecker class")
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
            rospy.logwarn(
                "Param '/play_motion/approach_planner/planning_groups' not set. We can't guess controllers")
        rospy.loginfo("Ready for making Validity calls")

    def close_SV(self):
        self.sv_srv.close()

    # Override the StateValidityChecker method;
    def isValid(self, state):
        # ToDo: Check the number of joints available in states;
        robot_state = convertStateToRobotState(state)
        return self.getStateValidity(robot_state).valid

    def getStateValidity(self, robot_state, group_name='both_arms_torso', constraints=None):
        """Given a RobotState and a group name and an optional Constraints
        return the validity of the State"""
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        # if not result.valid:
        #     print("The contact information are: %s" % result.contacts)
        # print("After checking for state validity. Result: %s" % result.valid)
        return result

class FCLStateValidityChecker():
    """
    This class directly uses FCL for collision checking instead of relying on the ROS interface.
    """
    def __init__(self):
        raise NotImplementedError

    def isValid(self):
        raise NotImplementedError