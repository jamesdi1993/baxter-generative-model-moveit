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
from baxter_interfaces.utils.utils import convertStateToRobotState

import rospy
import time

DEFAULT_SV_SERVICE = "/check_state_validity"

class DummyStateValidityChecker(ob.StateValidityChecker):
    def __init__(self):
        super(DummyStateValidityChecker, self).__init__(si)

    def isValid(self, state):
        # Always return true
        return True

class MoveitStateValidityChecker():

    def __init__(self):
        rospy.loginfo("Initializing MoveitStateValidityChecker class")
        self.sv_srv = rospy.ServiceProxy(DEFAULT_SV_SERVICE, GetStateValidity)
        rospy.loginfo("Connecting to State Validity service")
        try:
            self.sv_srv.wait_for_service()
            rospy.wait_for_service("check_state_validity")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

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
        # if not result.valid:
        #     print("The contact information are: %s" % result.contacts)
        # print("After checking for state validity. Result: %s" % result.valid)
        return result

class MoveitOMPLStateValidityChecker(ob.StateValidityChecker):

    """
    This is the base class for StateValidity Checking;
    """
    # Initialize a StateValidity class
    def __init__(self, si):
        super(MoveitOMPLStateValidityChecker, self).__init__(si)
        self.state_validity_checker = MoveitStateValidityChecker()

    # Override the StateValidityChecker method;
    def isValid(self, state):
        # ToDo: Check the number of joints available in states;
        robot_state = convertStateToRobotState(state)
        return self.state_validity_checker.getStateValidity(robot_state).valid

    def setTimer(self):
        self.timer_activated = True

class TimedMoveitOMPLStateValidityChecker(MoveitOMPLStateValidityChecker):

    def __init__(self, si):
        super(TimedMoveitOMPLStateValidityChecker, self).__init__(si)
        self.total_collision_check_time = 0
        self.timer_activated = False
        self.counter = 0

    def isValid(self, state):
        # Not thread-safe;
        if self.timer_activated:
            start = time.time()
            isValid = super(TimedMoveitOMPLStateValidityChecker, self).isValid(state)
            self.total_collision_check_time = self.total_collision_check_time + time.time() - start
            self.counter += 1
            return isValid
        else:
            return super(TimedMoveitOMPLStateValidityChecker, self).isValid(state)

    def resetTimer(self):
        self.total_collision_check_time = 0
        self.timer_activated = True
        self.counter = 0

    def getTime(self):
        return self.total_collision_check_time, self.counter



class FCLStateValidityChecker():
    """
    This class directly uses FCL for collision checking instead of relying on the ROS interface.
    """
    def __init__(self):
        raise NotImplementedError

    def isValid(self):
        raise NotImplementedError