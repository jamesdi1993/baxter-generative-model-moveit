#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from baxter_interfaces.utils.utils import get_joint_names, convertStateToJointState


"""
Client for Getting cartesian coordinate from joint coordinate;
Adapted from: https://github.com/uts-magic-lab/moveit_python_tools/blob/master/baxter_interfaces/moveit_python_tools/get_fk.py
"""

class FKClient():

    def __init__(self):
        rospy.loginfo("Initalizing GetFK...")
        self.fk_srv = rospy.ServiceProxy('/compute_fk',
                                         GetPositionFK)
        rospy.loginfo("Waiting for /compute_fk service...")
        self.fk_srv.wait_for_service()
        rospy.loginfo("Connected!")

    def get_fk(self, joint_state, fk_links, frame_id):
        """
        Get end-effector cartesian coordinate from joint coordinates;
        """
        req = GetPositionFKRequest()
        req.header.frame_id = frame_id
        req.fk_link_names = fk_links
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
            return resp

def test_fk():
    rospy.init_node('test_fk')
    rospy.loginfo("Querying for FK")
    fkclient = FKClient()
    state = [0, -0.55, 0, 0.75, 0, 1.26, 0]
    # state = [0, 0, 0, 0, 0, 0, 0]
    joint_state = convertStateToJointState(state)
    resp = fkclient.get_fk(joint_state, ["right_hand"], "/map")
    print("The response for FK is: %s" % resp)
    for i in range(len(resp.pose_stamped)):
        position = resp.pose_stamped[i].pose.position
        rospy.loginfo("Querying Cartesian Coordinates for joint state: %s" % joint_state)
        rospy.loginfo("The positions for the joint state is: %s" % (position))


if __name__ == '__main__':
    test_fk()