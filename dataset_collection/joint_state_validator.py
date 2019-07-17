#!/usr/bin/python
from moveit_msgs.msg import DisplayRobotState
from joint_state_generator import COLLISION_KEY, FILE_DELIMETER, initialize_environment, get_current_state, fill_waypoint
from baxter_interfaces.demo import StateValidity

import baxter_interface
import copy as cp
import csv
import rospy
import sys
import time

COLLISION_KEY = 'collisionFree'

def visualize_waypoint(limb, waypoint, robot_state_collision_pub, publish_collision = False):    
    limb.move_to_joint_positions(waypoint) # moves to waypointO for visual confirmation

    if not publish_collision and limb is not None:
        # Publish collision information
        drs = DisplayRobotState()
        drs.state = rs
        robot_state_collision_pub.publish(drs)

def checkCollisionFlag(sv, limb_name, waypoint):
    # Deep copy the waypoint first, then pop the key
    point = cp.deepcopy(waypoint)
    collisionFree = point[COLLISION_KEY]
    point.pop(COLLISION_KEY)

    current_state = get_current_state()
    rs = fill_waypoint(point, current_state)

    collisionflag = sv.getStateValidity(rs, group_name=limb_name+'_arm').valid
    print("Collision flag: %s" % bool(collisionflag))
    assert collisionflag == collisionFree, "Collisionflag does not agree. CollisionFlag in data: %d; Current Flag: %d; \nWaypoint: %s"\
    % (collisionFree, collisionflag, point)


if __name__ == '__main__':
    file = sys.argv[1]
    number_of_points = int(sys.argv[2])
    print("Checking the first %d waypoints in file: %s" % (number_of_points, file))

    scene, robot = initialize_environment()
    rospy.sleep(2)
    robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=0)
    sv = StateValidity()

    limb_name = file.split(FILE_DELIMETER)[0]
    limb = baxter_interface.Limb(limb_name)
    # robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=0)
    sv = StateValidity()

    with open(file, mode='rb') as csv_file:                                                                                                                                                                                        
        csv_reader = csv.DictReader(csv_file, quoting=csv.QUOTE_NONNUMERIC)
        lines_read = 0
        for waypoint in csv_reader:
            if (lines_read > number_of_points):
                break
            else:
                print("Checking waypoint %d" % (lines_read + 1))
                print(waypoint)
                checkCollisionFlag(sv, limb_name, waypoint)
                waypoint.pop(COLLISION_KEY)
                lines_read += 1

                visualize_waypoint(limb, waypoint, False, robot_state_collision_pub)
                # Sleep 5 seconds for wavepoint to finish.
                time.sleep(5)
