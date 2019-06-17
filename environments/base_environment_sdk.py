from moveit_commander import PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, DisplayTrajectory
from src.utils.utils import sample_loc

import baxter_interface
import rospy
import moveit_commander

"""
Base class for all baxter-based environment, using Baxter SDK;
An environment consists of:
-a robot (robot description and robot model)
-a Planning Scene; 
-a Limb that can be moved (either right, left, or both-arms)
"""
class BaseEnvironment():

    def __init__(self, limb_name):
        rospy.init_node('Baxter_Env')
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.scene._scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=0)
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.group = moveit_commander.MoveGroupCommander(limb_name + "_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=0)
        rospy.sleep(2)
        print("Initialized Base environment.")

    def construct_robot_state(self, joint):
        rs = RobotState()
        robot_state = self.robot.get_current_state()
        rs.joint_state.name = robot_state.joint_state.name
        rs.joint_state.position = list(robot_state.joint_state.position) # filler for rest of the joint angles not found in waypoint

        joint_name_indices = [rs.joint_state.name.index(n) for n in waypoint.keys()]
        for i, idx in enumerate(joint_name_indices):
            rs.joint_state.position[idx] = waypoint.values()[i]
        return rs

    def move_to_random_state(self):
        waypoint = sample_loc(1, self.limb_name)
        self.limb.move_to_joint_positions(waypoint) # Baxter SDK;
        print("Moving robot arms to waypoint: %s" % (waypoint,))


    def move_to_goal(self, goal):
        # Move the limb to goal; 
        self.group.set_joint_value_target(goal)
        result = self.group.go()
        print "============ Waiting while moving robot to goal."
        print "============ Printing result after moving: %s" % result

    def plan_trajectory_in_cspace(self, goal, visualization = True):
        ## Then, we will get the current set of joint values for the group
        self.group.set_joint_value_target(goal)
        plan = self.group.plan()

        if visualization:
            print "============ Visualizing trajectory_plan"
            display_trajectory = DisplayTrajectory()

            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self.display_trajectory_publisher.publish(display_trajectory);

            print "============ Waiting while plan is visualized (again)..."
            rospy.sleep(5)
        return plan

if __name__ == '__main__':
    limb_name = 'right'
    env = BaseEnvironment(limb_name)
    # env.move_to_random_state()
    random_goal = sample_loc(1, limb_name)
    env.move_to_goal(random_goal)
    # plan = env.plan_trajectory_in_cspace(goal=random_goal, visualization=True)

