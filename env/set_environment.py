import rospy
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped, Point
from moveit_msgs.msg import ObjectColor, PlanningScene

def setColor(name, color_dict, rgb, a):
    # Create our color
    color = ObjectColor()
    color.id = name
    color.color.r = rgb[0]
    color.color.g = rgb[1]
    color.color.b = rgb[2]
    color.color.a = a
    color_dict[name] = color

def sendColors(color_dict, scene):
    # Need to send a planning scene diff
    p = PlanningScene()
    p.is_diff = True
    for color in color_dict.values():
        p.object_colors.append(color)
    scene._scene_pub.publish(p)

def color_norm(col):
    norm_col = [x/255.0 for x in col]
    return norm_col

def main():
    rospy.init_node('Baxter_Env')
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    scene._scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=0)
    rospy.sleep(2)

    # centre table
    p1 = PoseStamped()
    p1.header.frame_id = robot.get_planning_frame()
    p1.pose.position.x = 1. # position of the table in the world frame
    p1.pose.position.y = 0.
    p1.pose.position.z = 0.

    # left table (from baxter's perspective)
    p1_l = PoseStamped()
    p1_l.header.frame_id = robot.get_planning_frame()
    p1_l.pose.position.x = 0.5 # position of the table in the world frame
    p1_l.pose.position.y = 1.
    p1_l.pose.position.z = 0.
    p1_l.pose.orientation.x = 0.707
    p1_l.pose.orientation.y = 0.707

    # right table (from baxter's perspective)
    p1_r = PoseStamped()
    p1_r.header.frame_id = robot.get_planning_frame()
    p1_r.pose.position.x = 0.5 # position of the table in the world frame
    p1_r.pose.position.y = -1.
    p1_r.pose.position.z = 0.
    p1_r.pose.orientation.x = 0.707
    p1_r.pose.orientation.y = 0.707

    # open back shelf
    p2 = PoseStamped()
    p2.header.frame_id = robot.get_planning_frame()
    p2.pose.position.x = 1.2 # position of the table in the world frame
    p2.pose.position.y = 0.0
    p2.pose.position.z = 0.75
    p2.pose.orientation.x = 0.5
    p2.pose.orientation.y = -0.5
    p2.pose.orientation.z = -0.5
    p2.pose.orientation.w = 0.5

    pw = PoseStamped()
    pw.header.frame_id = robot.get_planning_frame()

    # add an object to be grasped
    p_ob1 = PoseStamped()
    p_ob1.header.frame_id = robot.get_planning_frame()
    p_ob1.pose.position.x = .92
    p_ob1.pose.position.y = 0.3
    p_ob1.pose.position.z = 0.89

    # the ole duck
    p_ob2 = PoseStamped()
    p_ob2.header.frame_id = robot.get_planning_frame()
    p_ob2.pose.position.x = 0.87
    p_ob2.pose.position.y = -0.4
    p_ob2.pose.position.z = 0.24

    # clean environment
    scene.remove_world_object("table_center")
    scene.remove_world_object("table_side_left")
    scene.remove_world_object("table_side_right")
    scene.remove_world_object("shelf")
    scene.remove_world_object("wall")
    scene.remove_world_object("part")
    scene.remove_world_object("duck")

    rospy.sleep(1)

    scene.add_box("table_center", p1, size=(.5, 1.5, 0.4)) # dimensions of the table
    scene.add_box("table_side_left", p1_l, size=(.5, 1.5, 0.4))
    scene.add_box("table_side_right", p1_r, size=(.5, 1.5, 0.4))
    scene.add_mesh("shelf", p2, "/home/nikhildas/ros_ws/baxter_interfaces/baxter_moveit_config/baxter_scenes/bookshelf_light.stl", size=(.025, .01, .01))
    scene.add_plane("wall", pw, normal=(0, 1, 0))

    part_size = (0.07, 0.05, 0.12)
    scene.add_box("part", p_ob1, size=part_size)
    scene.add_mesh("duck", p_ob2, "/home/nikhildas/ros_ws/baxter_interfaces/baxter_moveit_config/baxter_scenes/duck.stl", size=(.001, .001, .001))

    rospy.sleep(1)

    print(scene.get_known_object_names())

    ## ---> SET COLOURS

    table_color = color_norm([105, 105, 105]) # normalize colors to range [0, 1]
    shelf_color = color_norm([139, 69, 19])
    duck_color = color_norm([255, 255, 0])

    _colors = {}

    setColor('table_center', _colors, table_color, 1)
    setColor('table_side_left', _colors, table_color, 1)
    setColor('table_side_right', _colors, table_color, 1)
    setColor('shelf', _colors, shelf_color, 1)
    setColor('duck', _colors, duck_color, 1)
    sendColors(_colors, scene)

if __name__ == '__main__':
    main()
