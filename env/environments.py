from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped
import argparse
import rospy

def sendColors(color_dict, scene):
    # Need to send a planning scene diff
    p = PlanningScene()
    p.is_diff = True
    for color in color_dict.values():
        p.object_colors.append(color)
    scene._scene_pub.publish(p)
    rospy.sleep(1)

def setColor(name, color_dict, rgb, a=1.0):
    # Create our color
    color = ObjectColor()
    color.id = name
    color.color.r = rgb[0]
    color.color.g = rgb[1]
    color.color.b = rgb[2]
    color.color.a = a
    color_dict[name] = color

def add_box(scene, name, pose, size):
    # center box
    p = PoseStamped()
    p.header.frame_id = "/world"
    p.pose.position.x = pose[0] # position of the box in the world frame
    p.pose.position.y = pose[1]
    p.pose.position.z = pose[2]
    p.pose.orientation.x = pose[3] # orientation, in quaternion;
    p.pose.orientation.y = pose[4]
    p.pose.orientation.z = pose[5]
    p.pose.orientation.w = pose[6]

    scene.add_box(name, p, size=size)  # dimension of the box
    rospy.sleep(1)

def initialize_environment():
    # Assume rosnode is already is initialized;
    scene = PlanningSceneInterface()
    scene._scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=0)
    rospy.sleep(2)
    return scene

def empty_environment():
    scene = initialize_environment()
    known_objects = scene.get_known_object_names()
    for object in known_objects:
        scene.remove_world_object(object)
    rospy.sleep(1)

def one_box_environment():
    scene = initialize_environment()

    # clean environment
    scene.remove_world_object("box_center")
    rospy.sleep(1)

    # add object
    add_box(scene, "box_center", (0.8, 0.0, 0.3, 1., 0., 0., 0.), (.25, .25, .25))
    print(scene.get_known_object_names())

    # set color for object
    box_center_color = [0.0, 1.0, 0.0]  # green box
    _colors = {}

    setColor('box_center', _colors, box_center_color)
    sendColors(_colors, scene)

def one_pillar_environment():
    scene = initialize_environment()

    # clean environment
    scene.remove_world_object("pillar")
    rospy.sleep(1)

    # add object
    add_box(scene, "pillar", (0.8, 0.0, 0.3, 1., 0., 0., 0.), (.55, .55, 1.5))
    print(scene.get_known_object_names())

    # set color for object
    box_center_color = [0.0, 1.0, 0.0]  # green box
    _colors = {}

    setColor('box_center', _colors, box_center_color)
    sendColors(_colors, scene)

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--env')
    args, _ = parser.parse_known_args()
    env = args.env

    rospy.init_node('EnvironmentSetter')
    if env == "empty_environment":
        empty_environment()
    elif env == "one_box_environment":
        one_box_environment()
    elif env == "one_pillar_environment":
        one_pillar_environment()











