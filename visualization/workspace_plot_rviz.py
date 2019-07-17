from baxter_interfaces.forward_kinematics.fkClient import FKClient
from baxter_interfaces.sampler.self_collision_free_sampler import VAE, generate_samples
from baxter_interfaces.utils.utils import convertStateToJointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from moveit_msgs.msg import RobotState
import torch
import rospy

"""
Adapted from this source:
https://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/Lab4/waypoint.py
"""
class WaypointPublisher():

    def __init__(self):
        # rospy.init_node('WaypointPublisher')
        self.publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        rospy.sleep(0.5) # sleep for 0.5 seconds;
        self.marker_id = 0

    def publish_waypoints(self, waypoints, collision_free=True, scale=[0.01, 0.01, 0.01], t=100, color=None):
        """
        Publish markers for waypoints;
        :param waypoints: The waypoints to publish;
        :return: N/A
        """
        points = Marker()
        points.header.frame_id = "/world"  # publish point in the origin frame;
        points.type = points.POINTS
        points.action = points.ADD
        points.lifetime = rospy.Duration(t)
        points.id = self.marker_id
        self.marker_id += 1
        points.scale.x = scale[0]
        points.scale.y = scale[1]
        points.scale.z = scale[2]
        points.color.a = 1.0  # alpha;
        if color is None:
            if not collision_free:
                points.color.r = 1.0  # yellow
                points.color.g = 1.0
                points.color.b = 0.0
            else:
                points.color.r = 0.0  # blue
                points.color.g = 0.0
                points.color.b = 1.0
        else:
            if not collision_free:
                points.color.r = color[0, 0]
                points.color.g = color[0, 1]
                points.color.b = color[0, 2]
            else:
                points.color.r = color[1, 0]
                points.color.g = color[1, 1]
                points.color.b = color[1, 2]
        points.pose.orientation.w = 1.0 # orientation;
        points.points = waypoints
        # Publish the MarkerArray
        self.publisher.publish(points)

# def convert_sample_to_end_pos(sample):
#     # rs = convertStateToRobotState(sample)
#     # end_effector_pose = rs.getGlobalLinkTransform("right_gripper")
#     rs = RobotState()
#     # robot_state = robot.get_current_state()
#     rs.joint_state.name = get_joint_names('right')
#     rs.joint_state.position = [sample[0], sample[1], sample[2], sample[3], sample[4], sample[5], sample[6]]
#     pos = rs.getFrameTransform("right_gripper")
#     # pos = end_effector_pose.translation()
#
#     print("The robot joint configuration is: %s" % sample)
#     print("The end effector position is: %s" % pos)
#     return pos

def load_model():
    d_input = 7
    h_dim1 = 256
    h_dim2 = 100
    d_output = 7  # latent layer;

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = VAE(d_input, h_dim1, h_dim2, d_output)
    model.load_state_dict(torch.load("./data/model/model.pth"))
    model.eval()
    return model, device, d_output

if __name__=="__main__":
    # Publishing for an environment;
    rospy.init_node('WaypointPublisher')
    model, device, d_output = load_model()
    batch_size = 10000
    samples = generate_samples(batch_size, d_output, device, model)
    fkclient = FKClient()
    publisher = WaypointPublisher()

    # Iterate over samples
    for i in range(samples.shape[0]):
        sample = samples[i]
        joint_state = convertStateToJointState(sample)
        resp = fkclient.get_fk(joint_state, ["right_hand"], "world")
        # print("The response for FK is: %s" % resp)
        # Iterate over each link in pose;
        for i in range(len(resp.pose_stamped)):
            position = resp.pose_stamped[i].pose.position
            # print("The position of the data point is: %s" % position)
            publisher.publish_waypoints([position])

