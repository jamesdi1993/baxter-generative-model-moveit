try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og

from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from src.environments.base_environment import initialize_space
from src.state_validity_check.state_validity_checker import MoveitStateValidityChecker
from src.sampler.self_collision_free_sampler import VAE
from src.utils.utils import convertStateToJointState, convertStateToRobotState, JOINT_LIMITS
from src.utils.utils import get_joint_names
from time import sleep
from src.forward_kinematics.fkClient import FKClient
from src.visualization.workspace_plot_rviz import WaypointPublisher

import numpy as np
import rospy
import torch

def linear_interpolation(start, end, max_segment):
    dir = end - start
    edge_length = np.linalg.norm(dir)
    num_points = np.ceil(edge_length / max_segment) + 1 # starting point + path_points;
    midpoints = np.linspace(0, 1, num=num_points)

    path = []
    for i in range(midpoints.shape[0]):
        pos = start + dir * midpoints[i]
        path.append(pos)
    return path

def cubic_interpolation(start, end, max_segment):
    pass

def visualize_movement(start, path):
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=100) # for visualizing the robot movement;

    sleep(0.5)

    display_trajectory = DisplayTrajectory()

    display_trajectory.trajectory_start = convertStateToRobotState(start)
    trajectory = RobotTrajectory()
    points = []
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = get_joint_names('right')
    for state, _ in path:
        point = JointTrajectoryPoint()
        point.positions = convertStateToRobotState(state).joint_state.position
        points.append(point)
    joint_trajectory.points = points
    trajectory.joint_trajectory = joint_trajectory

    # print("The joint trajectory is: %s" % trajectory)

    display_trajectory.trajectory.append(trajectory)
    display_trajectory_publisher.publish(display_trajectory)

def visualize_segment(edge, color):
    """
    Visualize the interpolated data points along an edge, after applying forward kinematics to them.
    :param edge: The edge to check for; A list of tuple (position, isValid)
    :return: N/A
    """
    fkclient = FKClient()
    marker_publisher = WaypointPublisher()
    rospy.sleep(0.5)

    for (point, isValid) in edge:
        joint_state = convertStateToJointState(point)
        resp = fkclient.get_fk(joint_state, ["right_hand"], "world")
        position = resp.pose_stamped[0].pose.position
        marker_publisher.publish_waypoints([position], isValid, scale=[0.02, 0.02, 0.02], t=5, color=color)

def set_max_segment_truncated_gaussian(mean, std, max_segment_factor = 0.01):
    dist = 6 * std # 6 standard deviation;
    return np.linalg.norm(dist) * max_segment_factor

def set_max_segment_cspace(limits, max_segment_factor = 0.01):
    dist = limits[:, 1] - limits[:, 0]
    return np.linalg.norm(dist) * max_segment_factor

def test_set_max_segment_truncated_gaussian():
    mean = np.zeros(7)
    std = np.ones(7)
    max_segment_length = set_max_segment_truncated_gaussian(mean, std, 0.01)
    assert np.allclose(0.158745, max_segment_length, atol=0.000001), "The expected max length and actual max length " \
        + " are not equal. Expected: %s; Actual: %s" % (0.158745, max_segment_length)
    print("Test for set_max_truncated_gaussian passed.")

def generate_edge():
    rospy.init_node('latent_space_experiments')
    space = initialize_space()
    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    state_validity_checker = MoveitStateValidityChecker(si)
    # TODO: Implement a state validity checker
    ss.setStateValidityChecker(state_validity_checker)

    d_input = 7
    h_dim1 = 256
    h_dim2 = 100
    d_output = 7  # latent layer;
    max_segment_factor = 0.1

    # load the generator model;
    model = VAE(d_input, h_dim1, h_dim2, d_output)
    model.load_state_dict(torch.load("./data/model/model.pth"))
    model.eval()

    # set up device;
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # TODO: Start with two data points in cspace first;
    endpoints = np.random.randn(2, d_input)
    start_pos = endpoints[0, :]
    end_pos = endpoints[1, :]

    print("The start position is: %s" % start_pos)
    print("The end position is: %s" % end_pos)

    latent_max_segment = set_max_segment_truncated_gaussian(np.zeros(d_output), np.ones(d_output), max_segment_factor)
    latent_path = linear_interpolation(start_pos, end_pos, latent_max_segment)

    # Generate decoded path;
    decoded_path = []
    for pos in latent_path:
        with torch.no_grad():
            point = torch.from_numpy(pos).float().to(device) # latent_space sample
            sample = model.decode(point).cpu().double().numpy() # c_space sample
            state = ob.State(space)
            for i in range(sample.shape[0]):
                state[i] = sample[i]
            isValid = state_validity_checker.isValid(state) # checking collision status in c-space;
            decoded_path.append((sample, isValid))

    cspace_max_segment = set_max_segment_cspace(np.array(JOINT_LIMITS.values()), max_segment_factor)
    cspace_path = []

    with torch.no_grad():
        start_cspace = model.decode(torch.from_numpy(start_pos).float().to(device)).cpu().double().numpy()
        end_cspace = model.decode(torch.from_numpy(end_pos).float().to(device)).cpu().double().numpy()
        path = linear_interpolation(start_cspace, end_cspace, cspace_max_segment)
        for pos in path:
            state = ob.State(space)
            for i in range(sample.shape[0]):
                state[i] = pos[i]
            isValid = state_validity_checker.isValid(state)  # checking collision status in c-space;
            cspace_path.append((state, isValid))
    return decoded_path, cspace_path
    # print("The path is: %s" % path)

if __name__=="__main__":
    test_set_max_segment_truncated_gaussian()

    t = 2000
    # 6 x t array; Start_free, End_free, latent_edge_in_collision, latent_edge_length, cspace_edge_in_collision, cspace_edge_length
    stats = np.zeros((6, t))
    for i in range(t):
        latent_edge, cspace_edge = generate_edge()
        latent_collision_free_status = np.array(zip(*latent_edge)[1]) # get only the collision status;
        cspace_collision_free_status = np.array(zip(*cspace_edge)[1])

        # print("collision_free_status is: %s" % collision_free_status)
        stats[0, i] = latent_collision_free_status[0]
        stats[1, i] = latent_collision_free_status[-1]
        if 0 in latent_collision_free_status[1:-1]:
            stats[2, i] = 0 # if any point is in collision;
        else:
            stats[2, i] = 1 # if no point is in collision;
        stats[3, i] = len(latent_edge)

        if 0 in cspace_collision_free_status[1:-1]:
            stats[4, i] = 0 # if any point is in collision;
        else:
            stats[4, i] = 1 # if no point is in collision;
        stats[5, i] = len(cspace_edge)

        # visualize_movement(latent_edge[0][0], latent_edge) # get only the start position

        latent_color = np.array([[1.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        cspace_color = np.array([[1.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
        visualize_segment(latent_edge, latent_color)
        visualize_segment(cspace_edge, cspace_color)
        # sleep(5) # sleep for 5 seconds for visual examination

    # print("The statistics are: Start (Collision-Free), End (Collision-free), Edge (Collision-free)")
    # print(stats)
    # print("------------------------------")
    endpoints_collision_free = np.logical_and(stats[0, :], stats[1, :])
    latent_edge_collision_free = np.logical_and(stats[2, :], endpoints_collision_free)
    cspace_edge_collision_free = np.logical_and(stats[4, :], endpoints_collision_free)
    # print("Endpoints collision free is: %s" % (endpoints_collision_free,))
    # print("Edge collision free is: %s" % (edge_collision_free,))
    print("P(latent_edge_collision_free | endpoints_collision_free) = %s" % (np.sum(latent_edge_collision_free, dtype=float) / np.sum(endpoints_collision_free, dtype=float)))
    print("P(cspace_edge_collision_free | endpoints_collision_free) = %s" % (
                np.sum(cspace_edge_collision_free, dtype=float) / np.sum(endpoints_collision_free, dtype=float)))
    print("P(endpoints_collision_free) = %s" % (np.sum(endpoints_collision_free, dtype=float) / t))
    print("Average number of node in the latent interpolated path: %s" % (np.sum(stats[3, :]) / t))
    print("Average number of node in the cspace interpolated path: %s" % (np.sum(stats[5, :]) / t))