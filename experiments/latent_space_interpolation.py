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

from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from baxter_interfaces.dataset_collection.path_configs import MODEL_OUTPUT_TEMPLATE
from baxter_interfaces.state_validity_check.state_validity_checker import MoveitStateValidityChecker
from baxter_interfaces.sampler.self_collision_free_sampler import VAE
from baxter_interfaces.utils.utils import convertStateToJointState, convertStateToRobotState, convertWaypointToJointState, JOINT_LIMITS
from baxter_interfaces.utils.utils import get_joint_names, sample_loc
from time import sleep
from baxter_interfaces.forward_kinematics.fkClient import FKClient
from baxter_interfaces.visualization.workspace_plot_rviz import WaypointPublisher
from baxter_interfaces.env.environments import one_box_environment

import argparse
import numpy as np
import rospy
import time
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

def visualize_segment(fkclient, marker_publisher, edge, color):
    """
    Visualize the interpolated data points along an edge, after applying forward kinematics to them.
    :param edge: The edge to check for; A list of tuple (position, isValid)
    :return: N/A
    """
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

def get_cfree_endpoints(state_validity_checker, fk_client, include_pos):
    # print("Include position is: %s" % include_pos)
    endpoints = []
    while len(endpoints) < 2:
        samples = sample_loc(1, 'right')
        joint_value_array = zip(*samples.values())
        joint_array = map(lambda joint: dict(zip(samples.keys(), joint)), joint_value_array)
        joint_state = convertWaypointToJointState(joint_array[0])
        # print("The joint state is: %s" % (joint_state,))
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        isValid = state_validity_checker.getStateValidity(robot_state).valid
        if isValid:
            if include_pos:
                position = fk_client.get_fk(joint_state, ["right_hand"], "world").pose_stamped[0].pose.position
                pos = [position.x, position.y, position.z]
                endpoints.append(joint_state.position + pos)
            else:
                endpoints.append(joint_state.position)
    return endpoints

def test_set_max_segment_truncated_gaussian():
    mean = np.zeros(7)
    std = np.ones(7)
    max_segment_length = set_max_segment_truncated_gaussian(mean, std, 0.01)
    assert np.allclose(0.158745, max_segment_length, atol=0.000001), "The expected max length and actual max length " \
        + " are not equal. Expected: %s; Actual: %s" % (0.158745, max_segment_length)
    print("Test for set_max_truncated_gaussian passed.")

def generate_edges(model, device, state_validity_checker, fk_client, include_pos):
    max_segment_factor = 0.01

    endpoints = get_cfree_endpoints(state_validity_checker, fk_client, include_pos)
    start = endpoints[0]
    end = endpoints[1]
    # print("The c_free endpoints are: %s" % endpoints,)

    start_latent = start
    end_latent = end
    with torch.no_grad():
        mu_s, logvar_s = model.encode(torch.from_numpy(np.expand_dims(start_latent, axis=0)).float().to(device))
        z_s = model.reparameterize(mu_s, logvar_s)
        start_latent = z_s.numpy()[0, :]

        mu_e, logvar_e = model.encode(torch.from_numpy(np.expand_dims(end_latent, axis=0)).float().to(device))
        z_e = model.reparameterize(mu_e, logvar_e)
        end_latent = z_e.numpy()[0, :]

    # print("The start position is: %s" % start_pos)
    # print("The end position is: %s" % end_pos)

    latent_max_segment = set_max_segment_truncated_gaussian(np.zeros(d_output), np.ones(d_output), max_segment_factor)
    latent_path = linear_interpolation(start_latent, end_latent, latent_max_segment)

    # Generate decoded path;
    decoded_path = []
    for pos in latent_path:
        with torch.no_grad():
            point = torch.from_numpy(pos).float().to(device) # latent_space sample
            sample = model.decode(point).cpu().double().numpy() # c_space sample
            robot_state = convertStateToRobotState(sample[:7])
            isValid = state_validity_checker.getStateValidity(robot_state).valid# checking collision status in c-space;
            decoded_path.append((sample, isValid))

    cspace_max_segment = set_max_segment_cspace(np.array(JOINT_LIMITS.values()), max_segment_factor)
    cspace_path = []

    start_cspace = np.array(start)[:7]
    end_cspace = np.array(end)[:7]
    path = linear_interpolation(start_cspace, end_cspace, cspace_max_segment)
    for pos in path:
        robot_state = convertStateToRobotState(pos)
        isValid = state_validity_checker.getStateValidity(robot_state).valid  # checking collision status in c-space;
        cspace_path.append((pos, isValid))
    return decoded_path, cspace_path
    # print("The path is: %s" % path)

if __name__=="__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id')
    parser.add_argument('--env')
    parser.add_argument('--times', type=int)
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--include-pos', action='store_true')

    # TODO: pass the parameters of the model from the pth file itself;

    parser.add_argument('--d-input', type=int)
    parser.add_argument('--h-dim1', type=int)
    parser.add_argument('--h-dim2', type=int)
    parser.add_argument('--d-output', type=int)

    args, _ = parser.parse_known_args()
    run_id = args.run_id
    env = args.env
    t = args.times
    d_input = args.d_input
    h_dim1 = args.h_dim1
    h_dim2 = args.h_dim2
    d_output = args.d_output
    visualize = args.visualize
    include_pos = args.include_pos
    model_path = MODEL_OUTPUT_TEMPLATE % (env,  run_id)

    # test_set_max_segment_truncated_gaussian()
    rospy.init_node('latent_space_experiments')
    one_box_environment()

    # initialize forward kinematics and marker publishing client;
    fkclient = FKClient()
    marker_publisher = WaypointPublisher()
    rospy.sleep(0.5)

    # load the generator model;
    model = VAE(d_input, h_dim1, h_dim2, d_output)
    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    model.eval()

    # set up device;
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    state_validity_checker = MoveitStateValidityChecker()

    # 6 x t array; Start_free, End_free, latent_edge_in_collision, latent_edge_length, cspace_edge_in_collision, cspace_edge_length
    stats = np.zeros((6, t))
    for i in range(t):
        latent_edge, cspace_edge = generate_edges(model, device, state_validity_checker, fkclient, include_pos)
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

        latent_color = np.array([[1.0, 1.0, 0.0], [0.0, 0.0, 1.0]]) # blue
        cspace_color = np.array([[1.0, 1.0, 0.0], [1.0, 0.0, 0.0]]) # red
        if visualize:
            visualize_segment(fkclient, marker_publisher, latent_edge, latent_color)
            time.sleep(3.0)
            visualize_segment(fkclient, marker_publisher, cspace_edge, cspace_color)
            time.sleep(3.0)

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