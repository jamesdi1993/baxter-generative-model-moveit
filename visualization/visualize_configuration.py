from baxter_interfaces.env.environments import empty_environment, one_box_environment
from baxter_interfaces.sampler.self_collision_free_sampler import generate_samples
from baxter_interfaces.state_validity_check.state_validity_checker import MoveitStateValidityChecker
from baxter_interfaces.dataset_collection.path_configs import MODEL_OUTPUT_TEMPLATE
from baxter_interfaces.utils.model_loader import load_model
from baxter_interfaces.utils.positions import NEURTRAL_POSITION
from baxter_interfaces.utils.utils import convertStateToJointState
from moveit_msgs.msg import DisplayRobotState, RobotState
import argparse
import rospy
import time

def main(args):
    test = args.test
    if test:
        test_publish_robot_state()
    else:
        run_id = args.run_id
        size = args.size
        env = args.env
        display_status = args.display_status
        duration = args.duration
        model_path = MODEL_OUTPUT_TEMPLATE % (env, run_id)

        # initialize environment
        rospy.init_node('Baxter_Configuration_Visualizer')
        if env == "empty_environment":
            empty_environment()
        elif env == "one_box_environment":
            one_box_environment()

        # TODO: Add state validity check here;
        # state_validity_checker = MoveitStateValidityChecker()

        robot_state_publisher = rospy.Publisher('/display_robot_state', DisplayRobotState, queue_size=10)
        rospy.sleep(0.5)

        # Check for validity of the state
        state_validity_checker = None
        if display_status  != 'all':
            state_validity_checker = MoveitStateValidityChecker()
        print("Displaying states with status: %s" % display_status)

        model, device, d_output = load_model(model_path)
        samples = generate_samples(size, d_output, device, model)

        total_count = 0

        for i in range(samples.shape[0]):
            sample = samples[i, :]
            state_displayed = publish_robot_state(robot_state_publisher, sample, state_validity_checker,
                                                  duration=duration, display_status=display_status)
            if state_displayed:
                total_count += 1
        print("Total number of states displayed: %d" % (total_count,))

def publish_robot_state(robot_state_publisher, state, state_validity_checker=None,
                        duration=1.0, display_status='all'):
    robot_state = RobotState()
    robot_state.joint_state = convertStateToJointState(state)  # convert this to a way-point first
    # robot_state.is_diff = False
    robot_state_msg = DisplayRobotState()
    robot_state_msg.state = robot_state

    if display_status != 'all':
        result = state_validity_checker.getStateValidity(robot_state, group_name='both_arms_torso', constraints=None)
        # print("The result for validity check is: %s" % result)
        if display_status == 'valid' and result.valid or (display_status == 'inValid' and not result.valid):
            robot_state_publisher.publish(robot_state_msg)
            time.sleep(duration)
            if not result.valid:
                print("The result for validity check is: %s" % result)
            return True
    else:
        robot_state_publisher.publish(robot_state_msg)
        time.sleep(duration)
        return True
    return False

def test_publish_robot_state():
    rospy.init_node('Baxter_Configuration_Visualizer_Test')
    one_box_environment()
    robot_state_publisher = rospy.Publisher('/display_robot_state', DisplayRobotState, queue_size=10)
    rospy.sleep(0.5)

    pos = NEURTRAL_POSITION
    publish_robot_state(robot_state_publisher, display_status='all')

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')

    # model argument
    parser.add_argument('--run-id')
    parser.add_argument('--env')
    parser.add_argument('--size', type=int)
    parser.add_argument('--display-status', choices=['all','valid','inValid'])
    parser.add_argument('--duration', type=float, default='0.1')
    args, _ = parser.parse_known_args()
    main(args)
