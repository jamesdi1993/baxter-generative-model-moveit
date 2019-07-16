from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from src.env.environments import one_box_environment, empty_environment
from src.utils.utils import get_joint_names

import copy as cp
import csv
import os
import time

BATCH_SIZE  = 1000

def check_config(config, state_validity_checker, limb_name):
    """
    Check the validity of a configuration
    :param : The configurations to check against.
    :param state_validity_checker: The validity checker
    :param space: The space to check for
    :param limb_name: The name of the limb.
    :return: The validty of the configurations.
    """
    # Construct a robotstate object from a dictionary
    rs = RobotState()
    js = JointState()
    js.name = get_joint_names('right')
    js.position = [config[joint] for joint in js.name]
    rs.joint_state = js
    result = state_validity_checker.getStateValidity(rs, group_name=limb_name + '_arm')
    return result.valid

def augment_dataset(file_name, env_name, label_generator):
    """
    Augment the dataset with newly generated labels;
    :param file_name: The file to read data from
    :param env_name: The environment for the dataset
    :param label_generator: The generator for the label
    :return: file: The file that contains the new_dataset
    """
    print("Augmenting labels %s for: %s" % (label_generator.headers, file_name))
    start = time.time()

    # initialize_environment
    if env_name == 'one_box_environment':
        one_box_environment() # initialize env
    elif env_name == 'empty_environment':
        empty_environment()

    # Get file name;
    file_prefix = os.path.abspath(file_name).split('.')[0]
    output_file_name = file_prefix + "_%s.csv" % label_generator.label

    with open(file_name, mode='rb') as input_file:
        csv_reader = csv.DictReader(input_file, quoting=csv.QUOTE_NONNUMERIC)
        lines_read = 0
        headers = cp.deepcopy(csv_reader.fieldnames)
        headers += label_generator.headers

        print("Writing to output file: %s" % output_file_name)

        with open(output_file_name, mode='w') as output_file:
            # Quote header such that it is easier for reader to parse data
            writer = csv.DictWriter(output_file, fieldnames=headers, quoting=csv.QUOTE_NONNUMERIC)
            writer.writeheader()
            points = []
            batch = 0

            for waypoint in csv_reader:
                label_generator.fill_label(waypoint)
                points.append(waypoint)
                lines_read += 1
                if lines_read % BATCH_SIZE == 0:
                    writer.writerows(points)
                    points = []
                    batch += 1
                    print("Finished generating data for batch: %s" % (batch))
            writer.writerows(points)
    print("Finished generating augmented dataset. Time Elasped: %s" % (time.time() - start))
    return output_file