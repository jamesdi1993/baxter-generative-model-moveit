from baxter_interfaces.dataset_collection.label_generators import EnvironmentCollisionLabelGenerator, \
    SelfCollisionLabelGenerator, EndEffectorPositionGenerator
from baxter_interfaces.dataset_collection.common import augment_dataset
from baxter_interfaces.dataset_collection.path_configs import VALIDATED_OUTPUT_TEMPLATE, SAMPLED_OUTPUT_TEMPLATE, TEMP_VALIDATED_OUTPUT_TEMPLATE

import argparse
import rospy
import sys
import time

def main(args):
    rospy.init_node('BaxterEnvironmentCollisionValidator')
    env = args.env
    run_id = args.run_id
    start = int(time.time())


    self_collision_label_generator = SelfCollisionLabelGenerator(env_name=env)

    input_file = SAMPLED_OUTPUT_TEMPLATE % (env, run_id)
    self_collision_validated_file = TEMP_VALIDATED_OUTPUT_TEMPLATE % (env, run_id)
    augment_dataset(input_file=input_file, output_file=self_collision_validated_file, env_name="empty_environment",
                              label_generator=self_collision_label_generator)  # generate the new dataset;

    env_collision_generator = EnvironmentCollisionLabelGenerator(env_name=env)
    output_file = VALIDATED_OUTPUT_TEMPLATE % (env, run_id)
    augment_dataset(input_file=self_collision_validated_file, output_file=output_file,
                    env_name=env,
                    label_generator=env_collision_generator)
    print("Finished validating the dataset for environment: %s; Output file: %s" % (env, output_file))
    print("Time elapsed: %s" % (time.time() - start))

if __name__=='__main__':
    parser = argparse.ArgumentParser()

    # dataset configurations
    parser.add_argument('--env')
    parser.add_argument('--run-id')
    args, _ = parser.parse_known_args()
    main(args)