from src.dataset_collection.label_generators import EnvironmentCollisionLabelGenerator, SelfCollisionLabelGenerator
from src.dataset_collection.common import augment_dataset

import argparse
import rospy
import sys
import time

def main(args):
    rospy.init_node('BaxterEnvironmentCollisionValidator')
    env = args.env
    label = args.label
    start = int(time.time())

    label_generator = None
    if label == 'self':
        label_generator = SelfCollisionLabelGenerator(env)
    elif label == 'env':
        label_generator = EnvironmentCollisionLabelGenerator(env_name=env)
    else:
        raise RuntimeError("Unknown label to generate collision data for.")
    file = "./data/generated_data/input/%s/right_7_1000000_with_pos_dout_7_envCollision.csv" % env
    dataset = augment_dataset(file_name=file, env_name=env,
                              label_generator=label_generator)  # generate the new dataset;
    print("Finished generating the entire dataset for environment: %s" % env)
    print("Time elapsed: %s" % (time.time() - start))

if __name__=='__main__':
    parser = argparse.ArgumentParser()

    # dataset configurations
    parser.add_argument('--env')
    parser.add_argument('--label')
    args, _ = parser.parse_known_args()
    main(args)