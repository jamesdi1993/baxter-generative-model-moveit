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

from src.sampler.self_collision_free_sampler import VAE
from src.env.space import initialize_space
from src.state_validity_check.state_validity_checker import MoveitStateValidityChecker
from src.utils.utils import JOINT_NAMES

import csv
import numpy as np
import os
import rospy
import torch

BATCH_SIZE = 1000

def writeToFile(filename, data, headers):
    with open(filename, mode='w') as output_file:
        # Quote header such that it is easier for reader to parse data
        writer = csv.writer(output_file, quoting=csv.QUOTE_NONNUMERIC)
        writer.writerow(headers)
        points = []
        lines_read = 0
        batch = 0

        for i in range(data.shape[0]):
            points.append(data[i, :])
            lines_read += 1
            if lines_read % BATCH_SIZE == 0:
                writer.writerows(points)
                points = []
                print("Finish filling data for batch: %s" % (batch))
                batch += 1
        # write the last rows;
        writer.writerows(points)

def main():
    directory_formatter =  './data/latent_space/{}_num_joints/{}_beta'
    file_formatter = 'right_{}_{}.csv'
    rospy.init_node("LatentSpaceSamplesGenerator")
    space = initialize_space()
    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    state_validity_checker = MoveitStateValidityChecker(si)
    # TODO: Implement a state validity checker
    ss.setStateValidityChecker(state_validity_checker)

    d_input = 7
    h_dim1 = 256
    h_dim2 = 100
    d_output = 7
    beta = 0.1

    generated_batch_size = 1000
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    directory = directory_formatter.format(str(d_input), str(beta))
    filename = file_formatter.format(str(d_input), str(generated_batch_size))
    headers = ['latent' + name for name in JOINT_NAMES][0:d_input]
    headers.append("collision_free")

    if not os.path.exists(directory):
        os.makedirs(directory)
        print("Writing to file: %s" % filename)

    # load the generator model;
    model = VAE(d_input, h_dim1, h_dim2, d_output)
    model.load_state_dict(torch.load("./data/model/model.pth"))
    model.eval()

    with torch.no_grad():
        normals = torch.randn(generated_batch_size, d_output).to(device)
        samples = model.decode(normals).double().cpu().numpy()

        collisionFlags = np.zeros((normals.shape[0], 1), dtype=int)

        for i in range(samples.shape[0]):
            sample = samples[i, :]
            state = ob.State(space)
            for j in range(sample.shape[0]):
                state[j] = sample[j]
            isValid = state_validity_checker.isValid(state)  # checking collision status in c-space;
            collisionFlags[i, 0] = isValid
        data = np.append(samples, collisionFlags, axis=1)
        writeToFile(os.path.join(directory,filename), data, headers=headers)

if __name__=="__main__":
    main()











