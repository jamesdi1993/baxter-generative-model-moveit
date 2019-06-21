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

from torch.nn import functional as F
from torch import nn
from ..utils.utils import JOINT_LIMITS, JOINT_INDEX
import torch

class VAE(nn.Module):

    def __init__(self, d_in, h_dim1, h_dim2, d_out, keep_prob=0):
        super(VAE, self).__init__()
        # The latent variable is defined as z = N(\mu, \sigma).
        self.d_in = d_in

        # Encoder network
        self.fc1 = torch.nn.Linear(d_in, h_dim1)
        self.fc2 = torch.nn.Linear(h_dim1, h_dim2)
        self.mean = torch.nn.Linear(h_dim2, d_out)
        self.logvar = torch.nn.Linear(h_dim2, d_out)

        # Decoder network
        self.fc3 = torch.nn.Linear(d_out, h_dim2)
        self.fc4 = torch.nn.Linear(h_dim2, h_dim1)
        self.fc5 = torch.nn.Linear(h_dim1, d_in)

    # Encode the input into a normal distribution
    def encode(self, x):
        h1 = F.elu(self.fc1(x))
        h2 = F.elu(self.fc2(h1))
        return self.mean(h2), self.logvar(h2)

    # Reparametrize the normal;
    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return eps.mul(std).add_(mu)

    def decode(self, z):
        h2 = F.elu(self.fc3(z))
        h1 = F.elu(self.fc4(h2))
        # No need to normalize when using mse loss;
        # return torch.sigmoid(self.fc5(h1))
        return self.fc5(h1)

    # Forward pass;
    def forward(self, x):
        # Change the array to float first;
        mu, logvar = self.encode(x.view(-1, self.d_in))
        # print("The value of mu and logvars are: %s, %s" % (mu, logvar))
        z = self.reparameterize(mu, logvar)
        return self.decode(z), mu, logvar

def generate_samples(generated_batch_size, d_output, device, model):
    # Generate sampled outputs;
    with torch.no_grad():
        norms = torch.randn(generated_batch_size, d_output).to(device)
        samples = model.decode(norms).double().cpu().numpy()
        return samples

class SelfCollisionFreeStateSampler(ob.StateSampler):

    def __init__(self, space):
        super(SelfCollisionFreeStateSampler, self).__init__(space)
        self.name_ = "self_collision_free_sampler"
        self.d_input = 7
        self.h_dim1 = 256
        self.h_dim2 = 100
        self.d_output = 7  # latent layer;

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = VAE(self.d_input, self.h_dim1, self.h_dim2, self.d_output)
        self.model.load_state_dict(torch.load("./data/model/model.pth"))
        self.model.eval()

        # TODO: Load model here;
    def sampleUniform(self, state):
        sample = generate_samples(1, self.d_output, self.device, self.model)
        sample = sample.astype(float)
        for i in range(7):
            state[i] = sample[0, i]

# return an obstacle-based sampler
def allocSelfCollisionFreeStateSampler(si):
    # we can perform any additional setup / configuration of a sampler here,
    # but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return SelfCollisionFreeStateSampler(si)

if __name__=="__main__":
    dof = 7
    space = ob.RealVectorStateSpace(dof)

    # set the bounds
    bounds = ob.RealVectorBounds(dof)
    for key, value in JOINT_LIMITS.items():
        i = JOINT_INDEX[key]
        bounds.setLow(i, value[0])
        bounds.setHigh(i, value[1])
        print("Setting bound for the %sth joint: %s, bound: %s" % (i, key, value))
    space.setBounds(bounds)

    # ss = og.SimpleSetup(space)
    #
    # si = ss.getSpaceInformation()
    sampler = allocSelfCollisionFreeStateSampler(space)
    state = ob.State(space)
    sampler.sampleUniform(state)
    print("The state is: %s" % state)
