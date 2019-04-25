import numpy as np

def sample_loc(num_samples, name):
    joint = {}
    # ref hardware specs for baxter joint limits
    joint[name+'_s0'] = float(np.random.uniform(-1.7016, 1.7016, num_samples)) # s0
    joint[name+'_s1'] = float(np.random.uniform(-2.147, 1.047, num_samples))  # s1
    joint[name+'_e0'] = float(np.random.uniform(-3.0541, 3.0541, num_samples)) # e0
    joint[name+'_e1'] = float(np.random.uniform(-0.05, 2.618, num_samples)) # e1
    joint[name+'_w0'] = float(np.random.uniform(-3.059, 3.059, num_samples)) # w0
    joint[name+'_w1'] = float(np.random.uniform(-1.5707, 2.094, num_samples)) # w1
    joint[name+'_w2'] = float(np.random.uniform(-3.059, 3.059, num_samples)) # w2
    return joint
