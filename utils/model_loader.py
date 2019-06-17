from src.sampler.self_collision_free_sampler import VAE
import torch

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
