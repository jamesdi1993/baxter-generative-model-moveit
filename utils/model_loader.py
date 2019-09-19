from baxter_interfaces.sampler.collision_free_sampler import VAE
import torch

# TODO: Include model configurations into the model file, including d_input, h_dim1, h_dim2, d_output
def load_model(model_path):
    d_input = 7
    h_dim1 = 256
    h_dim2 = 100
    d_output = 7  # latent layer;

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = VAE(d_input, h_dim1, h_dim2, d_output)
    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    model.eval()
    return model, device, d_output
