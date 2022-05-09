from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import rsg_ouzel
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo.module as ppo_module
import os
import math
import time
import torch
import argparse
import numpy as np


# configuration
parser = argparse.ArgumentParser()
parser.add_argument("-w", "--weight", help="trained weight path", type=str, default="")
parser.add_argument("-c", "--config", help="config file name", type=str, default="cfg.yaml")
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
home_path = task_path + "/../../../../.."

# config
config_fpath = os.path.join(task_path, "config", args.config)
print(f"loading config file {config_fpath}")
cfg = YAML().load(open(config_fpath, "r"))

# create environment from the configuration file
cfg["environment"]["num_envs"] = 1

env = VecEnv(rsg_ouzel.RaisimGymEnv(home_path + "/rsc", dump(cfg["environment"], Dumper=RoundTripDumper)))
seed = int(time.time())
print(f"the seed is {seed}")
env.seed(seed)

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts

weight_path = args.weight
iteration_number = weight_path.rsplit("/", 1)[1].split("_", 1)[1].rsplit(".", 1)[0]
weight_dir = weight_path.rsplit("/", 1)[0] + "/"

activation = None
if cfg["activation"] == "tanh":
    activation = torch.nn.Tanh
else:
    print("Error ! No valid activation given in cfg file")

if weight_path == "":
    print("Can't find trained weight, please provide a trained weight with --weight switch\n")
else:
    print("Loaded weight from {}\n".format(weight_path))
    start = time.time()
    env.reset()
    reward_ll_sum = 0
    done_sum = 0
    average_dones = 0.0
    n_steps = math.floor(cfg["environment"]["max_time"] / cfg["environment"]["control_dt"])
    total_steps = n_steps * 1
    start_step_id = 0

    print("loading model: ", weight_path)
    loaded_graph = ppo_module.MLP(cfg["architecture"]["policy_net"], activation, ob_dim, act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)["actor_architecture_state_dict"])

    example_input = torch.rand(1, ob_dim)
    traced_script_module = torch.jit.trace(loaded_graph.architecture, example_input)
    output_fname = os.path.join(weight_dir, "trained_model.pt")
    traced_script_module.save(output_fname)
    print(f"export model to {output_fname}")

    mean_path = os.path.join(weight_dir, f"mean{iteration_number}.csv")
    var_path = os.path.join(weight_dir, f"var{iteration_number}.csv")
    print(f"loading mean from {mean_path}")
    means = np.loadtxt(mean_path)
    vars = np.loadtxt(var_path)
    mean_save_fname = os.path.join(weight_dir, "trained_mean.csv")
    var_save_fname = os.path.join(weight_dir, "trained_var.csv")
    np.savetxt(mean_save_fname, means)
    np.savetxt(var_save_fname, vars)
    print(f"saved means to {mean_save_fname}")
