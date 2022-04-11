from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import rsg_ouzel
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo.module as ppo_module
from raisimGymTorch.helper.output_helper import EvaluationVisualizer
import os
import math
import time
import torch
import argparse
import numpy as np


# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='')
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1

env = VecEnv(rsg_ouzel.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper))) # cfg['environment']
# env = VecEnv(rsg_ouzel.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), normalize_ob=False) # cfg['environment']
seed = int(time.time())
print(f"the seed is {seed}")
env.seed(seed)

visualize_simulation = True
# visualize_simulation = False

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts

weight_path = args.weight
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'

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
    average_dones = 0.
    n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
    total_steps = n_steps * 1
    start_step_id = 0

    print("loading model: ", weight_path)
    loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], activation, ob_dim, act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

    example_input = torch.rand(1, 30)
    traced_script_module = torch.jit.trace(loaded_graph.architecture, example_input)
    output_fname = os.path.join(weight_dir, "trained_model.pt")
    traced_script_module.save(output_fname)
    print(f"export model to {output_fname}")
