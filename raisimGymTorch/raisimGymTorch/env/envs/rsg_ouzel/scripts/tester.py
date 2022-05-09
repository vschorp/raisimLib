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

visualize_simulation = True
# visualize_simulation = False

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

    print("Visualizing and evaluating the policy: ", weight_path)
    loaded_graph = ppo_module.MLP(cfg["architecture"]["policy_net"], activation, ob_dim, act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)["actor_architecture_state_dict"])

    env.load_scaling(weight_dir, int(iteration_number))
    env.turn_on_visualization()

    max_steps = 1000  ## 10 secs

    eval_visualizer = EvaluationVisualizer(max_steps, ob_dim, task_path)
    # eval_visualizer.load_normalization_params(weight_dir, iteration_number)

    for step in range(max_steps):
        if visualize_simulation:
            time.sleep(0.01)
        obs = env.observe(False)
        action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())
        reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
        reward_ll_sum = reward_ll_sum + reward_ll[0]
        eval_visualizer.parse_obs(obs, step)
        eval_visualizer.parse_action(action_ll, step)
        if dones or step == max_steps - 1:
            print("----------------------------------------------------")
            print(
                "{:<40} {:>6}".format(
                    "average ll reward: ", "{:0.10f}".format(reward_ll_sum / (step + 1 - start_step_id))
                )
            )
            print("{:<40} {:>6}".format("time elapsed [sec]: ", "{:6.4f}".format((step + 1 - start_step_id) * 0.01)))
            print("----------------------------------------------------\n")
            start_step_id = step + 1
            reward_ll_sum = 0.0

    env.turn_off_visualization()
    env.reset()
    eval_visualizer.visualize()
    print("Finished at the maximum visualization steps")
