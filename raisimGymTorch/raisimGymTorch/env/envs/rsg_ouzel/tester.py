import numpy as np
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.bin import rsg_ouzel
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo.module as ppo_module
import os
import math
import time
import torch
import argparse


# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='/home/vincent/rl_4_aerial_manipulator/catkin_ws/src/raisimLib/raisimGymTorch/data/ouzel_only_planning/2022-03-28-18-42-37/full_165000.pt')
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1

env = VecEnv(rsg_ouzel.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])

# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts

weight_path = args.weight
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'

body_pos_W_all = np.empty([1, 30])
body_orient_mat_W_all = np.empty([1, 30])
body_linear_vel_W_all = np.empty([1, 30])
body_angular_vel_W_all = np.empty([1, 30])
ref_pos_W_all = np.empty([1, 30])
ref_orient_mat_W_all = np.empty([1, 30])

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

    print("Visualizing and evaluating the policy: ", weight_path)
    loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, ob_dim, act_dim)
    loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

    env.load_scaling(weight_dir, int(iteration_number))
    env.turn_on_visualization()

    # max_steps = 1000000
    max_steps = 1000 ## 10 secs

    for step in range(max_steps):
        time.sleep(0.01)
        obs = env.observe(False)
        action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())
        reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
        reward_ll_sum = reward_ll_sum + reward_ll[0]
        if dones or step == max_steps - 1:
            print('----------------------------------------------------')
            print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(reward_ll_sum / (step + 1 - start_step_id))))
            print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * 0.01)))
            print('----------------------------------------------------\n')
            start_step_id = step + 1
            reward_ll_sum = 0.0

    env.turn_off_visualization()
    env.reset()
    print("Finished at the maximum visualization steps")

def parse_obs(obs: np.ndarray, body_pos_W_all, body_orient_mat_W_all, body_linear_vel_W_all, _angular_vel_W_all, pos_W_all, orient_mat_W_all):
    body_pos_W = obs[0][0:3]
    body_orient_mat_W = np.concatenate(obs[0][3:6], obs[0][6:9], obs[0][9:12])
    body_linear_vel_W = obs[0][12:15]
    body_angular_vel_W = obs[0][15:18]
    ref_pos_W = obs[0][18:21]
    ref_orient_mat_W = np.concatenate(obs[0][21:24], obs[0][24:27], obs[0][27:30])

    body_orient_mat_W_all.append(body_orient_mat_W)
    body_linear_vel_W_all.append(body_linear_vel_W)
    body_angular_vel_W_all.append(body_angular_vel_W)
    ref_pos_W_all.append(ref_pos_W)
    ref_orient_mat_W_all.append(ref_orient_mat_W)
