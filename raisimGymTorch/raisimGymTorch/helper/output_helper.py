import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt
import torch

def parse_obs(obs: np.ndarray, step, body_pos_W_all, body_orient_quat_W_all, body_linear_vel_W_all, body_angular_vel_W_all, ref_pos_W_all, ref_orient_quat_W_all):
    body_pos_W = obs[0][0:3]
    body_orient_mat_W = np.vstack([obs[0][3:6], obs[0][6:9], obs[0][9:12]]).transpose()
    body_orient_W = Rotation.from_matrix(body_orient_mat_W)
    body_linear_vel_W = obs[0][12:15]
    body_angular_vel_W = obs[0][15:18]
    ref_pos_W = obs[0][18:21]
    ref_orient_mat_W = np.vstack([obs[0][21:24], obs[0][24:27], obs[0][27:30]]).transpose()
    ref_orient_W = Rotation.from_matrix(ref_orient_mat_W)

    body_pos_W_all[step, :] = body_pos_W
    body_orient_quat_W_all[step, :] = body_orient_W.as_quat()
    body_linear_vel_W_all[step, :] = body_linear_vel_W
    body_angular_vel_W_all[step, :] = body_angular_vel_W
    ref_pos_W_all[step, :] = ref_pos_W
    ref_orient_quat_W_all[step, :] = ref_orient_W.as_quat()


def parse_action(action_ll: torch.Tensor, step, action_lin_corr_all, action_orient_corr_all):
    action_np = action_ll.detach().numpy()[0]
    lin_corr = action_np[0:3]
    orient_vec_1 = action_np[3:6]
    orient_vec_2 = action_np[6:9]

    orient_vec_1_norm = np.linalg.norm(orient_vec_1)
    orient_vec_2_norm = np.linalg.norm(orient_vec_2)

    rot_mat = np.identity(3)
    if (orient_vec_1_norm > 0 and orient_vec_2_norm > 0):
        e1 = orient_vec_1 / orient_vec_1_norm
        u2 = orient_vec_2 - np.dot(e1, orient_vec_2) * e1
        e2 = u2 / np.linalg.norm(u2)
        e3 = np.cross(e1, e2)
        rot_mat = np.vstack([e1, e2, e3]).transpose()

    action_orient_corr_mat = Rotation.from_matrix(rot_mat)

    action_lin_corr_all[step, :] = lin_corr
    action_orient_corr_all[step, :] = action_orient_corr_mat.as_quat()


def visualize(task_path, body_pos_W_all, body_orient_quat_W_all, body_linear_vel_W_all, body_angular_vel_W_all, ref_pos_W_all, ref_orient_quat_W_all, action_lin_corr_all, action_orient_corr_all):
    save_path = task_path + "/../../../../data/images/"
    plt.figure(figsize=[20, 10])
    plt.plot(body_pos_W_all[:, 0], 'r-', label='body_pos_W_all_x')
    plt.plot(body_pos_W_all[:, 1], 'g-',label='body_pos_W_all_y')
    plt.plot(body_pos_W_all[:, 2], 'b-', label='body_pos_W_all_z')
    plt.plot(ref_pos_W_all[:, 0], 'r--', label='ref_pos_W_all_x')
    plt.plot(ref_pos_W_all[:, 1], 'g--', label='ref_pos_W_all_y')
    plt.plot(ref_pos_W_all[:, 2], 'b--', label='ref_pos_W_all_z')
    plt.plot(action_lin_corr_all[:, 0] + ref_pos_W_all[:, 0], 'r*', label='ref_corr_pos_W_all_x')
    plt.plot(action_lin_corr_all[:, 1] + ref_pos_W_all[:, 1], 'g*', label='ref_corr_pos_W_all_y')
    plt.plot(action_lin_corr_all[:, 2] + ref_pos_W_all[:, 2], 'b*', label='ref_corr_pos_W_all_z')
    plt.legend()
    plt.savefig(save_path + "pos.png")

    plt.figure(figsize=[20, 10])
    plt.plot(body_orient_quat_W_all[:, 0], 'r-', label='body_orient_quat_W_all_x')
    plt.plot(body_orient_quat_W_all[:, 1], 'g-', label='body_orient_quat_W_all_y')
    plt.plot(body_orient_quat_W_all[:, 2], 'b-', label='body_orient_quat_W_all_z')
    plt.plot(body_orient_quat_W_all[:, 3], 'm-', label='body_orient_quat_W_all_w')
    plt.plot(ref_orient_quat_W_all[:, 0], 'r--', label='ref_orient_quat_W_all_x')
    plt.plot(ref_orient_quat_W_all[:, 1], 'g--', label='ref_orient_quat_W_all_y')
    plt.plot(ref_orient_quat_W_all[:, 2], 'b--', label='ref_orient_quat_W_all_z')
    plt.plot(ref_orient_quat_W_all[:, 3], 'm--', label='ref_orient_quat_W_all_w')
    plt.plot(action_orient_corr_all[:, 0] + ref_orient_quat_W_all[:, 0], 'r*', label='ref_orient_corr_quat_W_all_x')
    plt.plot(action_orient_corr_all[:, 1] + ref_orient_quat_W_all[:, 1], 'g*', label='ref_orient_corr_quat_W_all_y')
    plt.plot(action_orient_corr_all[:, 2] + ref_orient_quat_W_all[:, 2], 'b*', label='ref_orient_corr_quat_W_all_z')
    plt.plot(action_orient_corr_all[:, 3] + ref_orient_quat_W_all[:, 3], 'm*', label='ref_orient_corr_quat_W_all_w')
    plt.legend()
    plt.savefig(save_path + "orient.png")

    plt.figure(figsize=[20, 10])
    plt.plot(body_linear_vel_W_all[:, 0], 'r-', label='body_linear_vel_W_all_x')
    plt.plot(body_linear_vel_W_all[:, 1], 'g-', label='body_linear_vel_W_all_y')
    plt.plot(body_linear_vel_W_all[:, 2], 'b-', label='body_linear_vel_W_all_z')
    plt.plot(np.linalg.norm(body_linear_vel_W_all, axis=1), 'm--', label='body_linear_vel_W_all_norm')
    plt.legend()
    plt.savefig(save_path + "linear_vel.png")

    plt.figure(figsize=[20, 10])
    plt.plot(body_angular_vel_W_all[:, 0], 'r-', label='body_angular_vel_W_all_x')
    plt.plot(body_angular_vel_W_all[:, 1], 'g-', label='body_angular_vel_W_all_y')
    plt.plot(body_angular_vel_W_all[:, 2], 'b-', label='body_angular_vel_W_all_z')
    plt.plot(np.linalg.norm(body_angular_vel_W_all, axis=1), 'm--', label='body_angular_vel_W_all_norm')
    plt.legend()
    plt.savefig(save_path + "angular_vel.png")

    plt.figure(figsize=[20, 10])
    plt.plot(np.linalg.norm(body_pos_W_all - ref_pos_W_all, axis=1), 'r-', label='pos_W_error')
    #TODO: fix angle error plot

    body_orient_W_all = Rotation.from_quat(body_orient_quat_W_all)
    ref_orient_W_all = Rotation.from_quat(ref_orient_quat_W_all)
    diff_orient_W_all = body_orient_W_all.inv() * ref_orient_W_all
    diff_angles_W_all = diff_orient_W_all.magnitude()
    diff_angles_W_all_scaled = np.where(np.abs(diff_angles_W_all) < np.abs(diff_angles_W_all - np.pi), diff_angles_W_all, diff_angles_W_all - np.pi)
    plt.plot(diff_angles_W_all_scaled / np.pi * 180.0, 'b-', label='angle_error_deg')

    plt.legend()
    plt.savefig(save_path + "error_metrics.png")
