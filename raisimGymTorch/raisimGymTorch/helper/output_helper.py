import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt
import torch
import os

class EvaluationVisualizer:
    def __init__(self, max_steps, ob_dim, save_path):
        self.max_steps = max_steps
        self.ob_dim = ob_dim
        self.save_path = save_path + "/../../../../data/images/"

        self.body_pos_W_all = np.zeros([max_steps, 3])
        self.body_orient_quat_W_all = np.zeros([max_steps, 4])
        self.body_linear_vel_W_all = np.zeros([max_steps, 3])
        self.body_angular_vel_W_all = np.zeros([max_steps, 3])
        self.ref_pos_W_all = np.zeros([max_steps, 3])
        self.ref_orient_quat_W_all = np.zeros([max_steps, 4])

        self.action_lin_corr_all = np.zeros([max_steps, 3])
        self.action_orient_corr_all = np.zeros([max_steps, 4])

        self.means = np.zeros(ob_dim)
        self.vars = np.ones(ob_dim)
        self.epsilon = 1e-8

    def load_normalization_params(self, path, it_number):
        mean_fname = "mean" + it_number + ".csv"
        var_fname = "var" + it_number + ".csv"
        mean_fpath = os.path.join(path, mean_fname)
        var_fpath = os.path.join(path, var_fname)

        self.means = np.loadtxt(mean_fpath)
        self.vars = np.loadtxt(var_fpath)
        self.vars += np.ones(self.ob_dim) * self.epsilon

    def parse_obs(self, obs: np.ndarray, step):
        obs_orig = np.multiply(obs, self.vars) + self.means

        body_pos_W = obs_orig[0][0:3]
        body_orient_mat_W = np.vstack([obs_orig[0][3:6], obs_orig[0][6:9], obs_orig[0][9:12]]).transpose()
        body_orient_W = Rotation.from_matrix(body_orient_mat_W)
        body_linear_vel_W = obs_orig[0][12:15]
        body_angular_vel_W = obs_orig[0][15:18]
        ref_pos_W = obs_orig[0][18:21]
        ref_orient_mat_W = np.vstack([obs_orig[0][21:24], obs_orig[0][24:27], obs_orig[0][27:30]]).transpose()
        ref_orient_W = Rotation.from_matrix(ref_orient_mat_W)

        self.body_pos_W_all[step, :] = body_pos_W
        self.body_orient_quat_W_all[step, :] = body_orient_W.as_quat()
        self.body_linear_vel_W_all[step, :] = body_linear_vel_W
        self.body_angular_vel_W_all[step, :] = body_angular_vel_W
        self.ref_pos_W_all[step, :] = ref_pos_W
        self.ref_orient_quat_W_all[step, :] = ref_orient_W.as_quat()

    def parse_action(self, action_ll: torch.Tensor, step):
        action_np = action_ll.detach().numpy()[0]
        lin_corr = action_np[0:3]
        orient_vec_1 = action_np[3:6]
        orient_vec_2 = action_np[6:9]

        orient_vec_1_norm = np.linalg.norm(orient_vec_1)
        orient_vec_2_norm = np.linalg.norm(orient_vec_2)

        rot_mat = np.identity(3)
        if orient_vec_1_norm > 0 and orient_vec_2_norm > 0:
            e1 = orient_vec_1 / orient_vec_1_norm
            u2 = orient_vec_2 - np.dot(e1, orient_vec_2) * e1
            e2 = u2 / np.linalg.norm(u2)
            e3 = np.cross(e1, e2)
            rot_mat = np.vstack([e1, e2, e3]).transpose()

        action_orient_corr_mat = Rotation.from_matrix(rot_mat)

        self.action_lin_corr_all[step, :] = lin_corr
        self.action_orient_corr_all[step, :] = action_orient_corr_mat.as_quat()

    def visualize(self):
        plt.figure(figsize=[20, 10])
        plt.plot(self.body_pos_W_all[:, 0], 'r-', label='body_pos_W_all_x')
        plt.plot(self.body_pos_W_all[:, 1], 'g-',label='body_pos_W_all_y')
        plt.plot(self.body_pos_W_all[:, 2], 'b-', label='body_pos_W_all_z')
        plt.plot(self.ref_pos_W_all[:, 0], 'r--', label='ref_pos_W_all_x')
        plt.plot(self.ref_pos_W_all[:, 1], 'g--', label='ref_pos_W_all_y')
        plt.plot(self.ref_pos_W_all[:, 2], 'b--', label='ref_pos_W_all_z')
        plt.plot(self.ref_pos_W_all[:, 0] + self.action_lin_corr_all[:, 0], 'r*', label='ref_corr_pos_W_all_x')
        plt.plot(self.ref_pos_W_all[:, 1] + self.action_lin_corr_all[:, 1], 'g*', label='ref_corr_pos_W_all_y')
        plt.plot(self.ref_pos_W_all[:, 2] + self.action_lin_corr_all[:, 2], 'b*', label='ref_corr_pos_W_all_z')
        plt.legend()
        plt.savefig(self.save_path + "pos.png")

        body_orient_W_all_rot = Rotation.from_quat(self.body_orient_quat_W_all)
        body_orient_euler_W_all = body_orient_W_all_rot.as_euler("xyz")
        ref_orient_W_all_rot = Rotation.from_quat(self.ref_orient_quat_W_all)
        ref_orient_euler_W_all = ref_orient_W_all_rot.as_euler("xyz")
        action_orient_corr_all_rot = Rotation.from_quat(self.action_orient_corr_all)
        corr_ref_W_all_rot = ref_orient_W_all_rot * action_orient_corr_all_rot
        corr_ref_W_all_euler = corr_ref_W_all_rot.as_euler("xyz")
        plt.figure(figsize=[20, 10])
        plt.plot(body_orient_euler_W_all[:, 0], 'r-', label='body_orient_euler_W_all_x')
        plt.plot(body_orient_euler_W_all[:, 1], 'g-', label='body_orient_euler_W_all_y')
        plt.plot(body_orient_euler_W_all[:, 2], 'b-', label='body_orient_euler_W_all_z')
        # plt.plot(self.body_orient_quat_W_all[:, 3], 'm-', label='body_orient_quat_W_all_w')
        plt.plot(ref_orient_euler_W_all[:, 0], 'r--', label='ref_orient_euler_W_all_x')
        plt.plot(ref_orient_euler_W_all[:, 1], 'g--', label='ref_orient_euler_W_all_y')
        plt.plot(ref_orient_euler_W_all[:, 2], 'b--', label='ref_orient_euler_W_all_z')
        # plt.plot(self.ref_orient_quat_W_all[:, 3], 'm--', label='ref_orient_quat_W_all_w')
        plt.plot(corr_ref_W_all_euler[:, 0], 'r*', label='ref_orient_corr_euler_W_all_x')
        plt.plot(corr_ref_W_all_euler[:, 1], 'g*', label='ref_orient_corr_euler_W_all_y')
        plt.plot(corr_ref_W_all_euler[:, 2], 'b*', label='ref_orient_corr_euler_W_all_z')
        # plt.plot(corr_ref_W_all_quat[:, 3], 'm*', label='ref_orient_corr_quat_W_all_w')
        plt.legend()
        plt.savefig(self.save_path + "orient.png")


        plt.figure(figsize=[20, 10])
        plt.plot(self.body_linear_vel_W_all[:, 0], 'r-', label='body_linear_vel_W_all_x')
        plt.plot(self.body_linear_vel_W_all[:, 1], 'g-', label='body_linear_vel_W_all_y')
        plt.plot(self.body_linear_vel_W_all[:, 2], 'b-', label='body_linear_vel_W_all_z')
        plt.plot(np.linalg.norm(self.body_linear_vel_W_all, axis=1), 'm--', label='body_linear_vel_W_all_norm')
        plt.legend()
        plt.savefig(self.save_path + "linear_vel.png")

        plt.figure(figsize=[20, 10])
        plt.plot(self.body_angular_vel_W_all[:, 0], 'r-', label='body_angular_vel_W_all_x')
        plt.plot(self.body_angular_vel_W_all[:, 1], 'g-', label='body_angular_vel_W_all_y')
        plt.plot(self.body_angular_vel_W_all[:, 2], 'b-', label='body_angular_vel_W_all_z')
        plt.plot(np.linalg.norm(self.body_angular_vel_W_all, axis=1), 'm--', label='body_angular_vel_W_all_norm')
        plt.legend()
        plt.savefig(self.save_path + "angular_vel.png")

        plt.figure(figsize=[20, 10])
        plt.plot(np.linalg.norm(self.body_pos_W_all - self.ref_pos_W_all, axis=1), 'r-', label='pos_W_error')
        #TODO: fix angle error plot

        body_orient_W_all = Rotation.from_quat(self.body_orient_quat_W_all)
        ref_orient_W_all = Rotation.from_quat(self.ref_orient_quat_W_all)
        diff_orient_W_all = body_orient_W_all.inv() * ref_orient_W_all
        diff_angles_W_all = diff_orient_W_all.magnitude()
        diff_angles_W_all_scaled = np.where(np.abs(diff_angles_W_all) < np.abs(diff_angles_W_all - np.pi), diff_angles_W_all, diff_angles_W_all - np.pi)
        plt.plot(diff_angles_W_all_scaled, 'b-', label='angle_error_rad')
        # plt.plot(diff_angles_W_all_scaled / np.pi * 180.0, 'b-', label='angle_error_deg')

        plt.legend()
        plt.savefig(self.save_path + "error_metrics.png")
