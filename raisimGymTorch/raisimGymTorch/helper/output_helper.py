import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt
import torch
import os


class EvaluationVisualizer:
    def __init__(self, max_steps, ob_dim, save_path, is_delta=False):
        self.is_delta = is_delta
        self.max_steps = max_steps
        self.ob_dim = ob_dim
        self.save_path = save_path + "/../../../../data/images/"

        self.orient_quat_W_all = np.zeros([max_steps, 4])
        self.body_linear_vel_all = np.zeros([max_steps, 3])
        self.body_angular_vel_all = np.zeros([max_steps, 3])
        self.ref_orient_quat_W_all = np.zeros([max_steps, 4])
        self.error_angle = np.zeros([max_steps, 1])
        if is_delta:
            self.ref_delta_position_offset_W_all = np.zeros([max_steps, 3])
            self.delta_ouzel_position_offet_W_all = np.zeros([max_steps, 3])
            self.delta_joint_angle_all = np.zeros([max_steps, 3])
            self.delta_joint_angular_vel_all = np.zeros([max_steps, 3])
            self.ref_delta_position_abs_W_all = np.zeros([max_steps, 3])
        else:
            self.pos_RC_W_all = np.zeros([max_steps, 3])

        self.action_lin_corr_all = np.zeros([max_steps, 3])
        self.action_orient_corr_all = np.zeros([max_steps, 4])
        if is_delta:
            self.action_delta_angles_all = np.zeros([max_steps, 3])

        self.means = np.zeros(ob_dim)
        self.vars = np.ones(ob_dim)
        self.stds = np.ones(ob_dim)
        self.epsilon = 1e-8
        self.stds = np.ones(ob_dim)

        self.zero_line = np.zeros([max_steps, 1])

    def load_normalization_params(self, path, it_number):
        mean_fname = "mean" + it_number + ".csv"
        var_fname = "var" + it_number + ".csv"
        mean_fpath = os.path.join(path, mean_fname)
        var_fpath = os.path.join(path, var_fname)

        self.means = np.loadtxt(mean_fpath)
        self.vars = np.loadtxt(var_fpath)
        self.vars += np.ones(self.ob_dim) * self.epsilon
        self.stds = np.sqrt(self.vars)

    def parse_obs(self, obs: np.ndarray, step):
        obs_orig = np.multiply(obs, self.stds) + self.means

        if self.is_delta:
            ref_delta_position_offset_W = obs_orig[0][0:3]
            delta_ouzel_position_offet_W = obs_orig[0][3:6]
            orient_mat_W = np.vstack([obs_orig[0][6:9], obs_orig[0][9:12], obs_orig[0][12:15]]).transpose()
            orient_W = Rotation.from_matrix(orient_mat_W)
            body_linear_vel_W = obs_orig[0][15:18]
            body_angular_vel_W = obs_orig[0][18:21]
            ref_orient_mat_W = np.vstack([obs_orig[0][21:24], obs_orig[0][24:27], obs_orig[0][27:30]]).transpose()
            ref_orient_W = Rotation.from_matrix(ref_orient_mat_W)
            delta_joint_angles = obs_orig[0][30:33]
            delta_joint_anglular_vel = obs_orig[0][33:36]
            ref_delta_position_abs_W = obs_orig[0][36:39]
        else:
            pos_RC_W = obs_orig[0][0:3]
            orient_mat_W = np.vstack([obs_orig[0][3:6], obs_orig[0][6:9], obs_orig[0][9:12]]).transpose()
            orient_W = Rotation.from_matrix(orient_mat_W)
            body_linear_vel_W = obs_orig[0][12:15]
            body_angular_vel_W = obs_orig[0][15:18]
            ref_orient_mat_W = np.vstack([obs_orig[0][18:21], obs_orig[0][21:24], obs_orig[0][24:27]]).transpose()
            ref_orient_W = Rotation.from_matrix(ref_orient_mat_W)

        error_rotation = orient_W.inv() * ref_orient_W

        self.orient_quat_W_all[step, :] = orient_W.as_quat()
        self.body_linear_vel_all[step, :] = body_linear_vel_W
        self.body_angular_vel_all[step, :] = body_angular_vel_W
        self.ref_orient_quat_W_all[step, :] = ref_orient_W.as_quat()
        self.error_angle[step, :] = error_rotation.magnitude()
        if self.is_delta:
            self.ref_delta_position_offset_W_all[step, :] = ref_delta_position_offset_W
            self.delta_ouzel_position_offet_W_all[step, :] = delta_ouzel_position_offet_W
            self.delta_joint_angle_all[step, :] = delta_joint_angles
            self.delta_joint_angular_vel_all[step, :] = delta_joint_anglular_vel
            self.ref_delta_position_abs_W_all[step, :] = ref_delta_position_abs_W
        else:
            self.pos_RC_W_all[step, :] = pos_RC_W

    def parse_action(self, action_ll: torch.Tensor, step):
        action_np = action_ll.detach().numpy()[0]
        lin_corr = action_np[0:3]
        orient_vec_1 = action_np[3:6]
        orient_vec_2 = action_np[6:9]
        if self.is_delta:
            delta_angles = action_np[9:12]

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
        if self.is_delta:
            self.action_delta_angles_all[step, :] = delta_angles

    def visualize(self):
        if self.is_delta:
            plt.figure(figsize=[20, 10])
            plt.plot(self.ref_delta_position_offset_W_all[:, 0] + self.ref_delta_position_abs_W_all[:, 0], 'r-', label='delta_position_W_x')
            plt.plot(self.ref_delta_position_offset_W_all[:, 1] + self.ref_delta_position_abs_W_all[:, 1], 'g-', label='delta_position_W_y')
            plt.plot(self.ref_delta_position_offset_W_all[:, 2] + self.ref_delta_position_abs_W_all[:, 2], 'b-', label='delta_position_W_z')
            plt.plot(self.ref_delta_position_abs_W_all[:, 0], 'r*', label='delta_ref_pos_W_x')
            plt.plot(self.ref_delta_position_abs_W_all[:, 1], 'g*', label='delta_ref_pos_W_y')
            plt.plot(self.ref_delta_position_abs_W_all[:, 2], 'b*', label='delta_ref_pos_W_z')
            plt.legend()
            plt.savefig(self.save_path + "pos_delta_abs.png")

        plt.figure(figsize=[20, 10])
        if self.is_delta:
            plt.plot(self.ref_delta_position_offset_W_all[:, 0] + self.ref_delta_position_abs_W_all[:, 0] + self.delta_ouzel_position_offet_W_all[:, 0], 'r-',
                     label='ouzel_position_W_x')
            plt.plot(self.ref_delta_position_offset_W_all[:, 1] + self.ref_delta_position_abs_W_all[:, 1] + self.delta_ouzel_position_offet_W_all[:, 1], 'g-',
                     label='ouzel_position_W_y')
            plt.plot(self.ref_delta_position_offset_W_all[:, 2] + self.ref_delta_position_abs_W_all[:, 2] + self.delta_ouzel_position_offet_W_all[:, 2], 'b-',
                     label='ouzel_position_W_z')
            plt.plot(self.action_lin_corr_all[:, 0] + self.ref_delta_position_abs_W_all[:, 0], 'r*', label='ref_ouzel_position_W_x')
            plt.plot(self.action_lin_corr_all[:, 1] + self.ref_delta_position_abs_W_all[:, 1], 'g*', label='ref_ouzel_position_W_y')
            plt.plot(self.action_lin_corr_all[:, 2] + self.ref_delta_position_abs_W_all[:, 2], 'b*', label='ref_ouzel_position_W_z')
        else:
            plt.figure(figsize=[20, 10])
            plt.plot(self.pos_RC_W_all[:, 0], 'r-', label='pos_RC_W_all_x')
            plt.plot(self.pos_RC_W_all[:, 1], 'g-', label='pos_RC_W_all_y')
            plt.plot(self.pos_RC_W_all[:, 2], 'b-', label='pos_RC_W_all_z')
            plt.plot(self.action_lin_corr_all[:, 0], 'r*', label='ref_corr_pos_W_all_x')
            plt.plot(self.action_lin_corr_all[:, 1], 'g*', label='ref_corr_pos_W_all_y')
            plt.plot(self.action_lin_corr_all[:, 2], 'b*', label='ref_corr_pos_W_all_z')
        plt.legend()
        plt.savefig(self.save_path + "pos_omav_abs.png")

        if self.is_delta:
            plt.figure(figsize=[20, 10])
            plt.plot(self.delta_joint_angle_all[:, 0], 'r-', label='delta_angle_1')
            plt.plot(self.delta_joint_angle_all[:, 1], 'g-', label='delta_angle_2')
            plt.plot(self.delta_joint_angle_all[:, 2], 'b-', label='delta_angle_3')
            plt.plot(self.action_delta_angles_all[:, 0], 'r*', label='ref_delta_angle_1')
            plt.plot(self.action_delta_angles_all[:, 1], 'g*', label='ref_delta_angle_2')
            plt.plot(self.action_delta_angles_all[:, 2], 'b*', label='ref_delta_angle_3')
            plt.legend()
            plt.savefig(self.save_path + "delta_angles.png")

            plt.figure(figsize=[20, 10])
            plt.plot(self.zero_line[:, 0], "k-", label="zero line")
            plt.plot(self.delta_joint_angular_vel_all[:, 0], 'r-', label='delta_angular_vel_1')
            plt.plot(self.delta_joint_angular_vel_all[:, 1], 'g-', label='delta_angular_vel_2')
            plt.plot(self.delta_joint_angular_vel_all[:, 2], 'b-', label='delta_angular_vel_3')
            plt.legend()
            plt.savefig(self.save_path + "delta_angular_vel.png")

        orient_W_all_rot = Rotation.from_quat(self.orient_quat_W_all)
        body_orient_euler_W_all = orient_W_all_rot.as_euler("xyz")
        ref_orient_W_all_rot = Rotation.from_quat(self.ref_orient_quat_W_all)
        ref_orient_euler_W_all = ref_orient_W_all_rot.as_euler("xyz")
        action_orient_corr_all_rot = Rotation.from_quat(self.action_orient_corr_all)
        corr_ref_W_all_rot = ref_orient_W_all_rot * action_orient_corr_all_rot
        corr_ref_W_all_euler = corr_ref_W_all_rot.as_euler("xyz")
        corr_ref_W_all_quat = corr_ref_W_all_rot.as_quat()
        plt.figure(figsize=[20, 10])
        plt.plot(body_orient_euler_W_all[:, 0], 'r-', label='body_orient_euler_W_all_x')
        plt.plot(body_orient_euler_W_all[:, 1], 'g-', label='body_orient_euler_W_all_y')
        plt.plot(body_orient_euler_W_all[:, 2], 'b-', label='body_orient_euler_W_all_z')
        # plt.plot(self.orient_quat_W_all[:, 0], 'r-', label='body_orient_quat_W_all_x')
        # plt.plot(self.orient_quat_W_all[:, 1], 'g-', label='body_orient_quat_W_all_y')
        # plt.plot(self.orient_quat_W_all[:, 2], 'b-', label='body_orient_quat_W_all_z')
        # plt.plot(self.orient_quat_W_all[:, 3], 'm-', label='body_orient_quat_W_all_w')
        plt.plot(ref_orient_euler_W_all[:, 0], 'r--', label='ref_orient_euler_W_all_x')
        plt.plot(ref_orient_euler_W_all[:, 1], 'g--', label='ref_orient_euler_W_all_y')
        plt.plot(ref_orient_euler_W_all[:, 2], 'b--', label='ref_orient_euler_W_all_z')
        # plt.plot(self.ref_orient_quat_W_all[:, 0], 'r--', label='ref_orient_quat_W_all_x')
        # plt.plot(self.ref_orient_quat_W_all[:, 1], 'g--', label='ref_orient_quat_W_all_y')
        # plt.plot(self.ref_orient_quat_W_all[:, 2], 'b--', label='ref_orient_quat_W_all_z')
        # plt.plot(self.ref_orient_quat_W_all[:, 3], 'm--', label='ref_orient_quat_W_all_w')
        plt.plot(corr_ref_W_all_euler[:, 0], 'r*', label='ref_orient_corr_euler_W_all_x')
        plt.plot(corr_ref_W_all_euler[:, 1], 'g*', label='ref_orient_corr_euler_W_all_y')
        plt.plot(corr_ref_W_all_euler[:, 2], 'b*', label='ref_orient_corr_euler_W_all_z')
        # plt.plot(corr_ref_W_all_quat[:, 0], 'r*', label='ref_orient_corr_quat_W_all_r')
        # plt.plot(corr_ref_W_all_quat[:, 1], 'g*', label='ref_orient_corr_quat_W_all_g')
        # plt.plot(corr_ref_W_all_quat[:, 2], 'b*', label='ref_orient_corr_quat_W_all_b')
        # plt.plot(corr_ref_W_all_quat[:, 3], 'm*', label='ref_orient_corr_quat_W_all_w')
        plt.legend()
        plt.savefig(self.save_path + "orient.png")

        plt.figure(figsize=[20, 10])
        plt.plot(self.body_linear_vel_all[:, 0], 'r-', label='body_linear_vel_all_x')
        plt.plot(self.body_linear_vel_all[:, 1], 'g-', label='body_linear_vel_all_y')
        plt.plot(self.body_linear_vel_all[:, 2], 'b-', label='body_linear_vel_all_z')
        plt.plot(np.linalg.norm(self.body_linear_vel_all, axis=1), 'm--', label='body_linear_vel_all_norm')
        plt.legend()
        plt.savefig(self.save_path + "linear_vel.png")
        print(f"finale linear vel {np.linalg.norm(self.body_linear_vel_all, axis=1)[-1]}")

        plt.figure(figsize=[20, 10])
        plt.plot(self.body_angular_vel_all[:, 0], 'r-', label='body_angular_vel_all_x')
        plt.plot(self.body_angular_vel_all[:, 1], 'g-', label='body_angular_vel_all_y')
        plt.plot(self.body_angular_vel_all[:, 2], 'b-', label='body_angular_vel_all_z')
        plt.plot(np.linalg.norm(self.body_angular_vel_all, axis=1), 'm--', label='body_angular_vel_all_norm')
        plt.legend()
        plt.savefig(self.save_path + "angular_vel.png")
        print(f"finale angular vel {np.linalg.norm(self.body_angular_vel_all, axis=1)[-1]}")

        plt.figure(figsize=[20, 10])
        if self.is_delta:
            plt.plot(np.linalg.norm(self.delta_ouzel_position_offet_W_all - self.action_lin_corr_all, axis=1), 'r-',
                     label='ouzel_pos_W_error')
            plt.plot(np.linalg.norm(self.ref_delta_position_offset_W_all, axis=1), 'g-', label='delta_pos_W_error')
            print(f"final error delta {np.linalg.norm(self.ref_delta_position_offset_W_all, axis=1)[-1]}")
            print(f"final error angle {self.error_angle[-1]}")
        else:
            plt.plot(np.linalg.norm(self.pos_RC_W_all, axis=1), 'r-', label='pos_W_error')
        plt.plot(self.error_angle, 'b-', label='angle_error_rad')
        plt.legend()
        plt.savefig(self.save_path + "error_metrics.png")
