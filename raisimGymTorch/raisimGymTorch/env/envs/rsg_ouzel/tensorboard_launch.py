import os

from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, tensorboard_launcher

# task_name = "ouzel_only_planning"
# task_path = os.path.dirname(os.path.realpath(__file__))
# home_path = task_path + "/../../../../.."
#
# saver = ConfigurationSaver(log_dir=home_path + "/raisimGymTorch/data/"+task_name,
#                            save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])

# tensorboard_launcher(saver.data_dir+"/..")
# tensorboard_launcher('/home/vincent/rl_4_aerial_manipulator/catkin_ws/src/raisimLib/raisimGymTorch/data/ouzel_only_planning/')
tensorboard_launcher('/home/vincent/ouzel_only_planning')