from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, tensorboard_launcher

# this script starts the tensorboard on which the progress of the training can be checked during training.
# run example: $ python tensorboard_launcher.py

tensorboard_launcher('/home/vincent/rl_4_aerial_manipulator/catkin_ws/src/raisimLib/raisimGymTorch/data/ouzel_only_planning/')