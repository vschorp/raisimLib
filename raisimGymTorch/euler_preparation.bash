#!/bin/bash   
#BSUB -J "raisim-gym"           #Name of the job                                             
#BSUB -n 32                     #Requesting 32 core > run $ export OMP_NUM_THREADS=32 
#BSUB -R "rusage[mem=256]"
#BSUB -W 24:00                   #Requesting 24 hours running time

#source terminal
source /cluster/home/vschorp/.bashrc

#Change to the old software stack
source /cluster/apps/local/env2lmod.sh

#Load the needed modules
module load gcc/6.3.0 cmake/3.16.5 eigen boost

#activate conda env
conda activate raisim-gym

#rebuild env
rm -rf /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/build
rm /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/bin/rsg_*
cd /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch
python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/setup.py develop              

#run training script
#python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel/runner.py

# bsub -J "raisim-gym-1" -n 32 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel/scripts/runner.py --config cfg_1.yaml
bsub -J "raisim-gym-delta-1" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py --config cfg_1.yaml
bsub -J "raisim-gym-delta-2" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py --config cfg_2.yaml
bsub -J "raisim-gym-delta-3" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py --config cfg_3.yaml
bsub -J "raisim-gym-delta-4" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py --config cfg_4.yaml
bsub -J "raisim-gym-delta-5" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py --config cfg_5.yaml

# retrain
bsub -J "raisim-gym-delta-1" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py -m retrain --weight /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/data/ouzel_delta_planning/2022-05-26-15-14-21/full_71000.pt --config cfg_1.yaml
bsub -J "raisim-gym-delta-2" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py -m retrain --weight /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/data/ouzel_delta_planning/2022-05-26-15-14-21/full_71000.pt --config cfg_2.yaml
bsub -J "raisim-gym-delta-3" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py -m retrain --weight /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/data/ouzel_delta_planning/2022-05-26-15-14-21/full_71000.pt --config cfg_3.yaml
bsub -J "raisim-gym-delta-4" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py -m retrain --weight /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/data/ouzel_delta_planning/2022-05-26-15-14-21/full_71000.pt --config cfg_4.yaml
bsub -J "raisim-gym-delta-5" -n 64 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel_delta/scripts/runner.py -m retrain --weight /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/data/ouzel_delta_planning/2022-05-26-15-14-21/full_71000.pt --config cfg_5.yaml
