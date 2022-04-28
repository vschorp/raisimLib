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

# bsub -J "raisim-gym" -n 32 -R "rusage[mem=256]" -W 24:00 python /cluster/home/vschorp/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_ouzel/runner.py