## raisim_env_anymal

### How to use this repo
There is nothing to compile here. This only provides a header file for an environment definition. Read the instruction of raisimGym. 

### Dependencies
- raisimgym

### Run
1. Compile raisimgym: ```python setup develop```
2. run runner.py of the task (for anymal example): ```cd raisimGymTorch/env/envs/rsg_ouzel && python ./runner.py```

### Test policy
1. Compile raisimgym: ```python setup develop```
2. run tester.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_ouzel/tester.py --weight data/roughTerrain/FOLDER_NAME/full_XXX.pt```

### Retrain policy
1. run runner.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_ouzel/runner.py --mode retrain --weight data/roughTerrain/FOLDER_NAME/full_XXX.pt```

### Debugging
1. Compile raisimgym with debug symbols: ```python setup develop --Debug```. This compiles <YOUR_APP_NAME>_debug_app
2. Run it with Valgrind. I strongly recommend using Clion for 

### Envs
- rsg_anymal is the demo environment which is included in the original repo.
- rsg_ouzel is the environment containing the ouzel OMAV.
  - The environment takes the output from the policy which it feeds to the impendance controller. The impedance controller
    then compustes the wrench it wants to apply to the drone. The drone is treated as rigid system which has a wrench applied 
    to it at every timestep. The resulting movement is then simulated by raisim.
  - The agent is only given negative rewards (costs). The first two terms penalise the offset (linear and angular) to the reference pose.
    The last two terms penalise the magnitude of the policy output (action). This action is interpreted as correction terms which are added
    to the actual reference pose inside the impedance controller. 