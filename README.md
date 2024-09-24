## **Setup**

In a conda environment with Python 3.8.13,

Install this package using pip:
```bash
pip install -e .
```
Install the required libraries:

```bash
pip install -r requirements.txt
```
Clone and install Python-RVO2 library. Note that a compatible version of Cython 0.29.33 will be installed already in step 2. You should only need to run:

```bash
pip install -e <path-to-Python-RVO2-dir>/Python-RVO2/
```

(Recommended) Install HSL to use advanced solvers in IPOPT, the settings that we use in campc.py make use of a linear solver from HSL, however the code in this repo will work with the default IPOPT settings as well. Instructions: https://github.com/casadi/casadi/wiki/Obtaining-HSL.

Testing Crowd Navigation Algorithms

The following algorithms can be visualized by running the following command in the sicnav/ directory:

python simple_test.py --policy <policy>

--policy campc  :  collision avoidance MPC (SICNav)
--policy NewMPCChanging  :  our proposed MPC (developping)
--policy dwa  :  dynamic window  approch
--policy orca_plus  :  ORCA 


To train and visualize the Reinforcement Learning algorithms, SARL and RGL, please see the RL_nav/ subdirectory.

CrowdSimPlus Simulator

CrowdSimPlus is based on OpenAI gym and is an extension of CrowdSim (Repo, Paper), with the follwing added features:

Static obstacles to allow for more realistic dense scenarios.
Social Forces Model (SFM) policy to simulate human agents, in addition to the original Optimal Reciprocal Collision Avoidance (ORCA) policy. These policies can be found under crowd_sim_plus.envs.policy.
Stable Baselines 3 (SB3) integration for Reinforcement Learning (RL) methods.

Contributors
This repository is primarily developed by Sepehr Samavi (sicnav and crowd_sim_plus) and James R. Han (RL_nav and crowd_sim_plus).
