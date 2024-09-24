# Crowd Navigation Policies

This project provides several crowd navigation policies that can be tested using Python scripts. Below, you’ll find details on how to run the policies, configure the environment, and visualize predictions.

## Running Navigation Policies

To run a navigation policy, use the following command:

```bash
python Test.py --policy POLICY


To run crowd navigation policies


python Test.py --policy POLICY

usable policies:
campc - for SICNav policy
dwa - for dynamic window approach
orca_plus - for orca policy
ORCAPlusAll - for orca policy (with making output.txt file containing future predictions for few time steps ahead using orca_plus. but dont use  that predictions for navigation. I only uses orca_plus based navigation
NewMPCChanging - our policy (under development) uses ORCAPlusAll for future predictions and control the robot using MPC
NewMPC - our policy (under development) uses ORCAPlusAll for future predictions and control the robot using MPC

use env.config file for configure the environment (number of humans, environment size, radii of each human agents etc.)
use Test.py for change environment type (hallway, hallway_bottleneck)

To see the orca predictions (for varification purpose)
run python Test.py --policy ORCAPlusAll (this will create output.txt, make sure to clear or remove output.txt file before run this command)
run Python Plot.py (this will save a video in the predictions folder. Make sure to change values in Plot.py like num of humans, time horizon, plot size according to env.config file mannally)

