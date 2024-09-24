# Crowd Navigation Policies

This project provides several crowd navigation policies that can be tested using Python scripts. Below, you’ll find details on how to run the policies, configure the environment, and visualize predictions.

## Running Navigation Policies

To run a navigation policy, use the following command:

```bash
python Test.py --policy POLICY
```

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

Available Policies:

    campc: Implements the SICNav policy.
    dwa: Uses the Dynamic Window Approach (DWA) for navigation.
    orca_plus: Uses the ORCA policy for crowd navigation.
    ORCAPlusAll: This policy creates an output.txt file containing ORCA-based future predictions for a few time steps ahead. However, these predictions are not used for navigation, and the ORCA-based navigation is applied instead.
    NewMPCChanging: A policy under development that uses ORCAPlusAll for future predictions and controls the robot using MPC (Model Predictive Control).
    NewMPC: Another development policy that also uses ORCAPlusAll for future predictions and controls the robot using MPC.

Configuration

The environment can be configured using the env.config file, where you can specify:

    Number of humans: Number of human agents present in the simulation.
    Environment size: Dimensions of the navigation environment.
    Human radii: Radii of human agents for collision avoidance calculations.

Changing the Environment Type

You can switch between different environment types in Test.py. Supported environment types include:

    hallway
    hallway_bottleneck

Example command to change the environment type:

bash

python Test.py --policy POLICY --env_type hallway

Visualizing ORCA Predictions

To verify ORCA predictions, follow these steps:

    Generate ORCA Predictions:

    Run this command to create the output.txt file containing future predictions using the ORCAPlusAll policy:

    bash

python Test.py --policy ORCAPlusAll

    Note: Ensure the output.txt file is cleared or deleted before running this command to avoid overwriting issues.

Plot and Save the Predictions:

Run Plot.py to visualize the predictions and save a video in the predictions folder:

bash

    python Plot.py

        Important: Update the values in Plot.py (such as the number of humans, time horizon, plot size) according to the env.config file manually.

By following these steps, you can test different navigation policies, configure the environment, and visualize ORCA-based predictions.


