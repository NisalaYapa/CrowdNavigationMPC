To run crowd navigation policies


python Test.py --policy POLICY

usable policies:
campc - for SICNav policy
dwa - for dynamic window approach
orca_plus - for orca policy
ORCAPlusAll - for orca policy (with making output.txt file containing future predictions for few time steps ahead using orca_plus. but dont use  that predictions for navigation. I only uses orca_plus based navigation
NewMPCChanging - our policy (under development) uses ORCAPlusAll for future predictions and control the robot using MPC
NewMPC - our policy (under development) uses ORCAPlusAll for future predictions and control the robot using MPC
