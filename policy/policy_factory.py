from crowd_sim_plus.envs.policy.policy_factory import policy_factory
from sicnav.policy.dwa import DynamicWindowApproach
from sicnav.policy.campc import CollisionAvoidMPC
from sicnav.policy.NewMPC import NewMPC
from sicnav.policy.NewMPCChanging import NewMPCChanging

policy_factory['dwa'] = DynamicWindowApproach
policy_factory['campc'] = CollisionAvoidMPC
policy_factory['NewMPC']= NewMPC
policy_factory['NewMPCChanging']= NewMPCChanging

