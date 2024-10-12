import numpy as np
import logging

from .dynamic_window_approach import dwa_control

class DynamicWindowApproach():
    """
    Template to implement a teleoperation.
    This placeholder is a copy of crowd_sim_plus.envs.policy.linear
    """

    def __init__(self):
        self.name = "DWA"
        self.trainable = False
        self.kinematics = 'unicycle'
        self.multiagent_training = True
        # self.safety_space = 0
        # self.neighbor_dist = 10
        # self.max_neighbors = 10
        self.time_horizon = 2.5
        self.radius = 0.3
        self.max_speed = 0.8
        self.sim_config = None
        self.prev_theta = None

    def configure(self, config):
        pass

    def reset_dwa(self):
        self.prev_theta = None

    def predict(self, state):
        """
        Create a PythonRobotics DWA simulation at each time step and run one step

        # Function structure based on CrowdSim ORCA

        :param state:
        :return:
        """
        self_state = state.self_state


        v = np.sqrt(self_state.vx**2 + self_state.vy**2)
        w = self_state.omega
        if w is None:
            if self.prev_theta is None:
                self.prev_theta = self_state.theta
            w = (self_state.theta - self.prev_theta) / self.time_step
        PRdwa_state = [self_state.px, self_state.py, self_state.theta, v, w]


        u = dwa_control(PRdwa_state, self.sim_config, self_state.goal_position, self.sim_config.ob, self.sim_config.line_obs)
        action = (u[0], u[1])
        logging.debug(str(action))
        self.prev_theta = self_state.theta
        
        #logging.info('[NisalaTestRot] ActionRot {:}'.format(action))

        return action
