import casadi as cs
import numpy as np
import logging
from .orca_plus_All import ORCAPlusAll
from .state_plus import FullState, FullyObservableJointState

### This is a copy of NewMPCReal to test human path prediction
### To reduce computational code it is not solving the MPC
### This is just used for getting human path prediction by calling ORCA

class NewMPCReal():
    def __init__(self):
        
        # Environment-related variables
        self.time_step = 1 # Time step for MPC
        self.human_max_speed = 1
        
        # MPC-related variables
        self.horizon = 5# Fixed time horizon
        

    def predict(self, env_state):
        human_states = []
        robot_state = env_state.self_state
        robot_radius = robot_state.radius
        x_inital =(env_state.self_state.px, env_state.self_state.py, env_state.self_state.theta)


        if robot_state.omega is None:
            robot_state.omega = 0

        # Convert robot_state (of type SelfState) to FullState
        robot_full_state = FullState(px=robot_state.px,  py=robot_state.py, vx=robot_state.vx,  vy=robot_state.vy, radius=robot_state.radius, 
                                      gx=robot_state.gx,  gy=robot_state.gy, v_pref=robot_state.v_pref,  theta=robot_state.theta,  omega=robot_state.omega)
        
        for hum in env_state.human_states:               
            gx = 5 # hum.px + hum.vx * 2 
            gy = 5 # hum.py + hum.vy * 2 # need to remove when the goal prediction is fully completed
            hum_state = FullState(px=hum.px, py=hum.py, vx=hum.vx, vy=hum.vy, 
                                  gx=gx, gy=gy, v_pref=self.human_max_speed, 
                                  theta=np.arctan2(hum.vy, hum.vx), radius=hum.radius, omega=None)                    
            human_states.append(hum_state)
        
    

        # Create a FullyObservableJointState with the new robot_full_state
        state = FullyObservableJointState(self_state=robot_full_state, human_states=human_states, static_obs=env_state.static_obs)
            
        # Step 1: Predict future human positions over the time horizon using ORCA
        orca_policy = ORCAPlusAll()
        predicted_human_poses = orca_policy.predictAllForTimeHorizon(state, self.horizon)       
        #logging.info(f"predict {predicted_human_poses}")

        
        num_humans = len(predicted_human_poses[0][0][1:])
        future_human_states = [[[]]]
        if num_humans > 0:
            future_human_states = np.zeros((num_humans, self.horizon,2))



        for i in range(num_humans):
            for t in range(self.horizon):
                future_human_states[i][t][0] = float(predicted_human_poses[0][t][i+1][0])  # X-coordinate
                future_human_states[i][t][1] = float(predicted_human_poses[0][t][i+1][1]) # Y-coordinate


        return future_human_states# Return the optimal control action      


    