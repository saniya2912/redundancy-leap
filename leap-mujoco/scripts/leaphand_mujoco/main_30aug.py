import numpy as np
import time
import mujoco
import mujoco.viewer
import sys
import mediapy as media
import matplotlib.pyplot as plt
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import numpy as np
from scipy.linalg import block_diag



class LeapNodeMujoco:
    def __init__(self,model_path):    
        self.kP = 15 #25 for 22.8.24
        self.kI = 0
        self.kD = 5
        self.kP_slow =2
        self.kI = 0
        self.kD_slow = 1
        self.kPpalm = 250
        self.kIpalm = 0
        self.kDpalm = 10
        self.kPwrist = 15
        self.kIwrist = 0
        self.kDwrist = 1
        self.curr_lim = 0
        # self.prev_pos = self.pos = self.curr_pos = lhus.LEAPhand_to_LEAPsim(lhus.allegro_to_LEAPhand(np.zeros(16)))
        self.prev_pos = self.pos = self.curr_pos = 0.0
        self.prev_pos_palm=self.pos_palm=self.curr_pos_palm=0
        self.prev_pos_wrist=self.pos_wrist=self.curr_pos_wrist=0
       # self.model_path='/home/iitgn-robotics/Saniya/mujoco-3.1.6/model/leap hand/leaphand.xml'

        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)
        self.cam=mujoco.MjvCamera()

        # self.viewer=mujoco.viewer.launch(self.m, self.d)

         # Initialize Renderer and frames list
        self.renderer = mujoco.Renderer(self.m)
        self.frames = []

        # For PID control
        self.prev_error = np.zeros_like(self.curr_pos)
        self.prev_error_palm = np.zeros_like(self.curr_pos_palm)
        # self.integral_palm=np.zeros_like(self.curr_pos_palm)


    def apply_controls_wrist(self, desired_position):
        #GIVE SMOOTH TRAJECTORY INSTEAD OF DIRECT POSE


        # Calculate control signals based on PID control (if needed)
        current_positions = self.d.qpos[-17]
        # print(current_positions)
        error_wrist = desired_position - current_positions
        # self.integral_wrist += error_wrist
        derivative = error_wrist - self.prev_error_palm
        
        control_signal = (
            self.kPwrist * error_wrist +
            # self.kIwrist * self.integral_wrist +
            self.kDwrist * derivative
        )
        
        # Limit the current if necessary
        #control_signals = np.clip(control_signals, -self.curr_lim, self.curr_lim)

        self.d.ctrl[-17] = control_signal
        
        
        self.prev_error_wrist = error_wrist  

    def apply_controls_palm(self, desired_position):
        #GIVE SMOOTH TRAJECTORY INSTEAD OF DIRECT POSE


        # Calculate control signals based on PID control (if needed)
        current_position = self.d.qpos[-18]
        # print(current_positions)
        error_palm = desired_position - current_position
        # self.integral_palm += error_palm
        derivative = error_palm - self.prev_error_palm
        
        control_signal = (
            self.kPpalm * error_palm +
            # self.kIpalm * self.integral_palm +
            self.kDpalm * derivative
        )
        
        # Limit the current if necessary
        #control_signals = np.clip(control_signals, -self.curr_lim, self.curr_lim)

        self.d.ctrl[-18] = control_signal
        
        
        self.prev_error_palm = error_palm   

    def apply_controls_hand(self, desired_positions):

        # Calculate control signals based on PID control (if needed)
        current_positions = self.d.qpos[-16:]
        # print(current_positions)
        errors = desired_positions - current_positions
        # self.integral += errors
        derivative = errors - self.prev_error
        
        control_signals = (
            self.kP * errors +
            # self.kI * self.integral +
            self.kD * derivative
        )
        
        self.d.ctrl[-16:] = control_signals
        
        self.prev_error = errors    

    def apply_controls_hand_slow(self, desired_positions):

        # Calculate control signals based on PID control (if needed)
        current_positions = self.d.qpos[-16:]
        # print(current_positions)
        errors = desired_positions - current_positions
        # self.integral += errors
        derivative = errors - self.prev_error
        
        control_signals = (
            self.kP_slow * errors +
            # self.kI * self.integral +
            self.kD_slow * derivative
        )
        
        self.d.ctrl[-16:] = control_signals
        
        self.prev_error = errors   


   
    def step_video(self, framerate,camera):
        mujoco.mj_step(self.m, self.d)
        if len(self.frames) < self.d.time * framerate:
                self.renderer.update_scene(self.d,camera)
                pixels = self.renderer.render()
                self.frames.append(pixels)
                
        

      
    def play_video(self,framerate):
        print(f"Total frames captured: {len(self.frames)}")
        if self.frames:
            media.show_video(self.frames, fps=framerate)
        else:
            print("No frames captured.")

class GraspClass:
    def __init__(self):
        self.G_matrices=[]
        self.Jh_blocks=[]
        
    def G_i(self,contact_orientation, r_theta,b):
        matrix1 = contact_orientation
        r_theta_b = np.dot(r_theta,b)  # Matrix multiplication
        
        matrix2 = np.array([np.cross(r_theta_b.flatten(), contact_orientation[:, 0].flatten()),
                            np.cross(r_theta_b.flatten(), contact_orientation[:, 1].flatten()),
                            np.cross(r_theta_b.flatten(), contact_orientation[:, 2].flatten())])
        
        return np.vstack([matrix1, matrix2])


    def G(self,n,contact_orientations, r_theta,bs):
        for i in range(n):
            G_i_matrix = self.G_i(contact_orientations[i],r_theta,bs[i])
            self.G_matrices.append(G_i_matrix)

        # Concatenate all G_i matrices horizontally to form G
        G = np.hstack(self.G_matrices)
        return G
    
    def J(xml_path,site_name):
        model=mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        jacp = np.zeros((3, model.nv))  # translation jacobian
        jacr = np.zeros((3, model.nv)) 

        site_id=model.site(site_name).id
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

        return np.vstack((jacp, jacr))
    
    def Jh(self,n,contact_orientations,Rpks,Js):
        for i in range(n):
            Jh_i=np.matmul(np.matmul(contact_orientations[i].T,Rpks[i]),Js[i])
            self.Jh_blocks.append(Jh_i)
        return block_diag(*self.Jh_blocks)
    