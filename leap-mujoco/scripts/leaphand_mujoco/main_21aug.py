import numpy as np
import time
import mujoco
import mujoco.viewer
import sys
import mediapy as media
import matplotlib.pyplot as plt


class LeapNodeMujoco:
    def __init__(self,model_path):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        #self.kP=600
        self.kP = 10
        self.kI = 0
        self.kD = 5
        self.kPpalm = 250
        self.kIpalm = 0
        self.kDpalm = 10
        self.kPwrist = 10
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

       # self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] #define motors from somewhere

        # For PID control
        self.prev_error = np.zeros_like(self.curr_pos)
        self.prev_error_palm = np.zeros_like(self.curr_pos_palm)
        # self.integral_palm=np.zeros_like(self.curr_pos_palm)
        

        # Set initial control values
        #self.set_initial_controls()

    def set_initial_controls(self): #needs changing
        # This sets the initial control values directly in MuJoCo
        num_actuators = self.m.nu
        # self.d.ctrl[:num_actuators] = np.ones(num_actuators) * self.kP
        
        # self.d.ctrl[:num_actuators] = 0
        print("control is 0")
        # Initialize control values with Kp, Ki, Kd might be more complex depending on the control strategy


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

    def apply_controls(self, desired_positions):

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
        
        # Limit the current if necessary
        #control_signals = np.clip(control_signals, -self.curr_lim, self.curr_lim)

        self.d.ctrl[-16:] = control_signals
        
        
        self.prev_error = errors    

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.apply_controls(self.curr_pos)

    # #allegro compatibility
    # def set_allegro(self, pose):
    #     pose = lhus.allegro_to_LEAPhand(pose, zeros=False)
    #     self.prev_pos = self.curr_pos
    #     self.curr_pos = np.array(pose)
    #     self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    # #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap

    # Read position
    def read_pos(self):
        return self.d.qpos.tolist()
    
    # Read velocity
    def read_vel(self):
        return self.d.qvel.tolist()
    
   
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