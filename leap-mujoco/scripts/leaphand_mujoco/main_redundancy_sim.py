import numpy as np
import time
import mujoco
import mujoco.viewer
import sys
import mediapy as media
import matplotlib.pyplot as plt

from urdf_parser_py.urdf import URDF
import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation as R



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
        self.prev_pos_palm_y = self.pos_palm_y = self.curr_pos_palm_y = 0
        self.prev_error_palm = 0  # Initialize the previous error
        self.prev_error_palm_y = 0
        self.kPpalm_y = 20
        self.kDpalm_y = 4
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


    # def apply_controls_wrist(self, desired_position):
    #     #GIVE SMOOTH TRAJECTORY INSTEAD OF DIRECT POSE


    #     # Calculate control signals based on PID control (if needed)
    #     current_positions = self.d.qpos[-17]
    #     # print(current_positions)
    #     error_wrist = desired_position - current_positions
    #     # self.integral_wrist += error_wrist
    #     derivative = error_wrist - self.prev_error_palm
        
    #     control_signal = (
    #         self.kPwrist * error_wrist +
    #         # self.kIwrist * self.integral_wrist +
    #         self.kDwrist * derivative
    #     )
        
    #     # Limit the current if necessary
    #     #control_signals = np.clip(control_signals, -self.curr_lim, self.curr_lim)

    #     self.d.ctrl[-17] = control_signal
        
        
    #     self.prev_error_wrist = error_wrist  

    # def apply_controls_palm_z(self, desired_position):
    #     #GIVE SMOOTH TRAJECTORY INSTEAD OF DIRECT POSE


    #     # Calculate control signals based on PID control (if needed)
    #     current_position = self.d.qpos[-19]
    #     # print(current_positions)
    #     error_palm = desired_position - current_position
    #     # self.integral_palm += error_palm
    #     derivative = error_palm - self.prev_error_palm
        
    #     control_signal = (
    #         self.kPpalm * error_palm +
    #         # self.kIpalm * self.integral_palm +
    #         self.kDpalm * derivative
    #     )
        
    #     # Limit the current if necessary
    #     #control_signals = np.clip(control_signals, -self.curr_lim, self.curr_lim)

    #     self.d.ctrl[-19] = control_signal
        
        
    #     self.prev_error_palm = error_palm   

    # def apply_controls_palm_y(self,desired_position):
    # #GIVE SMOOTH TRAJECTORY INSTEAD OF DIRECT POSE

    #     # Calculate control signals based on PID control (if needed)
    #     current_position_y = self.d.qpos[-18]
    #     # print(current_positions)
    #     error_palm_y = desired_position - current_position_y
    #     # self.integral_palm += error_palm
    #     derivative = error_palm_y - self.prev_error_palm_y
        
    #     control_signal = (
    #         self.kPpalm_y * error_palm_y +
    #         # self.kIpalm * self.integral_palm +
    #         self.kDpalm_y * derivative
    #     )
        
    #     # Limit the current if necessary
    #     #control_signals = np.clip(control_signals, -self.curr_lim, self.curr_lim)

    #     self.d.ctrl[-18] = control_signal
        
        
    #     self.prev_error_palm_y = error_palm_y

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

    def apply_controls_index(self, desired_positions):

        # Calculate control signals based on PID control (if needed)
        current_positions = self.d.qpos[-16:-12]
        # print(current_positions)
        errors = desired_positions - current_positions
        # self.integral += errors
        derivative = errors - self.prev_error
        
        control_signals = (
            self.kP * errors +
            # self.kI * self.integral +
            self.kD * derivative
        )
        
        self.d.ctrl[-16:-12] = control_signals
        
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

class GraspClass2:
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
    
    # def J_p(self,model,data,site_name):
    #     mujoco.mj_forward(model, data)
    #     jacp = np.zeros((3, model.nv))  # translation jacobian
    #     jacr = np.zeros((3, model.nv)) 

    #     site_id=model.site(site_name).id
    #     mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

    #     return jacp
    
    def J(self,xml_path,site_name,qs):
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        data.qpos=qs
        mujoco.mj_forward(model, data)
        jacp = np.zeros((3, model.nv))  # translation jacobian
        jacr = np.zeros((3, model.nv)) 

        site_id=model.site(site_name).id
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

        # return np.vstack((jacp,jacr))
        return jacp

    def Jh(self,n,contact_orientations,Rpks,Js):
        for i in range(n):
            Jh_i=np.matmul(np.matmul(contact_orientations[i].T,Rpks[i]),Js[i])
            self.Jh_blocks.append(Jh_i)
        return block_diag(*self.Jh_blocks)    
    

class GraspClass:
    def __init__(self):
        self.G_matrices=[]
        self.Jh_blocks=[]
    
        
    def G_i(self,contact_orientation, r_theta,b):
        zero=np.zeros((3,1))
        matrix1 = np.hstack((contact_orientation, zero,zero,zero)) 
        r_theta_b = np.dot(r_theta,b)  # Matrix multiplication
        
        matrix2 = np.array([np.cross(r_theta_b.flatten(), contact_orientation[:, 0].flatten()),
                        np.cross(r_theta_b.flatten(), contact_orientation[:, 1].flatten()),
                        np.cross(r_theta_b.flatten(), contact_orientation[:, 2].flatten())])

    # Add one more row to matrix2 to match the shape of matrix1
        matrix2 = np.hstack([matrix2, contact_orientation[:, 0].reshape(3,1),contact_orientation[:, 1].reshape(3,1),contact_orientation[:, 2].reshape(3,1)]) 
        
        return np.vstack([matrix1, matrix2])

    def G_full(self,n,contact_orientations, r_theta,bs):
        for i in range(n):
            G_i_matrix = self.G_i(contact_orientations[i],r_theta,bs[i])
            self.G_matrices.append(G_i_matrix)

        # Concatenate all G_i matrices horizontally to form G
        G = np.hstack(self.G_matrices)
        return G
    
    # def J_p(self,model,data,site_name):
    #     mujoco.mj_forward(model, data)
    #     jacp = np.zeros((3, model.nv))  # translation jacobian
    #     jacr = np.zeros((3, model.nv)) 

    #     site_id=model.site(site_name).id
    #     mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

    #     return jacp
    
    def J(self,xml_path,site_name,qs):
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        data.qpos=qs
        mujoco.mj_forward(model, data)
        jacp = np.zeros((3, model.nv))  # translation jacobian
        jacr = np.zeros((3, model.nv)) 

        site_id=model.site(site_name).id
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

        return np.vstack((jacp,jacr))
        

    def Jh_full(self,n,contact_orientations,Rpks,Js):
        for i in range(n):
            wi_T=block_diag(contact_orientations[i].T, contact_orientations[i].T)
            Rpki=block_diag(Rpks[i],Rpks[i])
            Jh_i=np.matmul(np.matmul(wi_T,Rpki),Js[i])
            self.Jh_blocks.append(Jh_i)
        return block_diag(*self.Jh_blocks)
    
    def selection_matrix(self,n,type):
        if type=='HF':
            Hi=np.array([[1,0,0,0,0,0],
                        [0,1,0,0,0,0],
                        [0,0,1,0,0,0]])

        elif type=='SF':
            Hi=np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0]])
            
        diagonal_stacked = block_diag(*[Hi for _ in range(n)])

        return diagonal_stacked

# class PosRot:
#     def quaternion_multiply(self, q1, q2):
#         # Extract quaternion components from Rotation objects
#         q1 = q1.as_quat()  # [x1, y1, z1, w1]
#         q2 = q2.as_quat()  # [x2, y2, z2, w2]

#         x1, y1, z1, w1 = q1
#         x2, y2, z2, w2 = q2

#         # Perform quaternion multiplication
#         w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
#         x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
#         y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
#         z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

#         # Return the result as a new Rotation object
#         return R.from_quat([x, y, z, w])

#     def quaternion_inverse(self, q):
#         # Extract quaternion components as an array [x, y, z, w]
#         quat = q.as_quat()  # Extract [x, y, z, w] from Rotation object
#         x, y, z, w = quat  # Unpack the quaternion

#         # Return the inverse of the quaternion
#         return R.from_quat([x, -y, -z, -w])

#     def q_subs(self, model, data, body_name):
#         # Get the body ID from the model
#         body_id = model.body(body_name).id

#         # Access body position and quaternion from the data object
#         pos = data.body_xpos[body_id]  # Dynamic position of the body [x, y, z]
#         quat = data.body_xquat[body_id]  # Quaternion of the body [x, y, z, w]

#         # Create a Rotation object from the quaternion
#         quat_rot = R.from_quat(quat)

#         # Convert the quaternion to Euler angles
#         euler = quat_rot.as_euler('xyz', degrees=False)

#         # Stack position and Euler angles into one array
#         q_final = np.hstack((pos, euler))

#         return q_final



class PosRot:
    def quaternion_multiply(self, q1, q2):
    # Extract quaternion components from Rotation objects
        q1 = q1.as_quat()  # [x1, y1, z1, w1]
        q2 = q2.as_quat()  # [x2, y2, z2, w2]
        
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        # Perform quaternion multiplication
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        # Return the result as a new Rotation object
        return R.from_quat([x, y, z, w])


    def quaternion_inverse(self, q):
        # Extract quaternion components as an array [x, y, z, w]
        quat = q.as_quat()  # Extract [x, y, z, w] from Rotation object
        x, y, z, w = quat  # Unpack the quaternion
        # Return the inverse of the quaternion
        return R.from_quat([x, -y, -z, -w])


    # def q_subs(self, model1, model2, data1, data2, body_name):
    #     # Get positions
    #     pos1 = data1.body(model1.body(body_name).id).xpos.reshape(3)
    #     pos2 = data2.body(model2.body(body_name).id).xpos.reshape(3)
    #     pos = pos1 - pos2
        
    #     # Get quaternions from MuJoCo data (assuming they are in [x, y, z, w] format)
    #     quat1 = data1.body(model1.body(body_name).id).xquat
    #     quat2 = data2.body(model2.body(body_name).id).xquat
        
    #     # Create Rotation objects from quaternions
    #     quat1_rot = R.from_quat(quat1)  # Convert to Rotation object
    #     quat2_rot = R.from_quat(quat2)  # Convert to Rotation object
        
    #     # Invert quat2 and multiply with quat1
    #     quat2_rot_inv = self.quaternion_inverse(quat2_rot)  # Invert quaternion
    #     relative_rot = self.quaternion_multiply(quat1_rot, quat2_rot_inv)  # Multiply
        
    #     # Convert the relative rotation to Euler angles
    #     euler = relative_rot.as_euler('xyz', degrees=False)
        
    #     # Stack position and Euler angles into one array
    #     q_final = np.hstack((pos, euler))
        
    #     return q_final

    def q_subs(self, initial_pos, initial_quat, model, data, body_name):
        # Get the current position and quaternion of the body
        body_id = model.body(body_name).id
        current_pos = data.xpos[body_id]  # Current position [x, y, z]
        current_quat = data.xquat[body_id]  # Current quaternion [x, y, z, w]

        # Compute the difference in positions
        pos_error = current_pos - initial_pos

        # Create Rotation objects from quaternions
        initial_rot = R.from_quat(initial_quat)  # Convert initial quaternion
        current_rot = R.from_quat(current_quat)  # Convert current quaternion

        # Compute the relative rotation
        relative_rot = self.quaternion_multiply(current_rot, self.quaternion_inverse(initial_rot))

        # Convert the relative rotation to Euler angles
        euler_error = relative_rot.as_euler('xyz', degrees=False)

        # Stack position and Euler angle errors into one array
        q_final = np.hstack((pos_error, euler_error))

        return q_final
        
import numpy as np
import mujoco
import mujoco.viewer as viewer
import mediapy as media



class GradientDescentIK:
    def __init__(self,xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.jacp = np.zeros((3, self.model.nv))  # translation jacobian
        self.jacr = np.zeros((3, self.model.nv)) 
        self.step_size = 0.5
        self.tol = 0.01
        self.alpha = 0.5
        self.init_q = [0.0, 0.0, 0.0, 0.0]  
    
    def check_joint_limits(self, q):
        """Check if the joints are under or above their limits."""
        for i in range(len(q)):
            q[i] = max(self.model.jnt_range[i][0], min(q[i], self.model.jnt_range[i][1]))


    def calculate(self, goal_pos, goal_rot, site_name):
        self.data.qpos = self.init_q
        mujoco.mj_forward(self.model, self.data)

        site_id= self.model.site(site_name).id
        
        # Current pose and orientation
        current_pos = self.data.site(site_id).xpos
        current_rot = self.data.site(site_id).xmat.reshape(3, 3)

        # Position and orientation error
        pos_error = np.subtract(goal_pos, current_pos)
        rot_error = 0.5 * (np.cross(current_rot[:, 0], goal_rot[:, 0]) +
                           np.cross(current_rot[:, 1], goal_rot[:, 1]) +
                           np.cross(current_rot[:, 2], goal_rot[:, 2]))

        # Combine position and orientation errors
        error = np.concatenate([pos_error, rot_error])

        max_iterations = 100000
        iteration = 0

        while np.linalg.norm(error) >= self.tol and iteration < max_iterations:
            # Calculate Jacobian for position and orientation
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, site_id)
            full_jacobian = np.vstack((self.jacp, self.jacr))
            
            # Calculate gradient
            grad = self.alpha * full_jacobian.T @ error
            
            # Compute next step
            self.data.qpos += self.step_size * grad
            
            # Check joint limits
            self.check_joint_limits(self.data.qpos)
            
            # Compute forward kinematics
            mujoco.mj_forward(self.model, self.data)
            
            # Update position and orientation error
            current_pos = self.data.site(site_id).xpos
            current_rot = self.data.site(site_id).xmat.reshape(3, 3)
            pos_error = np.subtract(goal_pos, current_pos)
            rot_error = 0.5 * (np.cross(current_rot[:, 0], goal_rot[:, 0]) +
                               np.cross(current_rot[:, 1], goal_rot[:, 1]) +
                               np.cross(current_rot[:, 2], goal_rot[:, 2]))
            error = np.concatenate([pos_error, rot_error])

            iteration += 1

        if iteration >= max_iterations:
            print("Warning: Maximum iterations reached. The solution may not have converged.")
        
        result = self.data.qpos.copy()
        return result
    
class OnlyPosIK:
    def __init__(self,xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.jacp = np.zeros((3, self.model.nv))  # translation jacobian
        self.jacr = np.zeros((3, self.model.nv)) 
        self.step_size = 0.5
        self.tol = 0.01
        self.alpha = 0.5
        self.init_q = [0.0, 0.0, 0.0, 0.0]  
    
    def check_joint_limits(self, q):
        """Check if the joints is under or above its limits"""
        for i in range(len(q)):
            q[i] = max(self.model.jnt_range[i][0], min(q[i], self.model.jnt_range[i][1]))

    #Gradient Descent pseudocode implementation
    def calculate(self, goal, site_name):
        site_id=self.model.site(site_name).id
        self.data.qpos = self.init_q
        mujoco.mj_forward(self.model, self.data)
        current_pose = self.data.site(site_id).xpos
        error = np.subtract(goal, current_pose)

        max_iterations = 100000
        iteration = 0

        while (np.linalg.norm(error) >= self.tol) and iteration < max_iterations:
            #calculate jacobian
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr,site_id)
            #calculate gradient
            grad = self.alpha * self.jacp.T @ error
            #compute next step
            self.data.qpos += self.step_size * grad
            #check joint limits
            self.check_joint_limits(self.data.qpos)
            #compute forward kinematics
            mujoco.mj_forward(self.model, self.data) 
            #calculate new error
            error = np.subtract(goal, self.data.site(site_id).xpos)

            iteration += 1

        if iteration >= max_iterations:
            print("Warning: Maximum iterations reached. The solution may not have converged.")
        
        result = self.data.qpos.copy()
        return result





            
        
