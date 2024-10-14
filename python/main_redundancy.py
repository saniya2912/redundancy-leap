import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
#######################################################
"""This can control and query the LEAP Hand

I recommend you only query when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
#http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

#LEAP hand conventions:
#180 is flat out for the index, middle, ring, fingers, and positive is closing more and more.

"""
########################################################
class LeapNode_Poscontrol:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 250
        self.kI = 0
        self.kD = 25
        self.kP_slow = 300
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception as e:
            print("[DEBUG]", e)
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()

        self.dxl_client.set_torque_enabled(self.motors, False)
        ADDR_SET_MODE = 11
        LEN_SET_MODE = 1
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 3, ADDR_SET_MODE, LEN_SET_MODE)
        self.dxl_client.set_torque_enabled(self.motors, True)

        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        #self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    
    def read_pos_leap(self):
        pos=self.dxl_client.read_pos()-(np.ones(16)*3.14)
        return pos
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    
    def cubic_trajectory(self,q0, v0, q1, v1, t0, t1, current_time):
        # Ensure the current time is within the valid time range
        current_time = np.clip(current_time, t0, t1)

        # Define the matrix M for scalar time t0 and t1 (applies to all elements)
        M = np.array([
            [1, t0, t0**2, t0**3],
            [0, 1, 2*t0, 3*t0**2],
            [1, t1, t1**2, t1**3],
            [0, 1, 2*t1, 3*t1**2]
        ])

        # Stack the q0, v0, q1, v1 values into a matrix (each as a 16-element array)
        b = np.vstack([q0, v0, q1, v1])

        # Solve for the coefficients a for each set of q0, v0, q1, v1
        a = np.linalg.inv(M).dot(b)

        # Compute position (qd), velocity (vd), and acceleration (ad) for each element
        qd = a[0] + a[1]*current_time + a[2]*current_time**2 + a[3]*current_time**3
        vd = a[1] + 2*a[2]*current_time + 3*a[3]*current_time**2
        ad = 2*a[2] + 6*a[3]*current_time

        return qd
    

class LeapNode_Taucontrol():
    def __init__(self):
    # List of motor IDs
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        
        try:
            # Try connecting to /dev/ttyUSB0
            self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception as e:
            print("[DEBUG]", e)
            # Try connecting to /dev/ttyUSB1 if /dev/ttyUSB0 fails
            try:
                self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                # Try connecting to /dev/ttyUSB2 if /dev/ttyUSB1 fails
                self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()

        self.dxl_client.set_torque_enabled(self.motors, False)
        # Set the control mode to Torque Control Mode
        # Address 11 typically corresponds to setting the operating mode, and 0 is Torque Control Mode
        ADDR_SET_MODE = 11
        LEN_SET_MODE = 1
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))* 0, ADDR_SET_MODE, LEN_SET_MODE)
        self.dxl_client.set_torque_enabled(self.motors, True)

        # Set the current limit for Torque Control (Goal Current)
        # Address 102 might correspond to Goal Current, and 2 bytes is the length
        ADDR_GOAL_CURRENT = 102
        LEN_GOAL_CURRENT = 2
        self.curr_lim = 350  # Adjust the current limit as needed (this is just an example)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

    def set_desired_torque(self, desired_torque):
    # Convert desired torque to the corresponding current (depends on the motor's torque constant)
    # For example, assume a torque constant where 1 unit of torque corresponds to 1 unit of current
        

        # Ensure all elements in the desired_torque are scalars
        # For instance, you can flatten the list if necessary
        desired_torque_flat = [float(torque) for torque in desired_torque]  # Convert all to floats

        # Convert to NumPy array and calculate the current
        desired_current = np.array([torque / 0.51 for torque in desired_torque_flat])
        # Adjust this based on your motor's torque constant

        # Address for the Goal Current (or Torque) register
        ADDR_GOAL_CURRENT = 102
        LEN_GOAL_CURRENT = 2  # Length is usually 2 bytes

        # Write the desired current (which corresponds to the desired torque) to all motors
        self.dxl_client.sync_write(self.motors, desired_current*1000, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

    def read_pos(self):
        return self.dxl_client.read_pos()
    
    def read_pos_leap(self):
        pos=self.dxl_client.read_pos()-(np.ones(16)*3.14)
        return pos
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()

import numpy as np
import mujoco
import mujoco.viewer as viewer

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

import numpy as np
from scipy.linalg import block_diag
import mujoco

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

class TransMatrix:
    def rotation_x(self,theta):
        """
        Returns the rotation matrix for a rotation about the X-axis by an angle theta (in degrees).
        """
        radians = np.radians(theta)
        cos_theta = np.cos(radians)
        sin_theta = np.sin(radians)
        
        R_x = np.array([[1, 0, 0],
                        [0, cos_theta, -sin_theta],
                        [0, sin_theta, cos_theta]])
        
        return R_x

    # Define rotation matrix for rotation around the Y-axis
    def rotation_y(self,theta):
        """
        Returns the rotation matrix for a rotation about the Y-axis by an angle theta (in degrees).
        """
        radians = np.radians(theta)
        cos_theta = np.cos(radians)
        sin_theta = np.sin(radians)
        
        R_y = np.array([[cos_theta, 0, sin_theta],
                        [0, 1, 0],
                        [-sin_theta, 0, cos_theta]])
        
        return R_y

    # Define rotation matrix for rotation around the Z-axis
    def rotation_z(self,theta):
        """
        Returns the rotation matrix for a rotation about the Z-axis by an angle theta (in degrees).
        """
        radians = np.radians(theta)
        cos_theta = np.cos(radians)
        sin_theta = np.sin(radians)
        
        R_z = np.array([[cos_theta, -sin_theta, 0],
                        [sin_theta, cos_theta, 0],
                        [0, 0, 1]])
        
        return R_z


    # Function to compute the left and right finger rotations using the dynamic rotation matrices
    def compute_finger_rotations(self,object_pose_cam, palm_to_cam):
    
        

        # Extract rotation matrices from the transformation matrices
        R_object_cam = object_pose_cam[:3, :3]  # Object pose rotation in the camera frame
        R_palm_cam = palm_to_cam[:3, :3]       # Palm to camera rotation matrix
        R_cam_palm = R_palm_cam.T               # Camera to palm rotation matrix

        R_object_palm = R_object_cam @ R_cam_palm
        R_left=np.array([[1, 0, 0],
                            [0, 0, -1],
                            [0, 1, 0]])
        R_right=np.array([[1, 0, 0],
                            [0, 0, 1],
                            [0, -1, 0]])

        # Compute left finger's rotation matrix: T^B_L = R_object_cam * R_cam_palm * R_y(-90°)
        left_finger_rotation =R_left@ R_object_palm


        # Compute right finger's rotation matrix: T^B_R = R_object_cam * R_cam_palm * R_z(90°) * R_y(90°)
        right_finger_rotation = R_right@ R_object_palm

        return right_finger_rotation,left_finger_rotation, R_object_palm
    
    def compute_obj_pos(self,object_pose_cam, palm_wrt_cam):
        #R_y_180=self.rotation_y(180)

         # Extract rotation matrices from the transformation matrices
        R_object_cam = object_pose_cam[:3, :3]  # Object pose rotation in the camera frame
        R_palm_cam = palm_wrt_cam[:3, :3]       # Palm to camera rotation matrix
        R_cam_palm = R_palm_cam.T               # Camera to palm rotation matrix
        x_obj_cam=object_pose_cam[:3, 3]
        x_palm_cam=palm_wrt_cam[:3, 3]


        obj_pos_palm=R_cam_palm@(x_obj_cam-x_palm_cam)
        return obj_pos_palm

    def compute_contact_locations(self,object_pose_cam, palm_wrt_cam,bs):
        
        obj_pos_palm=self.compute_obj_pos(object_pose_cam, palm_wrt_cam)

        # Extract rotation matrices from the transformation matrices
        R_object_cam = object_pose_cam[:3, :3]  # Object pose rotation in the camera frame
        R_palm_cam = palm_wrt_cam[:3, :3]       # Palm to camera rotation matrix
        R_cam_palm = R_palm_cam.T               # Camera to palm rotation matrix

        R_object_palm = R_object_cam @ R_cam_palm


        contact1_pos=(obj_pos_palm.reshape(3,1)+np.dot(R_object_palm,bs[0])).reshape(3)
        contact2_pos=(obj_pos_palm.reshape(3,1)+np.dot(R_object_palm,bs[1])).reshape(3)

        return contact1_pos, contact2_pos
        

class convert_to_mujoco:
    def __init__(self,palm_wrt_cam):
        # self.T_indexbase_palm=np.array([[0.        , 0.        , 1.        , 0.01309895],
        #                                 [1.        , 0.        , 0.        , -0.0027],
        #                                 [0.        , 1.        , 0.        , 0.016012  ],
        #                                 [0.        , 0.        , 0.        , 1.        ]])
        # self.T_thumbbase_palm=np.array([[0.        , 0.        , 1.        , -0.0493],
        #                                 [0.        , 1.        , 0.        , -0.0027],
        #                                 [-1.        , 0.        , 0.        , 0.0131  ],
        #                                 [0.        , 0.        , 0.        , 1.        ]])
        
        # self.T_preal_pbm=np.array([[0, 0, -1, -0.0200952], 
        #                           [-1, 0, 0, 0.0257578],
        #                           [0, 1, 0, -0.0347224],
        #                           [0, 0, 0, 1]])

        # self.T_pbm_preal=np.array([[0.        , -1.        , 0        , 0.0257578],    
        #                                 [0        , 0.        , 1.        , 0.0347224],
        #                                 [-1.        , 0.        , 0.        , -0.0200952  ],
        #                                 [0.        , 0.        , 0.        , 1.        ]])
    #     self.T_preal_pbm=np.array([[0, -1, 0, -0.0200952], 
    #                               [0, 0, 1, 0.0257578],
    #                               [-1, 0, 0, -0.0347224],
    #                               [0, 0, 0, 1]])

    #     self.T_pbm_preal=np.array([[-0.       , -0.       , -1.       , -0.0347224],
    #    [-1.       , -0.       , -0.       , -0.0200952],
    #    [ 0.       ,  1.       ,  0.       , -0.0257578],
    #    [ 0.       ,  0.       ,  0.       ,  1.       ]])

        self.palm_wrt_cam=palm_wrt_cam
        

    def rotation_x(self,theta):
        """
        Returns the rotation matrix for a rotation about the X-axis by an angle theta (in degrees).
        """
        radians = np.radians(theta)
        cos_theta = np.cos(radians)
        sin_theta = np.sin(radians)
        
        R_x = np.array([[1, 0, 0],
                        [0, cos_theta, -sin_theta],
                        [0, sin_theta, cos_theta]])
        
        return R_x

    # Define rotation matrix for rotation around the Y-axis
    def rotation_y(self,theta):
        """
        Returns the rotation matrix for a rotation about the Y-axis by an angle theta (in degrees).
        """
        radians = np.radians(theta)
        cos_theta = np.cos(radians)
        sin_theta = np.sin(radians)
        
        R_y = np.array([[cos_theta, 0, sin_theta],
                        [0, 1, 0],
                        [-sin_theta, 0, cos_theta]])
        
        return R_y

    # Define rotation matrix for rotation around the Z-axis
    def rotation_z(self,theta):
        """
        Returns the rotation matrix for a rotation about the Z-axis by an angle theta (in degrees).
        """
        radians = np.radians(theta)
        cos_theta = np.cos(radians)
        sin_theta = np.sin(radians)
        
        R_z = np.array([[cos_theta, -sin_theta, 0],
                        [sin_theta, cos_theta, 0],
                        [0, 0, 1]])
        
        return R_z
    
    # def palm_rotated(self,palm_frame):
    #     palm_rotated=self.rotation_y(180)@palm_frame
    #     return palm_rotated

    # def pos_mujoco(self,pos):
    #     pos_mujoco=np.dot(self.rotation_y(180),pos)
    #     return pos_mujoco

    # def pos_mujocopalmbody(self, pos): #converts object_pos-palmreal in camera frame input is obj_pos wrt ca
    # # Convert the 3D pos vector to homogeneous coordinates by adding a 1
    #     pos_homogeneous = np.append(pos, 1)
        
    #     # Perform the matrix multiplication
    #     pos_palmbody = np.dot(self.T_pbm_preal, pos_homogeneous)
    #     pos_palmbody_cam=np.dot(self.palm_wrt_cam[:3,:3],pos_palmbody[:3])
        
    #     # Return the first three elements (x, y, z) of the result
    #     return pos_palmbody_cam

    # def obj_pbm_from_preal(self,obj_wrt_preal):
    #     v2=obj_wrt_preal-self.T_pbm_preal[:3,3]
    #     return np.dot(self.T_preal_pbm[:3,:3],v2)
    
    # def x_pbm_preal_c(self): #in camera frame
    #     x_pbm_preal=self.T_pbm_preal[:3,3]
    #     return np.dot(self.palm_wrt_cam[:3,:3],x_pbm_preal)

    # def pos_index_base_mujoco(self,pos):
    #     obj_palm=pos
    #     index_base_palm=self.T_indexbase_palm[:3,3]
    #     return obj_palm-index_base_palm

