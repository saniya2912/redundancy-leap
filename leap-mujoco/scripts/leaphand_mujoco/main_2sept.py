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

    def apply_controls_palm_z(self, desired_position):
        #GIVE SMOOTH TRAJECTORY INSTEAD OF DIRECT POSE


        # Calculate control signals based on PID control (if needed)
        current_position = self.d.qpos[-19]
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

        self.d.ctrl[-19] = control_signal
        
        
        self.prev_error_palm = error_palm   

    def apply_controls_palm_y(self,desired_position):
    #GIVE SMOOTH TRAJECTORY INSTEAD OF DIRECT POSE

        # Calculate control signals based on PID control (if needed)
        current_position_y = self.d.qpos[-18]
        # print(current_positions)
        error_palm_y = desired_position - current_position_y
        # self.integral_palm += error_palm
        derivative = error_palm_y - self.prev_error_palm_y
        
        control_signal = (
            self.kPpalm_y * error_palm_y +
            # self.kIpalm * self.integral_palm +
            self.kDpalm_y * derivative
        )
        
        # Limit the current if necessary
        #control_signals = np.clip(control_signals, -self.curr_lim, self.curr_lim)

        self.d.ctrl[-18] = control_signal
        
        
        self.prev_error_palm_y = error_palm_y

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
    
    def Jh(self,n,contact_orientations,Rpks,Js):
        for i in range(n):
            Jh_i=np.matmul(np.matmul(contact_orientations[i].T,Rpks[i]),Js[i])
            self.Jh_blocks.append(Jh_i)
        return block_diag(*self.Jh_blocks)
    
class LeapHandKinematics:
    def __init__(self, urdf_path):
        self.urdf_model = URDF.from_xml_file(urdf_path)
        self.base_link = 'palm_lower'

    def find_joint_for_link(self, child_link_name):
        """Find the joint in the URDF model that connects to the given child link."""
        for joint in self.urdf_model.joints:
            if joint.child == child_link_name:
                return joint
        return None

    def add_joint_to_chain(self, chain, joint):
        if joint.type == 'revolute' or joint.type == 'continuous':
            kdl_joint = kdl.Joint(
                joint.name,
                kdl.Vector(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]),
                kdl.Vector(joint.axis[0], joint.axis[1], joint.axis[2]),
                kdl.Joint.RotAxis
            )
        elif joint.type == 'fixed':
            kdl_joint = kdl.Joint(joint.name, kdl.Joint.Fixed)
        else:
            print(f"Unsupported joint type: {joint.type}")
            return False

        kdl_segment = kdl.Segment(joint.child, kdl_joint, kdl.Frame())
        chain.addSegment(kdl_segment)
        return True

    def create_kdl_chain(self, end_link):
        chain = kdl.Chain()
        current_link = end_link

        while current_link != self.base_link:
            joint = self.find_joint_for_link(current_link)
            if not joint:
                print(f"Joint for link {current_link} not found!")
                return None
            if not self.add_joint_to_chain(chain, joint):
                print(f"Failed to add joint {joint.name} to chain")
                return None
            current_link = joint.parent

        return chain

    def perform_fk(self, end_link, joint_positions):
        chain = self.create_kdl_chain(end_link)
        if not chain or chain.getNrOfSegments() == 0:
            raise RuntimeError(f"Chain for {end_link} could not be created.")

        fk_solver = kdl.ChainFkSolverPos_recursive(chain)
        end_effector_frame = kdl.Frame()
        print(f"Calculating FK for joint positions: {[joint_positions[i] for i in range(joint_positions.rows())]}")
        result = fk_solver.JntToCart(joint_positions, end_effector_frame)

        if result >= 0:
            return end_effector_frame
        else:
            raise RuntimeError("FK solver failed")

    def perform_ik(self, end_link, target_frame):
        chain = self.create_kdl_chain(end_link)
        if not chain or chain.getNrOfSegments() == 0:
            raise RuntimeError(f"Chain for {end_link} could not be created.")

        num_joints = chain.getNrOfJoints()
        joint_positions = kdl.JntArray(num_joints)
        ik_solver = kdl.ChainIkSolverPos_LMA(chain)

        initial_positions = kdl.JntArray(num_joints)  # Start with zero positions
        result = ik_solver.CartToJnt(initial_positions, target_frame, joint_positions)

        if result >= 0:
            return [joint_positions[i] for i in range(num_joints)]
        else:
            raise RuntimeError("IK solver failed")

    def compute_jacobian(self, end_link, joint_positions):
        chain = self.create_kdl_chain(end_link)
        if not chain or chain.getNrOfSegments() == 0:
            raise RuntimeError(f"Chain for {end_link} could not be created.")

        jacobian = kdl.Jacobian(chain.getNrOfJoints())
        jnt_to_jac_solver = kdl.ChainJntToJacSolver(chain)

        result = jnt_to_jac_solver.JntToJac(joint_positions, jacobian)

        if result >= 0:
            return jacobian
        else:
            raise RuntimeError("Jacobian computation failed")