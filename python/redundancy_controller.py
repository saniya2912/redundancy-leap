import pickle
import numpy as np
from main_redundancy import *

import time
# grasp = GraspClass()
transmatrix = TransMatrix()
leap_hand=LeapNode_Taucontrol()


index_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_index.xml'
thumb_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_thumb.xml'

pos_ik_index=OnlyPosIK(index_path)
pos_ik_thumb=OnlyPosIK(thumb_path)

T_indexbase_palm=np.array([[ 0.        ,  1.        ,  0.        ,  0.03811195],
       [ 0.        ,  0.        , -1.        ,  0.060044  ],
       [-1.        ,  0.        ,  0.        , -0.007246  ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
Rpk_index=T_indexbase_palm[:3,:3]

T_thumbbase_palm=np.array([[0, 0, 1, -0.024188],
              [0, 1, 0, 0.03574396],
              [-1, 0, 0, -0.010146],
              [0, 0, 0, 1]])
Rpk_thumb=T_thumbbase_palm[:3,:3]

Rpks=[Rpk_index,Rpk_thumb]
n = 2
palm_wrt_cam = np.array([[-0.01388162,  0.98129904,  0.19198282,  0.03377598],
              [-0.87071609,  0.08253191, -0.48481306,  0.04381788],
              [-0.49159166, -0.17389219,  0.85328728,  0.49460942],
              [ 0.,  0.,  0.,  1.]])





grasp = GraspClass()

def initialize(filename):
    try:
        with open(filename, 'rb') as file:  # Changed the variable name to 'file' to avoid conflicts
            print(f"Type of file: {type(file)}")  # Check if file is properly opened
            print("Attempting to load array from file...")

            array = pickle.load(file)  # Ensure this is using the correct `pickle.load` method
            #print("Loaded array:", array)

            # Compute f(array)
            result = transmatrix.T_obj_palm(np.array(array),palm_wrt_cam)
            #print("Result of computation:", result)
            

    except FileNotFoundError:
        print(f"File '{filename}' not found. Make sure the file exists.")
    except Exception as e:
        print(f"An error occurred: {e}")

    return result
    # array=np.random.rand(4,4)
    # f(np.array(array))

def f(array,Td):
    object_pose_cam=array
    
    b1 = np.array([[0],[-0.027],[0]])
    b2 = np.array([[0],[0.027],[0]])
    bs = [b1, b2]
    obj_pos=transmatrix.compute_obj_pos(object_pose_cam, palm_wrt_cam) #object position with respect to palm camera frame
    print('obj_pos',obj_pos)

    contactpos_1,contactpos_2=transmatrix.compute_contact_locations(object_pose_cam, palm_wrt_cam,bs)
    print('contact1',contactpos_1)
    print('contact2',contactpos_2)
    qs=leap_hand.read_pos()
    qs_real=qs
    temp = qs[0].copy()  # Use a temporary variable to hold the value of Tau[0]
    qs[0] = qs[1]
    qs[1] = temp

    qs1=qs[:4]
    qs2=qs[-4:]

    
    
    # print('obj_pos_mujoco',x_obj_pbm)

    # obj_pos_mujoco=np.array([0,0,-0.1])
    
    # qs1=pos_ik_index.calculate(contactpos_1,'contact_index')
    # qs2=pos_ik_thumb.calculate(contactpos_2,'contact_thumb')
    # qs1_real=qs1
    # qs2_real=qs2
    # qs1_real[0],qs1_real[1]=qs1_real[1],qs1_real[0]
    # qs=np.concatenate([qs1_real,np.zeros(8),qs2_real])
    # leap_pos=LeapNode_Poscontrol()
    # print(qs)
    
    
    # while time.time() - start_time < 4:
    #     leap_pos.set_allegro(qs)

    n = 2
    contactrot_index,contactrot_thumb,r_theta=transmatrix.compute_finger_rotations(object_pose_cam,palm_wrt_cam)
    contact_orientations=[contactrot_index,contactrot_thumb]
    
    #from mujoco xml filif __name__ == "__main__":
    J_index=grasp.J(index_path,'contact_index',qs1)
    J_thumb=grasp.J(thumb_path,'contact_thumb',qs2)
    Js=[J_index,J_thumb]
    
    Rpks = [Rpk_index, Rpk_thumb]
    

    # Compute the grasp matrices
    Jh_ = grasp.Jh_full(n, contact_orientations, Rpks, Js)
    G_ = grasp.G_full(n, contact_orientations, r_theta, bs)

    H=grasp.selection_matrix(n,'SF')

    Jh_leap=H@Jh_
    G_leap_T=H@G_.T
    G_leap=G_leap_T.T

    # Controller parameters
    Kp_d = 0.1*np.eye(6)
    Kp_k = 1

    n0 = np.ones([8,1])
    I = np.eye(8)
    phi_d = np.ones([8,1])

#leap_hand = LeapNode_Taucontrol()

# Main control loop
   

    posrot=PosRot()
    T=transmatrix.T_obj_palm(object_pose_cam,palm_wrt_cam)

    q_final=posrot.q_subs(Td,T)

    # Compute forces
    Fimp = np.dot(np.linalg.pinv(G_leap),np.dot(Kp_d , (q_final.reshape(6,1))))
    Fnull = (I - np.matmul(np.linalg.pinv(G_leap), G_leap)) @ n0
    
    # Compute desired torque
    Tau_dy = Jh_leap.T @ (Fimp + Fnull)
    
    
    
    # Read position from the Leap hand
    #phi = leap_hand.read_pos()
    phi=np.ones([8,1])
    
    # Kinematic control torque
    Tau_kin = Kp_k * (phi_d - phi)  # Corrected this line
    
    # Total torque
    Tau = Tau_dy + Tau_kin*0
    Tau_sim=Tau
    print(Tau)

    temp = Tau[0].copy()  # Use a temporary variable to hold the value of Tau[0]
    Tau[0] = Tau[1]
    Tau[1] = temp

    Tau_real=np.hstack([Tau[:4].flatten(), np.zeros(8), Tau[-4:].flatten()])

    print(Tau_real)

    start_time = time.time()
    while 0 < time.time() - start_time < 10:
        print(f"Elapsed Time: {time.time() - start_time:.2f} seconds")
        leap_hand.set_desired_torque(Tau_real)
        
        actual_position = leap_hand.read_pos()  # Read the actual position
        print("Sent Torque:", Tau_real)
        
        print("Actual Position:", actual_position)

    

def load_and_compute(filename,Td):
    """
    # Load the array from a pickle file and compute f(array).
    # """
    try:
        with open(filename, 'rb') as file:  # Changed the variable name to 'file' to avoid conflicts
            print(f"Type of file: {type(file)}")  # Check if file is properly opened
            print("Attempting to load array from file...")

            array = pickle.load(file)  # Ensure this is using the correct `pickle.load` method
            #print("Loaded array:", array)

            # Compute f(array)
            result = f(np.array(array),Td)
            #print("Result of computation:", result)

    except FileNotFoundError:
        print(f"File '{filename}' not found. Make sure the file exists.")
    except Exception as e:
        print(f"An error occurred: {e}")
    # array=np.random.rand(4,4)
    # f(np.array(array))
    

if __name__ == "__main__":
    # Specify the filename of the saved array
    filename = '/tmp/received_array.pkl'
    # while True:
    #     load_and_compute(filename)
    #     time.sleep(1)
    Td=initialize(filename)
    load_and_compute(filename,Td)
    