import pickle
import numpy as np
from main_redundancy import *
import pandas as pd

import time
# grasp = GraspClass()
transmatrix = TransMatrix()
# leap_hand=LeapNode_Taucontrol()
leap_hand=LeapNode_Poscontrol()

def save_to_csv(r_theta, G, Jh, phi, xobj):
    data = {
        'r_theta': [r_theta.tolist()],  # Store r_theta as a list
        'G': [G.tolist()],  # Store G matrix as a list
        'Jh': [Jh.tolist()],  # Store Jh matrix as a list
        'phi': [phi.tolist()],  # Store phi as a list
        'xobj': [xobj.tolist()]  # Store xobj as a list
    }
    
    combined_df = pd.DataFrame(data)

    # Save the combined DataFrame to a single CSV file
    combined_df.to_csv('pos5.csv', index=False)


index_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_index.xml'
thumb_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_thumb.xml'
index_path_J="/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/index_new.xml"
thumb_path_J="/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/thumb_new.xml"

pos_ik_index=OnlyPosIK(index_path)
pos_ik_thumb=OnlyPosIK(thumb_path)

# T_indexbase_palm=np.array([[ 0.        ,  1.        ,  0.        ,  0.03811195],
#        [ 0.        ,  0.        , -1.        ,  0.060044  ],
#        [-1.        ,  0.        ,  0.        , -0.007246  ],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]])
# Rpk_index=T_indexbase_palm[:3,:3]
Rpk_index_J=np.array([[ 1.,  0., 0.],
                    [0.,  -1.,  0.],         
                    [ 0.,  0.,  -1.]])
Rpk_index=np.eye(3)


# T_thumbbase_palm=np.array([[0, 0, 1, -0.024188],
#               [0, 1, 0, 0.03574396],
#               [-1, 0, 0, -0.010146],
#               [0, 0, 0, 1]])
# Rpk_thumb=T_thumbbase_palm[:3,:3]
Rpk_thumb_J=np.array([[ 1.,  0., 0.],
                    [0.,  0.,  -1.],         
                    [ 0.,  1.,  0.]])
Rpk_thumb=np.eye(3)

Rpks=[Rpk_index,Rpk_thumb]
Rpks_J=[Rpk_index_J,Rpk_thumb_J]
n = 2

palm_wrt_cam = np.array([[-0.01388162,  0.98129904,  0.19198282,  0.03377598],
              [-0.87071609,  0.08253191, -0.48481306,  0.04381788],
              [-0.49159166, -0.17389219,  0.85328728,  0.49460942],
              [ 0.,  0.,  0.,  1.]])





grasp = GraspClass()
grasp2=GraspClass2()

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
    # print('obj_pos',obj_pos)

    contactpos_1,contactpos_2=transmatrix.compute_contact_locations(object_pose_cam, palm_wrt_cam,bs)
    # print('contact1',contactpos_1)
    # print('contact2',contactpos_2)
    qs=leap_hand.read_pos_leap()
    print('qs',qs)
    qs_real=qs
    # print('qs_real',qs_real)

    temp = qs[0].copy()  # Use a temporary variable to hold the value of Tau[0]
    qs[0] = qs[1]
    qs[1] = temp

    print('qschanged',qs)

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
    J_index=grasp2.J(index_path,'contact_index',qs1)
    J_thumb=grasp2.J(thumb_path,'contact_thumb',qs2)

    J_index_J=grasp2.J(index_path_J,'contact_index',qs1)
    J_thumb_J=grasp2.J(thumb_path_J,'contact_thumb',qs2)

    Js=[J_index,J_thumb]
    Js_J=[J_index_J,J_thumb_J]
    
    
    # print('Rpks.shape')
    Jh_leap=grasp2.Jh(n, contact_orientations, Rpks, Js)
    Jh_leap_J=grasp2.Jh(n, contact_orientations, Rpks_J, Js_J)
    # print('Jh_leap',Jh_leap)
    # print('Jh_leap_J',Jh_leap_J.shape)
    # print('Jh',Jh_leap.shape)
    G_leap=grasp2.G(n, contact_orientations, r_theta, bs)
    # print('G_leap',G_leap)

#     # F=np.array([0,0,1,0,0,1]).reshape(6,1)
#     # w=np.dot(G_leap,F)
#     # print('w',w)
#     # J_index_full=grasp.J(index_path_J,'contact_index',qs1)
#     # J_thumb_full=grasp.J(thumb_path_J,'contact_thumb',qs2)
#     # Js_full=[J_index_full,J_thumb_full]

#     # # Compute the grasp matrices
#     # Jh_ = grasp.Jh_full(n, contact_orientations, Rpks, Js_full)
#     # G_ = grasp.G_full(n, contact_orientations, r_theta, bs)

#     # H=grasp.selection_matrix(n,'HF')

#     # print('Jh_full',Jh_.shape)

#     # Jh_leap_full=H@Jh_
#     # print('jh_full',Jh_leap_full)
#     # G_leap_T=H@G_.T
#     # G_leap_full=G_leap_T.T
#     # print('G_full',G_leap_full)

#     # Controller parameters
#     Kp_d = 0.1*np.eye(6)
#     Kp_k = 1

#     n0 = 100*np.ones([6,1])
#     I = np.eye(6)
#     phi_d = np.ones([8,1])

# #leap_hand = LeapNode_Taucontrol()

# # Main control loop
#     posrot=PosRot()
#     T=transmatrix.T_obj_palm(object_pose_cam,palm_wrt_cam)

#     q_final=posrot.q_subs(Td,T)

    # Compute forces
    # Fimp = np.dot(np.linalg.pinv(G_leap),np.dot(Kp_d , (q_final.reshape(6,1))))
    # Fnull = (I - np.matmul(np.linalg.pinv(G_leap), G_leap)) @ n0
    
    # # Compute desired torque
    # Tau_dy = Jh_leap.T @ (Fimp*0 + Fnull)
    # print('F_null',Fnull)
    # w=np.dot(G_leap,Fnull)
    # print('w',w)

#     rank = np.linalg.matrix_rank(G_leap)

# # Print the rank
#     print(f"Rank of the matrix: {rank}")

#     condition_number = np.linalg.cond(G_leap)

#     print("Condition Number:", condition_number)

#     U, s, Vt = np.linalg.svd(G_leap)
    
#     # print(s)

# # s is returned as a 1D array, so we can convert it to a diagonal matrix

#     # Replace the minimum singular value with 0
#     s_min_index = np.argmin(s)  # Find the index of the minimum singular value
#     s[s_min_index] = 0  # Set the minimum singular value to 0
#     # s_sorted_indices = np.argsort(s)  # Get indices of sorted singular values
#     # s[s_sorted_indices[:3]] = 0

#     # Reconstruct Sigma matrix with the modified singular values
#     Sigma = np.zeros((U.shape[0], Vt.shape[0]))
#     np.fill_diagonal(Sigma, s)

#     # print('sigma_new',Sigma)

#     # Reconstruct the matrix G_leap
#     G_leap_reconstructed = np.dot(U, np.dot(Sigma, Vt))

#     # print('G_new',G_leap_reconstructed)

#     F=np.array([0,0,1,0,0,1]).reshape(6,1)
#     w=np.dot(G_leap_reconstructed,F)

#     # print('wrench',w)

#     Fimp = np.dot(np.linalg.pinv(G_leap),np.dot(Kp_d , (q_final.reshape(6,1))))
#     # Fnull = (I - np.matmul(np.linalg.pinv(G_leap), G_leap)) @ n0
    
#     # Compute desired torque
    

#     Fnull = (I - np.matmul(np.linalg.pinv(G_leap_reconstructed), G_leap_reconstructed)) @ n0
#     Tau_dy = Jh_leap.T @ (Fimp*0 + Fnull)
#     Tau_dy_J = Jh_leap_J.T @ (Fimp*0 + Fnull)

#     print('Tau',Tau_dy)
#     print('Tau_J',Tau_dy_J)

    # print('fnull',Fnull)
    

    # U, s, Vt = np.linalg.svd(G_leap_reconstructed)

    # Sigma = np.zeros((U.shape[0], Vt.shape[0]))
    # np.fill_diagonal(Sigma, s)

    # # Print the results
    # print("Matrix U:")
    # print(U)
    # print("\nSingular values (s):")
    # print(s)
    # print("\nMatrix V^T:")
    # print(Vt)
    # print("\nMatrix Sigma:")
    # print(Sigma)
    
    
    # # Read position from the Leap hand
    # #phi = leap_hand.read_pos()
    # phi=np.ones([8,1])
    
    # # Kinematic control torque
    # Tau_kin = Kp_k * (phi_d - phi)  # Corrected this line
    
    # # Total torque
    # Tau = Tau_dy + Tau_kin*0
    # Tau_sim=Tau
    # # print(Tau)



    # temp = Tau[0].copy()  # Use a temporary variable to hold the value of Tau[0]
    # Tau[0] = Tau[1]
    # Tau[1] = temp

    # Tau_real=np.hstack([Tau[:4].flatten(), np.zeros(8), Tau[-4:].flatten()])
    # print('Tau',Tau_real)

    # print(Tau_real)

    # start_time = time.time()
    # while 0 < time.time() - start_time < 10:
    #     print(f"Elapsed Time: {time.time() - start_time:.2f} seconds")
    #     leap_hand.set_desired_torque(Tau_real)
        
    #     actual_position = leap_hand.read_pos()  # Read the actual position
    #     print("Sent Torque:", Tau_real)
        
    #     print("Actual Position:", actual_position)

    temp = qs_real[0].copy()  # Use a temporary variable to hold the value of Tau[0]
    qs_real[0] = qs_real[1]
    qs_real[1] = temp

    save_to_csv(r_theta,G_leap,Jh_leap,qs_real,obj_pos)

    while True:
        leap_hand.set_allegro(qs_real)
    

    

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
    