import pickle
import numpy as np
from main_redundancy import GraspClass, TransMatrix, OnlyPosIK
from main import LeapNode
import time
import pandas as pd 
# grasp = GraspClass()
transmatrix = TransMatrix()
leap=LeapNode()
grasp=GraspClass()

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




b1 = np.array([[0],[-0.027],[0]])
b2 = np.array([[0],[0.027],[0]])
bs = [b1, b2]

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
    combined_df.to_csv('pos2.csv', index=False)





def f(array):
    object_pose_cam=array
    obj_pos=transmatrix.compute_obj_pos(object_pose_cam, palm_wrt_cam) #object position with respect to palm camera frame

    contactpos_1,contactpos_2=transmatrix.compute_contact_locations(object_pose_cam, palm_wrt_cam,bs)
    
    qs1=pos_ik_index.calculate(contactpos_1,'contact_index')
    qs2=pos_ik_thumb.calculate(contactpos_2,'contact_thumb')
    qs1_real=qs1
    qs2_real=qs2
    qs=np.concatenate([qs1,np.zeros(8),qs2])
    qs1_real[0],qs1_real[1]=qs1_real[1],qs1_real[0]
    qs_real=np.concatenate([qs1_real,np.zeros(8),qs2_real])

    contactrot_index,contactrot_thumb,r_theta=transmatrix.compute_finger_rotations(object_pose_cam,palm_wrt_cam)
    contact_orientations=[contactrot_index,contactrot_thumb]
    G=grasp.G(n,contact_orientations,r_theta,bs)
    

    J_index=grasp.J(index_path,'contact_index',qs1)
    J_thumb=grasp.J(thumb_path,'contact_thumb',qs2)
    Js=[J_index,J_thumb]

    Jh=grasp.Jh(n,contact_orientations,Rpks,Js)

    save_to_csv(r_theta,G,Jh,qs_real,obj_pos)
    

    while True:
        leap.set_allegro(qs_real)

    

def load_and_compute(filename):
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
            result = f(np.array(array))
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
    
    load_and_compute(filename)
    