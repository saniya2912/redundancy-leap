import numpy as np
import pickle
import os
import fcntl
import time
from main_redundancy import GraspClass, TransMatrix, OnlyPosIK  # Adjust the import path based on your file structure

def load_object_pose_cam():
    """
    Loads the most recent object_pose_cam array from the file with file locking.
    """
    with open('object_pose_cam.pkl', 'rb') as f:
        fcntl.flock(f, fcntl.LOCK_SH)  # Shared lock for reading
        object_pose_cam = pickle.load(f)
        fcntl.flock(f, fcntl.LOCK_UN)  # Release the lock after reading
    print("Loaded object_pose_cam from file.")
    return object_pose_cam

def wait_for_new_file(filename, last_mod_time):
    """
    Waits for the file to be updated with a new modification time.
    """
    while True:
        new_mod_time = os.path.getmtime(filename)
        if new_mod_time > last_mod_time:
            print("New file detected!")
            return new_mod_time
        time.sleep(0.1)  # Wait for 1 second before checking again

def run_grasp_control():
    """
    Main control loop that loads the object_pose_cam from the file and uses it for control logic.
    """
    last_mod_time = 0
    grasp = GraspClass()
    transmatrix = TransMatrix()

    index_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_index.xml'
    thumb_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_thumb.xml'
    
    invkin_index=OnlyPosIK(index_path)
    invkin_thumb=OnlyPosIK(thumb_path)

    n = 2
    palm_to_cam = np.array([[ 0.1881993,   0.94833672, -0.25542343,  0.03447044],
    [-0.73070002, -0.03857433, -0.68160766, -0.00364414],
    [-0.65624541,  0.31491604,  0.68568879,  0.6054304 ],
    [ 0.,          0.,          0.,          1.        ]])

    b1 = np.array([[0],[0.027],[0]])
    b2 = np.array([[0],[-0.027],[0]])
    bs = [b1, b2]

    T_indexbase_palm=np.array([[0.        , 0.        , 1.        , 0.01309895],
       [1.        , 0.        , 0.        , -0.0027],
       [0.        , 1.        , 0.        , 0.016012  ],
       [0.        , 0.        , 0.        , 1.        ]])
    T_palm_indexbase = np.linalg.inv(T_indexbase_palm)

    T_thumbbase_palm=np.array([[0.        , 0.        , 1.        , -0.0493],
       [0.        , 1.        , 0.        , -0.0027],
       [-1.        , 0.        , 0.        , 0.0131  ],
       [0.        , 0.        , 0.        , 1.        ]])

    Rpk_index=T_indexbase_palm[:3,:3]
    Rpk_thumb=T_thumbbase_palm[:3,:3]

    

    
    # Wait for a new file modification
    last_mod_time = wait_for_new_file('object_pose_cam.pkl', last_mod_time)

    # Load the latest object_pose_cam data
    object_pose_cam = load_object_pose_cam()
    #print(object_pose_cam)

    contact1_orient, contact2_orient, r_theta = transmatrix.compute_finger_rotations(object_pose_cam, palm_to_cam)
    contact1_pos,contact2_pos=transmatrix.compute_contact_locations(object_pose_cam, palm_to_cam,bs)

    qs1=[0.5,0,0.5,0.5]
    qs2=[0.5,0,0.5,0.5]

    J1 = grasp.J(grasp.index_path,'contact_index',qs1)
    J2 = grasp.J(grasp.thumb_path,'contact_thumb',qs2)

    contact_orientations = [contact1_orient, contact2_orient]
    Rpks = [Rpk_index, Rpk_thumb]
    Js = [J1, J2]


    print('contact',contact1_pos)
    goal_pos_index_hom=T_palm_indexbase@np.append(contact1_pos,1)
    goal_pos_index=goal_pos_index_hom[:3]
    

        #q_index_mujoco=invkin_index.calculate(goal_pos_index, 'contact_index')

       # print(q_index_mujoco)

if __name__ == "__main__":
    run_grasp_control()  # Run the control loop using the latest file data