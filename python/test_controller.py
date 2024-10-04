# import numpy as np
# import pickle
# import os
# import fcntl
# import time
# from main_redundancy import GraspClass, TransMatrix  # Adjust the import path based on your file structure

# def load_object_pose_cam():
#     """
#     Loads the most recent object_pose_cam array from the file with file locking.
#     """
#     with open('object_pose_cam.pkl', 'rb') as f:
#         fcntl.flock(f, fcntl.LOCK_SH)  # Shared lock for reading
#         object_pose_cam = pickle.load(f)
#         fcntl.flock(f, fcntl.LOCK_UN)  # Release the lock after reading
#     print("Loaded object_pose_cam from file.")
#     return object_pose_cam

# def wait_for_new_file(filename, last_mod_time):
#     """
#     Waits for the file to be updated with a new modification time.
#     """
#     while True:
#         new_mod_time = os.path.getmtime(filename)
#         if new_mod_time > last_mod_time:
#             print("New file detected!")
#             return new_mod_time
#         time.sleep(0.01)  # Wait for 1 second before checking again

# def run_grasp_control():
#     """
#     Main control loop that loads the object_pose_cam from the file and uses it for control logic.
#     """
#     last_mod_time = 0
#     grasp = GraspClass()
#     transmatrix = TransMatrix()

#     n = 2
#     palm_to_cam = np.array([[-0.01209287,  0.99825156,  0.05721033,  0.01685895],
#                             [-0.84305334,  0.02058774, -0.53744149, -0.01142683],
#                             [-0.53770053, -0.05473075,  0.84135842,  0.50268827],
#                             [0.,  0.,  0.,  1.]])

#     while True:
#         # Wait for a new file modification
#         last_mod_time = wait_for_new_file('object_pose_cam.pkl', last_mod_time)
        
#         # Load the latest object_pose_cam data
#         object_pose_cam = load_object_pose_cam()
#         print(object_pose_cam)

#         contact1, contact2, r_theta = transmatrix.compute_finger_rotations(object_pose_cam, palm_to_cam)

#         Rpk_index = np.array([[0, 1, 0],
#                               [0, 0, -1],
#                               [-1, 0, 0]])

#         Rpk_thumb = np.array([[0, 0, 1],
#                               [0, 1, 0],
#                               [-1, 0, 0]])
#         qs1=[0.5,0,0.5,0.5]
#         qs2=[0.5,0,0.5,0.5]
        
#         J1 = grasp.J(grasp.index_path,'contact_index',qs1)
#         J2 = grasp.J(grasp.thumb_path,'contact_thumb',qs2)
#         b1 = np.ones([3, 1])
#         b2 = np.ones([3, 1])

#         # contact_orientations = [contact1, contact2]
#         # Rpks = [Rpk_index, Rpk_thumb]
#         # Js = [J1, J2]
#         # bs = [b1, b2]

#         # # Compute the grasp matrices
#         # Jh_leap = grasp.Jh(n, contact_orientations, Rpks, Js)
#         # G_leap = grasp.G(n, contact_orientations, r_theta, bs)

#         # Kp_d = 1
#         # Kp_k = 1
#         # qd = 1
#         # n0 = np.ones([6, 1])
#         # I = np.eye(G_leap.shape[1])
#         # phi_d = np.ones([8, 1])

#         # Main control loop (single iteration here for simplicity)
#         # q = np.ones([6, 1])
#         # Fimp = np.linalg.pinv(G_leap) @ (Kp_d * (qd - q))
#         # Fnull = (I - np.matmul(np.linalg.pinv(G_leap), G_leap)) @ n0
#         # Tau_dy = Jh_leap.T @ (Fimp + Fnull)
#         # phi = np.ones([8, 1])  # Dummy placeholder
#         # Tau_kin = Kp_k * (phi_d - phi)
#         # Tau = Tau_dy + Tau_kin

#         # Compute object position
#         pos = transmatrix.compute_obj_pos(object_pose_cam, palm_to_cam)
#         print("Object position:", pos)

# if __name__ == "__main__":
#     run_grasp_control()  # Run the control loop using the latest file data


# import numpy as np
# import pickle
# from main_redundancy import GraspClass ,TransMatrix # Adjust the import path based on your file structure

# def load_object_pose_cam():
#     """
#     Loads the most recent object_pose_cam array from the file.
#     """
#     with open('object_pose_cam.pkl', 'rb') as f:
#         object_pose_cam = pickle.load(f)
#     print("Loaded object_pose_cam from file.")
#     return object_pose_cam

# def run_grasp_control():
#     """
#     Main control loop that loads the object_pose_cam from the file and uses it for control logic.
#     """
#     # Load the latest object_pose_cam data
#     object_pose_cam = load_object_pose_cam()
    
#     # Example usage of object_pose_cam (continue with your control logic here)
#     grasp = GraspClass()
#     transmatrix=TransMatrix()


#     n = 2
#     palm_to_cam = np.array([[-0.01209287,  0.99825156,  0.05721033,  0.01685895],
#                             [-0.84305334,  0.02058774, -0.53744149, -0.01142683],
#                             [-0.53770053, -0.05473075,  0.84135842,  0.50268827],
#                             [ 0.,  0.,  0.,  1.]])
    
#     contact1,contact2,r_theta = transmatrix.compute_finger_rotations(object_pose_cam, palm_to_cam)

#     Rpk_index = np.array([[0, 1, 0],
#                           [0, 0, -1],
#                           [-1, 0, 0]])

#     Rpk_thumb = np.array([[0, 0, 1],
#                           [0, 1, 0],
#                           [-1, 0, 0]])

#     J1 = np.ones([3, 4])
#     J2 = np.ones([3, 4])
#     b1 = np.ones([3, 1])
#     b2 = np.ones([3, 1])

#     contact_orientations = [contact1, contact2]
#     Rpks = [Rpk_index, Rpk_thumb]
#     Js = [J1, J2]
#     bs = [b1, b2]

#     # Compute the grasp matrices
#     Jh_leap = grasp.Jh(n, contact_orientations, Rpks, Js)
#     G_leap = grasp.G(n, contact_orientations, r_theta, bs)

#     Kp_d = 1
#     Kp_k = 1
#     qd = 1
#     n0 = np.ones([6, 1])
#     I = np.eye(G_leap.shape[1])
#     phi_d = np.ones([8, 1])

#     # Main control loop (single iteration here for simplicity)
#     q = np.ones([6, 1])
#     Fimp = np.linalg.pinv(G_leap) @ (Kp_d * (qd - q))
#     Fnull = (I - np.matmul(np.linalg.pinv(G_leap), G_leap)) @ n0
#     Tau_dy = Jh_leap.T @ (Fimp + Fnull)
#     phi = np.ones([8, 1])  # Dummy placeholder
#     Tau_kin = Kp_k * (phi_d - phi)
#     Tau = Tau_dy + Tau_kin
#     # print("Torque:", Tau)
    
#     pos=transmatrix.compute_obj_pos(object_pose_cam, palm_to_cam)
#     print(pos)



# if __name__ == "__main__":
#     while True:
#         run_grasp_control()  # Run the control loop using the latest file data

# import numpy as np
# import pickle
# import fcntl
# import time
# from main_redundancy import GraspClass, TransMatrix  # Adjust the import path based on your file structure

# def load_object_pose_cam():
#     """
#     Loads the most recent object_pose_cam array from the file with file locking.
#     """
#     with open('object_pose_cam.pkl', 'rb') as f:
#         fcntl.flock(f, fcntl.LOCK_SH)  # Shared lock for reading
#         object_pose_cam = pickle.load(f)
#         fcntl.flock(f, fcntl.LOCK_UN)  # Release the lock after reading
#     print("Loaded object_pose_cam from file.")
#     return object_pose_cam

# def run_grasp_control():
#     """
#     Main control loop that loads the object_pose_cam from the file and uses it for control logic.
#     """
#     grasp = GraspClass()
#     transmatrix = TransMatrix()

#     n = 2
#     palm_to_cam = np.array([[-0.01209287,  0.99825156,  0.05721033,  0.01685895],
#                             [-0.84305334,  0.02058774, -0.53744149, -0.01142683],
#                             [-0.53770053, -0.05473075,  0.84135842,  0.50268827],
#                             [0.,  0.,  0.,  1.]])

#     while True:
#         # Continuously load the latest object_pose_cam data
#         object_pose_cam = load_object_pose_cam()
#         print(object_pose_cam)

#         contact1, contact2, r_theta = transmatrix.compute_finger_rotations(object_pose_cam, palm_to_cam)

#         Rpk_index = np.array([[0, 1, 0],
#                               [0, 0, -1],
#                               [-1, 0, 0]])

#         Rpk_thumb = np.array([[0, 0, 1],
#                               [0, 1, 0],
#                               [-1, 0, 0]])

#         J1 = np.ones([3, 4])
#         J2 = np.ones([3, 4])
#         b1 = np.ones([3, 1])
#         b2 = np.ones([3, 1])

#         contact_orientations = [contact1, contact2]
#         Rpks = [Rpk_index, Rpk_thumb]
#         Js = [J1, J2]
#         bs = [b1, b2]

#         # Compute the grasp matrices
#         Jh_leap = grasp.Jh(n, contact_orientations, Rpks, Js)
#         G_leap = grasp.G(n, contact_orientations, r_theta, bs)

#         # Compute object position
#         pos = transmatrix.compute_obj_pos(object_pose_cam, palm_to_cam)
#         print("Object position:", pos)

#         # Add a small delay to avoid excessive CPU usage
#         time.sleep(0.1)

# if __name__ == "__main__":
#     run_grasp_control()

import numpy as np
import pickle
import os
import fcntl
import time
from main_redundancy import GraspClass, TransMatrix  # Adjust the import path based on your file structure

def load_object_pose_cam():
    """
    Loads the most recent object_pose_cam array from the file with file locking.
    """
    with open('object_pose_cam.pkl', 'rb') as f:
        fcntl.flock(f, fcntl.LOCK_SH)  # Shared lock for reading
        object_pose_cam = pickle.load(f)
        fcntl.flock(f, fcntl.LOCK_UN)  # Release the lock after reading
    return object_pose_cam

def wait_for_new_file(filename, last_mod_time, timeout=10):  # Increased timeout
    """
    Waits for the file to be updated with a new modification time or times out after a certain period.
    """
    start_time = time.time()
    while True:
        new_mod_time = os.path.getmtime(filename)
        if new_mod_time > last_mod_time:
            print("New file detected!")
            return new_mod_time
        if time.time() - start_time > timeout:
            print("Timeout waiting for new file.")
            return last_mod_time
        time.sleep(0.01)  # Wait for 10 milliseconds before checking again

def run_grasp_control():
    last_mod_time = 0
    last_object_pose = None  # Store the last loaded object pose
    grasp = GraspClass()
    transmatrix = TransMatrix()

    palm_to_cam = np.array([[-0.01209287,  0.99825156,  0.05721033,  0.01685895],
                             [-0.84305334,  0.02058774, -0.53744149, -0.01142683],
                             [-0.53770053, -0.05473075,  0.84135842,  0.50268827],
                             [0.,  0.,  0.,  1.]])

    while True:
        # Wait for a new file modification
        last_mod_time = wait_for_new_file('object_pose_cam.pkl', last_mod_time)

        # Load the latest object_pose_cam data
        object_pose_cam = load_object_pose_cam()
        print("Loaded object_pose_cam:", object_pose_cam)

        # Check if the loaded data is different from the last loaded data
        if last_object_pose is None or not np.array_equal(object_pose_cam, last_object_pose):
            print("New data detected.")  # Track data changes
            contact1, contact2, r_theta = transmatrix.compute_finger_rotations(object_pose_cam, palm_to_cam)

            # Compute object position
            pos = transmatrix.compute_obj_pos(object_pose_cam, palm_to_cam)
            print("Object position:", pos)

            # Update the last loaded object pose
            last_object_pose = object_pose_cam
        else:
            print("No change in object_pose_cam data.")

if __name__ == "__main__":
    run_grasp_control()  # Run the control loop using the latest file data
