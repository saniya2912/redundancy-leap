# import pickle
# import numpy as np

# def f(array):
#     """
#     Example function to compute on the array.
#     """
#     if array.ndim == 2 and array.shape[0] == array.shape[1]:  # Check if it's a square matrix
#         determinant = np.linalg.det(array)
#         print("Determinant of the matrix:", determinant)
#         return determinant
#     else:
#         print("The input array is not a square matrix.")
#         return None

# def load_and_compute(filename):
#     """
#     Load the array from a pickle file and compute f(array).
#     """
#     try:
#         with open(filename, 'rb') as file:  # Changed the variable name to 'file' to avoid conflicts
#             print(f"Type of file: {type(file)}")  # Check if file is properly opened
#             print("Attempting to load array from file...")

#             array = pickle.load(file)  # Ensure this is using the correct `pickle.load` method
#             print("Loaded array:", array)

#             # Compute f(array)
#             result = f(np.array(array))
#             print("Result of computation:", result)

#     except FileNotFoundError:
#         print(f"File '{filename}' not found. Make sure the file exists.")
#     except Exception as e:
#         print(f"An error occurred: {e}")

# if __name__ == "__main__":
#     # Specify the filename of the saved array
#     filename = '/tmp/received_array.pkl'
#     load_and_compute(filename)


import pickle
import numpy as np
import time 
from main_redundancy import GraspClass, TransMatrix, OnlyPosIK 

# grasp = GraspClass()
transmatrix = TransMatrix()

index_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_index.xml'
# thumb_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_thumb.xml'

# invkin_index=OnlyPosIK(index_path)
# invkin_thumb=OnlyPosIK(thumb_path)

# n = 2
palm_wrt_cam = np.array([[-0.01209287,  0.99825156,  0.05721033,  0.01685895],
                        [-0.84305334,  0.02058774, -0.53744149, -0.01142683],
                        [-0.53770053, -0.05473075,  0.84135842,  0.50268827],
                        [0.,  0.,  0.,  1.]])

b1 = np.array([[0],[0.027],[0]])
b2 = np.array([[0],[-0.027],[0]])
bs = [b1, b2]

# T_indexbase_palm=np.array([[0.        , 0.        , 1.        , 0.01309895],
#     [1.        , 0.        , 0.        , -0.0027],
#     [0.        , 1.        , 0.        , 0.016012  ],
#     [0.        , 0.        , 0.        , 1.        ]])
# T_palm_indexbase = np.linalg.inv(T_indexbase_palm)

# T_thumbbase_palm=np.array([[0.        , 0.        , 1.        , -0.0493],
#     [0.        , 1.        , 0.        , -0.0027],
#     [-1.        , 0.        , 0.        , 0.0131  ],
#     [0.        , 0.        , 0.        , 1.        ]])

# Rpk_index=T_indexbase_palm[:3,:3]
# Rpk_thumb=T_thumbbase_palm[:3,:3]

def f(array):
    object_pose_cam = array
    #print(object_pose_cam)

    # contact1_orient, contact2_orient, r_theta = transmatrix.compute_finger_rotations(object_pose_cam, palm_to_cam)
    # contact1_pos,contact2_pos=transmatrix.compute_contact_locations(object_pose_cam, palm_to_cam,bs)

    # qs1=[0.5,0,0.5,0.5]
    # qs2=[0.5,0,0.5,0.5]

    # J1 = grasp.J(grasp.index_path,'contact_index',qs1)
    # J2 = grasp.J(grasp.thumb_path,'contact_thumb',qs2)

    # contact_orientations = [contact1_orient, contact2_orient]
    # Rpks = [Rpk_index, Rpk_thumb]
    # Js = [J1, J2]

    pos_hardware_frame = transmatrix.compute_obj_pos(object_pose_cam, palm_wrt_cam)
    print("Object position:", pos_hardware_frame)

    contact_pos_hardware=transmatrix.compute_contact_locations(object_pose_cam, palm_wrt_cam,bs)

    print('contact_thumb',contact_pos_hardware[0])
    # goal_pos_index_hom=T_palm_indexbase@np.append(contact1_pos,1)
    # goal_pos_index=goal_pos_index_hom[:3]
    

def load_and_compute(filename):
    """
    Continuously load the array from a pickle file and compute f(array).
    """
    previous_array = None  # To track the last loaded array
    while True:  # Loop indefinitely
        try:
            with open(filename, 'rb') as file:
                #print("Attempting to load array from file...")

                array = pickle.load(file)  # Load the array
                #print("Loaded array:", array)

                # Check if the array has changed before computing
                if not np.array_equal(array, previous_array):
                    # Compute f(array)
                    result = f(np.array(array))
                    #print("Result of computation:", result)

                    # Update the previous_array to the current one
                    previous_array = array
                else:
                    print("Array has not changed. Waiting for updates...")

            # Sleep for a short duration before checking again
            time.sleep(1)  # Adjust the sleep duration as needed

        except FileNotFoundError:
            print(f"File '{filename}' not found. Make sure the file exists.")
            time.sleep(2)  # Wait before retrying
        except Exception as e:
            print(f"An error occurred: {e}")
            time.sleep(2)  # Wait before retrying

if __name__ == "__main__":
    # Specify the filename of the saved array
    filename = '/tmp/received_array.pkl'
    load_and_compute(filename)

