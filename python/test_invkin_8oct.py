import pickle
import numpy as np
from main_redundancy import GraspClass, TransMatrix, OnlyPosIK, convert_to_mujoco
from main import LeapNode
import time
# grasp = GraspClass()
transmatrix = TransMatrix()
# leap=LeapNode()


index_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/index_finger.xml'
thumb_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/thumb.xml'

pos_ik_index=OnlyPosIK(index_path)
pos_ik_thumb=OnlyPosIK(thumb_path)

T_indexbase_palm=np.array([[0.        , 0.        , 1.        , 0.01309895],
    [1.        , 0.        , 0.        , -0.0027],
    [0.        , 1.        , 0.        , 0.016012  ],
    [0.        , 0.        , 0.        , 1.        ]])
T_palm_indexbase = np.linalg.inv(T_indexbase_palm)

T_thumbbase_palm=np.array([[0.        , 0.        , 1.        , -0.0493],
    [0.        , 1.        , 0.        , -0.0027],
    [-1.        , 0.        , 0.        , 0.0131  ],
    [0.        , 0.        , 0.        , 1.        ]])
# n = 2
palm_wrt_cam = np.array([[-0.01255732,  0.97356176,  0.22806171,  0.04361352],
                   [-0.87234324,  0.1008127,  -0.47838551,  0.04156835],
                   [-0.48873109, -0.2049571,   0.84801656,  0.499567],
                   [ 0.,          0.,          0.,          1.        ]])

mujoco_convert=convert_to_mujoco(palm_wrt_cam)


b1 = np.array([[0],[0.027],[0]])
b2 = np.array([[0],[-0.027],[0]])
bs = [b1, b2]


def f(array):
    object_pose_cam=array
    obj_pos=transmatrix.compute_obj_pos(object_pose_cam, palm_wrt_cam) #object position with respect to palm camera frame
    print('obj_pos',obj_pos)

    # #convert to mujoco requirements
    # v3=palm_wrt_cam[:3,3]+mujoco_convert.x_pbm_preal_c()-object_pose_cam[:3,3]
    # x_obj_pbm=np.dot(np.dot(palm_wrt_cam[:3,:3].T,mujoco_convert.T_pbm_preal[:3,:3].T),-v3)

    x_obj_pbm=mujoco_convert.obj_pbm_from_preal(obj_pos)

    
    
    print('obj_pos_mujoco',x_obj_pbm)

    # obj_pos_mujoco=np.array([0,0,-0.1])
    
    # qs1=pos_ik_index.calculate(x_obj_pbm,'contact_index')
    # qs2=pos_ik_thumb.calculate(x_obj_pbm,'contact_thumb')
    # qs1_real=qs1
    # qs2_real=qs2
    # qs1_real[0],qs1_real[1]=qs1_real[1],qs1_real[0]
    # qs=np.concatenate([qs1_real,np.zeros(8),qs2_real])
    # print(qs)
    # while True:
    #     leap.set_allegro(qs)

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
    while True:
        load_and_compute(filename)
        time.sleep(1)