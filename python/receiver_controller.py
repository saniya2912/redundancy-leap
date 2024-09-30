import struct
import pickle
import socket
import numpy as np
from main_redundancy import GraspClass, TransMatrix # Importing relevant parts from Code 2

def receive_array():
    host = '10.0.58.25'
    port = 12345

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))

    try:
        while True:
            raw_data_len = client_socket.recv(4)  
            if not raw_data_len:
                print("No data received, closing connection.")
                break

            data_len = struct.unpack('>I', raw_data_len)[0]  
            data = b""
            while len(data) < data_len:
                packet = client_socket.recv(4096)
                if not packet:
                    break
                data += packet

            if len(data) == data_len:
                array = pickle.loads(data)  # Here is the "array"
                print("Received array:", array)
                return array  # Return the array to use it in the next part

    except (ConnectionResetError, OSError) as e:
        print(f"Connection error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        client_socket.close()



def run_grasp_control(object_pose_cam, palm_to_cam):

    grasp = GraspClass()
    rot=TransMatrix(object_pose_cam, palm_to_cam)

    n = 2
    contact1 = np.ones([3,3])
    contact2 = np.ones([3,3])

    Rpk_index = np.array([[0, 1, 0],
                          [0, 0, -1],
                          [-1, 0, 0]])

    Rpk_thumb = np.array([[0, 0, 1],
                          [0, 1, 0],
                          [-1, 0, 0]])

    J1 = np.ones([3,4])
    J2 = np.ones([3,4])
    b1 = np.ones([3,1])
    b2 = np.ones([3,1])

    contact_orientations = [contact1, contact2]
    Rpks = [Rpk_index, Rpk_thumb]
    Js = [J1, J2]
    r_theta = np.ones([3,3])
    bs = [b1, b2]

    Jh_leap = grasp.Jh(n, contact_orientations, Rpks, Js)
    G_leap = grasp.G(n, contact_orientations, r_theta, bs)

    Kp_d = 1
    Kp_k = 1
    qd = 1
    n0 = np.ones([6,1])
    I = np.eye(G_leap.shape[1])
    phi_d = np.ones([8,1])

    while True:
        q = np.ones([6,1])
        Fimp = np.linalg.pinv(G_leap) @ (Kp_d * (qd - q))
        Fnull = (I - np.matmul(np.linalg.pinv(G_leap), G_leap)) @ n0
        Tau_dy = Jh_leap.T @ (Fimp + Fnull)
        phi = np.ones([8,1])  # This is where you'd use your leap hand input
        Tau_kin = Kp_k * (phi_d - phi)
        Tau = Tau_dy + Tau_kin
        print("Tau:", Tau)
        break  # Avoid infinite loop for testing

if __name__ == "__main__":
    palm_to_cam = np.array([[-0.01209287,  0.99825156,  0.05721033,  0.01685895],
                            [-0.84305334,  0.02058774, -0.53744149, -0.01142683],
                            [-0.53770053, -0.05473075,  0.84135842,  0.50268827],
                            [ 0.,  0.,  0.,  1.]])
    object_pose_cam = receive_array()  # Receive array from Code 1
    run_grasp_control(object_pose_cam,palm_to_cam)  # Pass it to Code 2 logic