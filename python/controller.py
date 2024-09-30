from main_redundancy import LeapNode_Poscontrol, LeapNode_Taucontrol, GradientDescentIK, OnlyPosIK, GraspClass
import numpy as np

grasp = GraspClass()

n = 2
contact1 = np.ones([3,3])
contact2 = np.ones([3,3])

palm_to_cam=np.array([[-0.01209287,  0.99825156,  0.05721033,  0.01685895],
       [-0.84305334,  0.02058774, -0.53744149, -0.01142683],
       [-0.53770053, -0.05473075,  0.84135842,  0.50268827],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

#from mujoco xml file
Rpk_index = np.array([[0, 1, 0],
                      [0, 0, -1],
                      [-1, 0, 0]])

Rpk_thumb = np.array([[0, 0, 1],
                      [0, 1, 0],
                      [-1, 0, 0]])


J1 = np.ones([3,4]) #ready just plug in
J2 = np.ones([3,4])
b1 = np.ones([3,1])
b2 = np.ones([3,1])

contact_orientations = [contact1, contact2]
Rpks = [Rpk_index, Rpk_thumb]
Js = [J1, J2]
r_theta = np.ones([3,3])
bs = [b1, b2]

# Compute the grasp matrices
Jh_leap = grasp.Jh(n, contact_orientations, Rpks, Js)
G_leap = grasp.G(n, contact_orientations, r_theta, bs)


# Controller parameters
Kp_d = 1
Kp_k = 1
qd = 1
n0 = np.ones([6,1])
I = np.eye(G_leap.shape[1])
phi_d = np.ones([8,1])

#leap_hand = LeapNode_Taucontrol()

# Main control loop
while True:
    q = np.ones([6,1])
    
    # Compute forces
    Fimp = np.linalg.pinv(G_leap) @ (Kp_d * (qd - q))
    Fnull = (I - np.matmul(np.linalg.pinv(G_leap), G_leap)) @ n0
    
    
    # Compute desired torque
    Tau_dy = Jh_leap.T @ (Fimp + Fnull)
    
    
    # Read position from the Leap hand
    #phi = leap_hand.read_pos()
    phi=np.ones([8,1])
    
    # Kinematic control torque
    Tau_kin = Kp_k * (phi_d - phi)  # Corrected this line
    
    # Total torque
    Tau = Tau_dy + Tau_kin

    
    # Some form of termination or exit condition should go here
    break  # Temporary break to avoid infinite loop
