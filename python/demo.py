# from main_redundancy import LeapNode_Taucontrol

# leap=LeapNode_Taucontrol()

# while True:
#     leap.set_desired_torque([ 0.08296698, -1.39005859, -1.19247948, -0.6852598,   0.,          0.,
#   0.,          0. ,         0. ,         0.  ,        0.   ,       0.,
#  -0.40699138, -0.01980462, -0.9056306,  -0.15917859])
from main_redundancy import *
grasp2=GraspClass2()
F=np.array([0,0,1,0,0,1]).reshape(6,1)
w=np.dot(grasp2.G(),F)