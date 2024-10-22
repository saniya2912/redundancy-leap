from main_redundancy import LeapNode_Taucontrol

leap=LeapNode_Taucontrol()


while True:
    leap.set_desired_torque([-0.01737768,-0.09891818 -0.16893855, -0.13638068, 0.  ,        0.,
  0.  ,        0.     ,     0.     ,     0.  ,        0.   ,       0.,
  0.0167972 ,  0.0035332 , -0.01879684, 0.10570867])
# from main_redundancy import *
# grasp2=GraspClass2()
# F=np.array([0,0,1,0,0,1]).reshape(6,1)
# w=np.dot(grasp2.G(),F)