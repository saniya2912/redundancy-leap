from main_redundancy import LeapNode_Taucontrol

leap=LeapNode_Taucontrol()


while True:
    leap.set_desired_torque([ 0.00102226 , 0.05903069  ,0.02262369 , 0.00725438  ,0.        ,  0.,
  0.   ,       0.     ,     0.     ,     0.        ,  0.  ,        0.,
 -0.00732454 ,-0.00060864 , 0.04465692  ,0.04510183])
# from main_redundancy import *
# grasp2=GraspClass2()
# F=np.array([0,0,1,0,0,1]).reshape(6,1)
# w=np.dot(grasp2.G(),F)