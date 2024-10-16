from main_redundancy import LeapNode_Taucontrol

leap=LeapNode_Taucontrol()

while True:
    leap.set_desired_torque([-0.00538729,  0.02930365,  0.02037127,  0.01092092,  0.,          0.,
  0.,          0.,          0.,          0.,          0. ,         0.,
  0, -0.00076342,  0.01565535,  0.00813734])
# from main_redundancy import *
# grasp2=GraspClass2()
# F=np.array([0,0,1,0,0,1]).reshape(6,1)
# w=np.dot(grasp2.G(),F)