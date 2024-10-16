from main_redundancy import LeapNode_Poscontrol

leap=LeapNode_Poscontrol()

qs=leap.read_pos_leap()

while True:
    leap.set_allegro(qs)
# from main_redundancy import *
# grasp2=GraspClass2()
# F=np.array([0,0,1,0,0,1]).reshape(6,1)
# w=np.dot(grasp2.G(),F)