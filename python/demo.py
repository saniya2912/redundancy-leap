import mujoco
import numpy as np
from main_redundancy import GraspClass

index_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_index.xml'
index_m = mujoco.MjModel.from_xml_path(index_path)
index_d = mujoco.MjData(index_m)
middle_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_middle.xml'
middle_m = mujoco.MjModel.from_xml_path(middle_path)
middle_d = mujoco.MjData(middle_m)
ring_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_ring.xml'
ring_m = mujoco.MjModel.from_xml_path(ring_path)
ring_d = mujoco.MjData(ring_m)
thumb_path='/home/saniya/LEAP/redundancy-leap/leap-mujoco/model/leap hand/redundancy/0_thumb.xml'
thumb_m = mujoco.MjModel.from_xml_path(thumb_path)
thumb_d = mujoco.MjData(thumb_m)

grasp=GraspClass()

index_d.qpos=[np.pi/2,0,np.pi/2,np.pi/2]
index_J=grasp.J(index_m,index_d,'contact_index')
print(index_J)