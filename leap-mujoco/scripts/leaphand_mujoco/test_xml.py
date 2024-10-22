import mujoco
import mujoco.viewer


model = mujoco.MjModel.from_xml_path('/home/iitgn-robotics/Saniya/redundancy-leap/leap-mujoco/model/leap hand/leaphand_redundancy.xml')
data = mujoco.MjData(model)

# create the viewer object
viewer =mujoco.viewer.launch(model, data) #mujoco_viewer.MujocoViewer(model, data)
