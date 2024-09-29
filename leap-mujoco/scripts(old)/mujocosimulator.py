import mujoco
import mujoco.viewer


model = mujoco.MjModel.from_xml_path('/home/iitgn-robotics/Saniya/mujoco-3.1.6/model/leap hand/index_thumb.xml')
data = mujoco.MjData(model)

print(f'Number of actuators in the model: {model.nu}')


# create the viewer object
viewer =mujoco.viewer.launch(model, data) #mujoco_viewer.MujocoViewer(model, data)

# # simulate and render
# for _ in range(10000):
#     if viewer.is_alive:
#         mujoco.mj_step(model, data)
#         viewer.render()
#     else:
#         break

# close
viewer.close()
