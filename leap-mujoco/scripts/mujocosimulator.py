import mujoco
import mujoco.viewer


model = mujoco.MjModel.from_xml_path('/home/saniya/LEAP/leap_hand_mujoco/model/leap hand/index_finger_diff_jac.xml')
data = mujoco.MjData(model)



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
# while True:
#     mujoco.mj_step(model, data)
#     viewer.render()

# Close the viewer
viewer.close()

