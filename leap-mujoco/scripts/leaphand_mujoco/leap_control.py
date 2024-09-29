#!/home/iitgn-robotics/anaconda3/envs/mujoco_env/bin/python3


import numpy as np


import leap_sim_utils.leapsim_utils as lhus
import time
import mujoco
import mujoco.viewer

import mujoco
import numpy as np
import matplotlib.pyplot as plt
import mujoco.viewer
import tempfile
import mediapy as media
import time
import os 
import sys
import matplotlib.pyplot as plt

# sys.path.append(os.path.abspath("/home/iitgn-robotics/Saniya/mujoco-3.1.6/scripts/leaphand_mujoco/mujoco_main.py"))
# print(sys.path)


from mujoco_main import LeapNodeMujoco
# Set the simulation duration and framerate
duration = 5.0  # Duration in seconds
framerate = 30  # Frames per second


def main():
    model_path = '/home/iitgn-robotics/Saniya/mujoco-3.1.6/model/leap hand/leaphand.xml'
    leap_hand = LeapNodeMujoco(model_path)

    #  # Example of setting LEAP positions
    # leap_hand.set_leap([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4])

    leap_hand.step_simulation(duration,framerate)
    #print(leap_hand.d.time)
    
    # # Read and print positions and velocities
    # print("Position:", leap_hand.read_pos())
    # print("Velocity:", leap_hand.read_vel())

if __name__ == "__main__":
    main()




# # if __name__ == "__main__":
# #     if len(sys.argv) != 2:
# #         print("Usage: leap_control.py <model_path>")
# #         sys.exit(1)

# #     model_path = sys.argv[1]
# #     main(model_path)
    
# # model_path = '/home/iitgn-robotics/Saniya/mujoco-3.1.6/model/leap hand/leaphand.xml'
# # #model_path = '/home/iitgn-robotics/Downloads/mujoco-3.1.6-linux-x86_64/mujoco-3.1.6/model/leap hand/test.xml'
# # m = mujoco.MjModel.from_xml_path(model_path)
# # d = mujoco.MjData(m)

# # renderer = mujoco.Renderer(m)

# # # Initialize arrays to store contact forces and weights
# # contact_forces = []
# # weights = []

# # # Initialize list to store frames
# # frames = []
# # cam=mujoco.MjvCamera()

# # options = mujoco.MjvOption()
# # mujoco.mjv_defaultOption(options)
# # options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True

# # # Simulation parameters
# # sim_duration = 200 # duration in seconds
# # framerate = 30  # frames per second

# # # Desired position for the joint
# # desired_position_0 = 1.0  # Target position
# # kp = 10.0  # Proportional gain





# # mujoco.viewer.launch(m,d) #mujoco_viewer.MujocoViewer(model, data)

# # # Show the video using mediapy
# # media.show_video(frames, fps=framerate)