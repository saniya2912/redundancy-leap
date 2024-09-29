import mujoco
import mujoco.viewer
import time

def set_joint_angle(data, joint_index, new_angle):
    """
    Set the angle of a specific joint in the simulation.
    """
    # Set the joint position
    data.qpos[joint_index] = new_angle

def main():
    # Load the model and initialize data
    model_path = '/home/iitgn-robotics/Downloads/mujoco-3.1.6-linux-x86_64/mujoco-3.1.6/model/leap hand/leaphand.xml'
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    # Launch the viewer
    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        while viewer.is_running():
            step_start = time.time()

            # Example: Set the angle of the first joint to 0.5 radians
            set_joint_angle(d, joint_index=0, new_angle=0.5)
            set_joint_angle(d, joint_index=1, new_angle=0.5)
            set_joint_angle(d, joint_index=2, new_angle=0.5)
            set_joint_angle(d, joint_index=3, new_angle=0.5)
            set_joint_angle(d, joint_index=4, new_angle=0.5)
            set_joint_angle(d, joint_index=5, new_angle=0.5)
            set_joint_angle(d, joint_index=6, new_angle=0.5)
            set_joint_angle(d, joint_index=7, new_angle=0.5)
            set_joint_angle(d, joint_index=8, new_angle=0.5)
            set_joint_angle(d, joint_index=9, new_angle=0.5)
            set_joint_angle(d, joint_index=10, new_angle=0.5)
            set_joint_angle(d, joint_index=11, new_angle=0.5)
            set_joint_angle(d, joint_index=12, new_angle=0.5)
            set_joint_angle(d, joint_index=13, new_angle=0.5)
            set_joint_angle(d, joint_index=14, new_angle=0.5)
            set_joint_angle(d, joint_index=14, new_angle=0.5)


            # Step the simulation
            mujoco.mj_step(m, d)

            # Example modification of a viewer option: toggle contact points every two seconds.
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

            # Sync the viewer with the simulation state
            viewer.sync()

            # Rudimentary time keeping
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            # Add a delay to keep the viewer open
            time.sleep(0.1)  # Adjust the delay as needed

        # Optional: Add an additional delay after exiting the main loop
        time.sleep(5)  # Keeps the viewer open for 5 more seconds after the loop exits

if __name__ == "__main__":
    main()
