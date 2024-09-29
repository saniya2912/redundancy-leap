
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import matplotlib.pyplot as plt

xml_path = '/home/ubuntu/Documents/mujoco200_linux/finger1/1fing.xml'  # path of the xml file

# callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# External force variables
apply_force = False
force_magnitude = 0.2
force_direction = np.array([0.0, 1.0, 0.0])  # Force direction
force_applied = np.zeros(3)  # store the applied force

# Lists to store control force data for plotting
time_data = []
left_finger_force_data = []
right_finger_force_data = []

def init_controller(model, data):
    print("Initializing controller...")
    
    initial_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  #initial positions for each joint
    data.qpos[:len(initial_pos)] = initial_pos

    #position of 'box' joint fixed in the x and y directions
    box_joint_index = 0  
    data.qpos[box_joint_index] = -0.005  # desired fixed position in the x direction
    data.qpos[box_joint_index + 1] = 0.05  # desired fixed position in the y direction

def controller(model, data):
    # Define desired positions and velocities for each joint
    desired_pos = np.array([0, 0, 0, 0, 0, 0, 0, 0])  # Desired joint positions
    desired_vel = np.array([10, 0, 50, 50, 50, 0, 50, 50])  # Desired joint velocities

    #impedance control parameters for each joint
    K = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Stiffness
    D = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])  # Damping

    # Define joint IDs for the fingers (assuming they are ordered correctly in the model)
    joint_names = ['box', '1', '2', '3', '4', '5', '6', '7']
    joint_ids = [0, 1, 2, 3, 4, 5, 6, 7]

    # Call the impedance control function
    impedance_control(model, data, desired_pos, desired_vel, K, D, joint_ids)

def impedance_control(model, data, desired_pos, desired_vel, K, D, joint_ids, contact_threshold=0.5, disturbance_threshold=0.1):
    global control_force_data, time_data
    print(f"Size of data.ctrl: {len(data.ctrl)}")
    
    # List to store current control forces for each joint
    current_control_forces = []

    # Loop through each joint and apply impedance control
    for i, joint_id in enumerate(joint_ids):
        # Check if joint_id is within the valid range of data.ctrl
        if joint_id >= len(data.ctrl):
            print(f"Invalid joint_id {joint_id} for data.ctrl of length {len(data.ctrl)}")
            continue

        # current joint position and velocity
        current_pos = data.qpos[joint_id]
        current_vel = data.qvel[joint_id]

        # position error
        pos_error = desired_pos[i] - current_pos

        # control force
        control_force = K[i] * pos_error + D[i] * (desired_vel[i] - current_vel)

        # Adjust control force based on contact forces
        contact_force = np.linalg.norm(data.cfrc_ext[joint_id][:3]) 
        if contact_force > contact_threshold:
            control_force *= 4.5  # Increase control force if contact force exceeds threshold

        # Amplify control force when perturbations are detected
        if np.abs(pos_error) > disturbance_threshold:
            control_force *= 5.0  # Amplification factor for perturbations

        # Apply the control force to the joint
        data.ctrl[joint_id] = control_force

        # Store the control force
        current_control_forces.append(control_force)

        # Debugging output
        print(f"Joint {joint_id}: pos_error={pos_error}, control_force={control_force}, contact_force={contact_force}")

    control_force_data.append(current_control_forces)

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right, apply_force

    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    if button_left and act == glfw.PRESS:
        apply_force = True
    elif button_left and act == glfw.RELEASE:
        apply_force = False

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H
    elif button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H
    elif button_middle:
        action = mj.mjtMouse.mjMOUSE_ZOOM
    else:
        action = None

    if action is not None:
        mj.mjv_moveCamera(model, action, dx/1000, dy/1000, scene, cam)

def scroll(window, xoffset, yoffset):
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, scene, cam)

def render_overlay(model, data, context, viewport, joint_names, joint_ids):
    mj.mjr_overlay(mj.mjtFontScale.mjFONTSCALE_150, mj.mjtFont.mjFONT_NORMAL,
                   viewport, 'Force Sensor Data', '', context)

    for i, joint_id in enumerate(joint_ids):
        joint_name = joint_names[i]
        force_data = data.sensordata[model.sensor_adr[joint_id]:model.sensor_adr[joint_id] + 3]
        force_str = f'{joint_name}: x={force_data[0]:.2f}, y={force_data[1]:.2f}, z={force_data[2]:.2f}'
        mj.mjr_overlay(mj.mjtFontScale.mjFONTSCALE_150, mj.mjtFont.mjFONT_NORMAL,
                       viewport, force_str, '', context)

#initializing
glfw.init()
glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

#initialize mujoco
model = mj.MjModel.from_xml_path(xml_path)  # Load the model
data = mj.MjData(model)                # Create a data structure

cam = mj.MjvCamera()                   # Abstract camera
opt = mj.MjvOption()                   # Visualization options
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=1000)  # Abstract scene
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)  # Custom GPU context

#initialize controller
init_controller(model, data)
mj.set_mjcb_control(controller)

left_finger_sensor_name = "left_finger_force"
right_finger_sensor_name = "right_finger_force"
left_finger_sensor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SENSOR, left_finger_sensor_name)
right_finger_sensor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SENSOR, right_finger_sensor_name)

def print_force_sensor_data(model, data):
    num_sensors = len(model.sensor_adr)
    left_finger_force = data.sensordata[model.sensor_adr[left_finger_sensor_id]:model.sensor_adr[left_finger_sensor_id] + 3]
    right_finger_force = data.sensordata[model.sensor_adr[right_finger_sensor_id]:model.sensor_adr[right_finger_sensor_id] + 3]
    
    print(f"Number of sensors: {num_sensors}")
    print(f"Left Finger Force Sensor Data: {left_finger_force}")
    print(f"Right Finger Force Sensor Data: {right_finger_force}")

# Main loop
while not glfw.window_should_close(window):
    # Step the simulation
    mj.mj_step(model, data)

    # Print force sensor data
    print_force_sensor_data(model, data)

    # Collect force sensor data
    left_finger_force = data.sensordata[left_finger_sensor_id * 3: (left_finger_sensor_id + 1) * 3]
    right_finger_force = data.sensordata[right_finger_sensor_id * 3: (right_finger_sensor_id + 1) * 3]
    
    time_data.append(data.time)
    left_finger_force_data.append(left_finger_force)
    right_finger_force_data.append(right_finger_force)

    # Apply external force to the box if the left mouse button is pressed
    if apply_force:
        box_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "box")  
        force_applied = force_magnitude * force_direction
        torque = np.zeros(3)
        point = data.xpos[box_body_id]
        mj.mj_applyFT(model, data, force_applied, torque, point, box_body_id, data.qfrc_applied)

        print(f"Applying force: {force_applied} to box with body ID: {box_body_id} at point {point}")

    # Update the scene and render
    viewport = mj.MjrRect(0, 0, glfw.get_framebuffer_size(window)[0], glfw.get_framebuffer_size(window)[1])
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()

# Convert force data to numpy arrays for easier manipulation
left_finger_force_data = np.array(left_finger_force_data)
right_finger_force_data = np.array(right_finger_force_data)

# Plotting the forces
plt.figure(figsize=(14, 8))

# Left Finger Force Data
plt.plot(time_data, left_finger_force_data[:, 0], label='Left Finger Force X', linestyle='-', color='blue')
plt.plot(time_data, left_finger_force_data[:, 1], label='Left Finger Force Y', linestyle='--', color='blue')
plt.plot(time_data, left_finger_force_data[:, 2], label='Left Finger Force Z', linestyle=':', color='blue')

# Right Finger Force Data
plt.plot(time_data, right_finger_force_data[:, 0], label='Right Finger Force X', linestyle='-', color='red')
plt.plot(time_data, right_finger_force_data[:, 1], label='Right Finger Force Y', linestyle='--', color='red')
plt.plot(time_data, right_finger_force_data[:, 2], label='Right Finger Force Z', linestyle=':', color='red')

# Plot aesthetics
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')
plt.title('Finger Force Sensor Data Over Time')
plt.legend()
plt.grid(True)
plt.show()
