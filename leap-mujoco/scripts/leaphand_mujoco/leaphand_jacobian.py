import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import numpy as np

def find_joint_for_link(urdf_model, child_link_name):
    """ Find the joint in the URDF model that connects to the given child link. """
    for joint in urdf_model.joints:
        if joint.child == child_link_name:
            return joint
    return None

def add_joint_to_chain(chain, joint):
    if joint.type == 'revolute' or joint.type == 'continuous':
        kdl_joint = kdl.Joint(
            joint.name,
            kdl.Vector(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]),
            kdl.Vector(joint.axis[0], joint.axis[1], joint.axis[2]),
            kdl.Joint.RotAxis
        )
    elif joint.type == 'fixed':
        kdl_joint = kdl.Joint(joint.name, kdl.Joint.Fixed)
    else:
        print(f"Unsupported joint type: {joint.type}")
        return False

    kdl_segment = kdl.Segment(joint.child, kdl_joint, kdl.Frame())
    chain.addSegment(kdl_segment)
    return True

def create_kdl_chain_from_urdf(urdf_model, base_link, end_link):
    chain = kdl.Chain()
    current_link = end_link

    while current_link != base_link:
        joint = find_joint_for_link(urdf_model, current_link)
        if not joint:
            print(f"Joint for link {current_link} not found!")
            return None
        if not add_joint_to_chain(chain, joint):
            print(f"Failed to add joint {joint.name} to chain")
            return None
        current_link = joint.parent

    return chain

def perform_fk(chain, joint_positions):
    fk_solver = kdl.ChainFkSolverPos_recursive(chain)
    end_effector_frame = kdl.Frame()
    print(f"Calculating FK for joint positions: {[joint_positions[i] for i in range(joint_positions.rows())]}")
    result = fk_solver.JntToCart(joint_positions, end_effector_frame)

    if result >= 0:
        return end_effector_frame
    else:
        raise RuntimeError("FK solver failed")

def compute_jacobian(chain, joint_positions):
    jacobian = kdl.Jacobian(chain.getNrOfJoints())
    jnt_to_jac_solver = kdl.ChainJntToJacSolver(chain)

    result = jnt_to_jac_solver.JntToJac(joint_positions, jacobian)

    if result >= 0:
        return jacobian
    else:
        raise RuntimeError("Jacobian computation failed")

def main():
    urdf_path = '/home/barat/mujoco_leaphand/leap_right/robot.urdf'
    urdf_model = URDF.from_xml_file(urdf_path)

    # Hardcoded joint angles for each finger
    joint_angles = [
        [0.5235987756, 0.1, 0.2, 0.3],  # Joint angles for fingertip
        [0.2, 0.3, 0.3, 0.3],           # Joint angles for fingertip_2
        [0.2, 0.3, 0.1, 0.2],           # Joint angles for fingertip_3
        [0.1, 0.2, 0.3, 0.1]            # Joint angles for thumb_fingertip
    ]

    # Assign joint angles to each finger
    fingers = {
        'fingertip': kdl.JntArray(len(joint_angles[0])),
        'fingertip_2': kdl.JntArray(len(joint_angles[1])),
        'fingertip_3': kdl.JntArray(len(joint_angles[2])),
        'thumb_fingertip': kdl.JntArray(len(joint_angles[3]))
    }

    # Populate the JntArray for each finger with the corresponding joint angles
    for i, finger in enumerate(fingers):
        for j in range(len(joint_angles[i])):
            fingers[finger][j] = joint_angles[i][j]

    base_link = 'palm_lower'
    for finger, joint_positions in fingers.items():
        chain = create_kdl_chain_from_urdf(urdf_model, base_link, finger)
        if chain and chain.getNrOfSegments() > 0:
            print(f"Chain for {finger} created with {chain.getNrOfSegments()} segments.")
            try:
                end_effector_frame = perform_fk(chain, joint_positions)
                print(f"End effector pose for {finger}: {end_effector_frame}")

                jacobian = compute_jacobian(chain, joint_positions)
                print(f"Jacobian for {finger}:\n{jacobian}")
            except RuntimeError as e:
                print(f"Computation failed for {finger}: {e}")
        else:
            print(f"Chain for {finger} could not be created.")

if __name__ == "__main__":
    main()