import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import numpy as np

class LeapHandKinematics:
    def __init__(self, urdf_path):
        self.urdf_model = URDF.from_xml_file(urdf_path)
        self.base_link = 'palm_lower'

    def find_joint_for_link(self, child_link_name):
        """Find the joint in the URDF model that connects to the given child link."""
        for joint in self.urdf_model.joints:
            if joint.child == child_link_name:
                return joint
        return None

    def add_joint_to_chain(self, chain, joint):
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

    def create_kdl_chain(self, end_link):
        chain = kdl.Chain()
        current_link = end_link

        while current_link != self.base_link:
            joint = self.find_joint_for_link(current_link)
            if not joint:
                print(f"Joint for link {current_link} not found!")
                return None
            if not self.add_joint_to_chain(chain, joint):
                print(f"Failed to add joint {joint.name} to chain")
                return None
            current_link = joint.parent

        return chain

    def perform_fk(self, end_link, joint_positions):
        chain = self.create_kdl_chain(end_link)
        if not chain or chain.getNrOfSegments() == 0:
            raise RuntimeError(f"Chain for {end_link} could not be created.")

        fk_solver = kdl.ChainFkSolverPos_recursive(chain)
        end_effector_frame = kdl.Frame()
        print(f"Calculating FK for joint positions: {[joint_positions[i] for i in range(joint_positions.rows())]}")
        result = fk_solver.JntToCart(joint_positions, end_effector_frame)

        if result >= 0:
            return end_effector_frame
        else:
            raise RuntimeError("FK solver failed")

    def perform_ik(self, end_link, target_frame):
        chain = self.create_kdl_chain(end_link)
        if not chain or chain.getNrOfSegments() == 0:
            raise RuntimeError(f"Chain for {end_link} could not be created.")

        num_joints = chain.getNrOfJoints()
        joint_positions = kdl.JntArray(num_joints)
        ik_solver = kdl.ChainIkSolverPos_LMA(chain)

        initial_positions = kdl.JntArray(num_joints)  # Start with zero positions
        result = ik_solver.CartToJnt(initial_positions, target_frame, joint_positions)

        if result >= 0:
            return [joint_positions[i] for i in range(num_joints)]
        else:
            raise RuntimeError("IK solver failed")

    def compute_jacobian(self, end_link, joint_positions):
        chain = self.create_kdl_chain(end_link)
        if not chain or chain.getNrOfSegments() == 0:
            raise RuntimeError(f"Chain for {end_link} could not be created.")

        jacobian = kdl.Jacobian(chain.getNrOfJoints())
        jnt_to_jac_solver = kdl.ChainJntToJacSolver(chain)

        result = jnt_to_jac_solver.JntToJac(joint_positions, jacobian)

        if result >= 0:
            return jacobian
        else:
            raise RuntimeError("Jacobian computation failed")

# Example usage
if __name__ == "__main__":
    urdf_path = '/home/sysidea/leap_hand_mujoco/model/leap hand/robot.urdf'
    leaphand_kinematics = LeapHandKinematics(urdf_path)

    # # Hardcoded joint angles for each finger
    # joint_angles = {
    #     'fingertip': [0.5235987756, 0.1, 0.2, 0.3],
    #     'fingertip_2': [0.2, 0.3, 0.3, 0.3],
    #     'fingertip_3': [0.2, 0.3, 0.1, 0.2],
    #     'thumb_fingertip': [0.1, 0.2, 0.3, 0.1]
    # }

    # for finger, angles in joint_angles.items():
    #     joint_positions = kdl.JntArray(len(angles))
    #     for i, angle in enumerate(angles):
    #         joint_positions[i] = angle

    #     print(f"--- {finger} ---")
    #     try:
    #         # Perform FK
    #         fk_result = leaphand_kinematics.perform_fk(finger, joint_positions)
    #         print(f"End effector pose for {finger}: {fk_result}")

    #         # Compute Jacobian
    #         jacobian = leaphand_kinematics.compute_jacobian(finger, joint_positions)
    #         print(f"Jacobian for {finger}:\n{jacobian}")

    #         # Perform IK
    #         ik_result = leaphand_kinematics.perform_ik(finger, fk_result)
    #         print(f"IK joint positions for {finger}: {ik_result}")

    #     except RuntimeError as e:
    #         print(f"Computation failed for {finger}: {e}")
    ik_result = leaphand_kinematics.perform_ik('fingertip', kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                            kdl.Vector(0, 0, 0)))
    print(f"IK joint positions for {'fingertip'}: {ik_result}")