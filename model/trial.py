import PyKDL
from urdf_parser_py.urdf import URDF

def urdf_to_kdl_chain(urdf_model, base_link, tip_link):
    """
    Converts a URDF model to a KDL Chain for a specified finger.
    
    :param urdf_model: URDF model object
    :param base_link: Base link name
    :param tip_link: Tip link name
    :return: KDL Chain
    """
    chain = PyKDL.Chain()
    
    # Get dictionaries of links and joints
    link_dict = {link.name: link for link in urdf_model.links}
    joint_dict = {joint.name: joint for joint in urdf_model.joints}
    
    # Start with the base link
    current_link = base_link
    
    while current_link != tip_link:
        link = link_dict[current_link]
        parent_joint_name = link.parent_joint
        joint = joint_dict[parent_joint_name]
        
        # Create PyKDL Joint
        if joint.type == 'revolute':
            kdl_joint = PyKDL.Joint(PyKDL.Joint.RotZ)
        elif joint.type == 'prismatic':
            kdl_joint = PyKDL.Joint(PyKDL.Joint.TransZ)
        else:
            raise NotImplementedError(f"Joint type {joint.type} is not supported")
        
        # Create PyKDL Frame for transformation
        origin = joint.origin
        transform = PyKDL.Frame(
            PyKDL.Rotation.RPY(origin[3], origin[4], origin[5]),
            PyKDL.Vector(origin[0], origin[1], origin[2])
        )
        
        # Add the segment to the chain
        chain.addSegment(PyKDL.Segment(kdl_joint, transform))
        
        # Move to the next link
        current_link = link.child
        if current_link == '':
            raise ValueError("Link not found in the URDF model")
    
    return chain

# Path to the URDF file
urdf_path = '/home/sysidea/leap_hand_mujoco/model/leap hand/robot2.urdf'
urdf_model = URDF.from_xml_file(urdf_path)

# Define the base and tip links for one finger (e.g., the one starting with mcp_joint)
base_link = 'mcp_joint'
tip_link = 'fingertip'

# Get the KDL Chain
kdl_chain = urdf_to_kdl_chain(urdf_model, base_link, tip_link)

# Print the KDL Chain
print("KDL Chain:")
for i in range(kdl_chain.getNrOfSegments()):
    segment = kdl_chain.getSegment(i)
    print(f"Segment {i}:")
    print(f"  Joint: {segment.getJoint().getName()}")
    print(f"  Frame: {segment.getFrame()}")
