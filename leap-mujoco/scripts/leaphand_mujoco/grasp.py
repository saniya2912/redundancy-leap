import numpy as np
from scipy.linalg import block_diag
import mujoco

class GraspClass:
    def __init__(self):
        self.G_matrices=[]
        self.Jh_blocks=[]
        
    def G_i(self,contact_orientation, r_theta,b):
        matrix1 = contact_orientation
        r_theta_b = np.dot(r_theta,b)  # Matrix multiplication
        
        matrix2 = np.array([np.cross(r_theta_b.flatten(), contact_orientation[:, 0].flatten()),
                            np.cross(r_theta_b.flatten(), contact_orientation[:, 1].flatten()),
                            np.cross(r_theta_b.flatten(), contact_orientation[:, 2].flatten())])
        
        return np.vstack([matrix1, matrix2])


    def G(self,n,contact_orientations, r_theta,bs):
        for i in range(n):
            G_i_matrix = self.G_i(contact_orientations[i],r_theta,bs[i])
            self.G_matrices.append(G_i_matrix)

        # Concatenate all G_i matrices horizontally to form G
        G = np.hstack(self.G_matrices)
        return G
    
    def J(self,xml_path,site_name):
        self.model=mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        jacp = np.zeros((3, self.model.nv))  # translation jacobian
        jacr = np.zeros((3, self.model.nv)) 

        site_id=self.model.site(site_name).id
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)

        return np.vstack((jacp, jacr))
    
    def Jh(self,n,contact_orientations,Rpks,Js):
        for i in range(n):
            Jh_i=np.matmul(np.matmul(contact_orientations[i].T,Rpks[i]),Js[i])
            self.Jh_blocks.append(Jh_i)
        return block_diag(*self.Jh_blocks)
    



   


