import numpy as np

# Define the transformation matrix
transformation_matrix = np.array([[0, 0, -1, -0.0200952],
                                  [-1, 0, 0, 0.0257578],
                                  [0, 1, 0, -0.0347224],
                                  [0, 0, 0, 1]])

R=transformation_matrix[:3,:3]
x=transformation_matrix[:3,3]
# Compute the inverse of the transformation matrix
inverse_matrix = np.linalg.inv(transformation_matrix)
print(np.dot(-R.T,x))

print(np.linalg.det(inverse_matrix))
