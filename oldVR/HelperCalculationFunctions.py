import numpy as np

"""
Space to write matrix constants and helpers for common matrix calculations.
"""

#Matricies for change of basis applications
mat_y_to_z_up = np.array([[0, 0, -1, 0],
                          [-1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 0, 1]])

hand_to_Inspire = np.array([[0, -1, 0, 0],
                           [0, 0, -1, 0],
                           [1, 0, 0, 0],
                           [0, 0, 0, 1]])


#Check if matrix is invertible
def mat_update(prev_mat, mat):
    if np.linalg.det(mat) == 0:
        return prev_mat, False #matrix is not invertible
    else:
        return mat
    
#Takes the inverse of a matrix
def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret
