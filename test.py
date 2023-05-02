import numpy as np
import open3d as o3d
from repair.transforms import cart_to_unwrapped

''' 
Rotate simulation points and make point cloud 
'''
coordinates = np.load("output/sim_points.npy")
# Define the rotation angle in radians
theta = -np.pi / 4 -np.pi / 2
# theta = 0

# Define the rotation matrix
rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                            [np.sin(theta), np.cos(theta), 0],
                            [0, 0, 1]])

# Apply the rotation matrix to the coordinates
rotated_coordinates = np.dot(rotation_matrix, coordinates.T).T

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(rotated_coordinates)

o3d.io.write_point_cloud("output/sim.pcd", point_cloud, write_ascii=True)
