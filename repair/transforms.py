# This module contains most of the 3D transformation logic necessary for the repair program.
import numpy as np
from skspatial.objects import Plane

''' 
Unwrapped Cylindrical Coordinate conversion functions, see:
https://link.springer.com/article/10.1007/s00170-022-09933-3/figures/4
'''
def cart_to_unwrapped(xyz):
    x = xyz[:,0]
    y = xyz[:,1]

    # Calculate r and theta arrays
    theta = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)

    # Return unwrapped cylindrical coordinates: (r_mean*theta, z, r)
    return np.column_stack((np.mean(r)*theta, xyz[:,2], r))

def unwrapped_to_cart(coords):
    r = coords[:,2]
    theta = coords[:,0] / np.mean(r)

    # Calculate x and y arrays
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # return cartesian coords
    return np.column_stack((x,y,coords[:,1]))

# Returns 4x4 matrix representing a rotation about the z-axis
def rotation_around_z(angle):
    cos = np.cos(angle)
    sin = np.sin(angle)
    # Apply rotation
    rot = np.array((
                (cos,	-sin,	0,  0),
                (sin,   cos,    0,  0),
                (0, 0,  1,  0),
                (0,	0,	0,	1)
            ), dtype=np.float64)
    return rot

'''
Represents the 3D cartesian bounding box of a set of points.
Allows you to return different faces of the bounding box as Plane objects.
'''
class BoundingBox():
    def __init__(self, points):
        if type(points) is np.ndarray:
            self.update_points(points)
        else:
            print("Error: BoundingBox init failed, points not ndarray")
    
    def update_points(self, points):
        # Compute the min/max for each axis (x,y,z)
        self.min = np.ndarray.min(points, 0)
        self.max = np.ndarray.max(points, 0)

        # Extent
        self.extent = self.max - self.min

        # Center of bounding box
        self.center = self.min + self.extent / 2

        # Box vertices
        vertices = [np.ndarray.copy(self.min), np.ndarray.copy(self.max)]
        for i in range(3):
            # Add each extent to each min for a vertex
            min = np.ndarray.copy(self.min)
            min[i] += self.extent[i]
            vertices.append(min)

            # Subtract each extent from each max for a vertex
            max = np.ndarray.copy(self.max)
            max[i] -= self.extent[i]
            vertices.append(max)
        self.vertices = np.array(vertices)

    #FIXME: refactor using numpy array masking and dict keys
    def get_face_coords(self, face):
        if face == "bottom":
            # 4 coords at lowest z value
            min = self.min[2]
            plane = []
            for v in self.vertices:
                if v[2] == min:
                    plane.append(v)
            return np.array(plane)

        elif face == "top":
            # 4 coords at highest z value
            max = self.max[2]
            plane = []
            for v in self.vertices:
                if v[2] == max:
                    plane.append(v)
            return np.array(plane)
        
        elif face == "front":
            # 4 coords at highest z value
            max = self.min[1]
            plane = []
            for v in self.vertices:
                if v[1] == max:
                    plane.append(v)
            return np.array(plane)
        
        elif face == "back":
            # 4 coords at lowest z value
            min = self.max[1]
            plane = []
            for v in self.vertices:
                if v[1] == min:
                    plane.append(v)
            return np.array(plane)

    def get_plane(self, face):
        if face == "bottom":
            return Plane(point=list(self.min), normal=[0,0,-1])

        elif face == "top":
            return Plane(point=list(self.max), normal=[0,0,1])

    def plane_center(self, face):
        face_coords = self.get_face_coords(face)
        return np.mean(face_coords, axis=0)

    def translate(self, t):
        # Apply translation and update points
        points = self.vertices + t
        return BoundingBox(points)
