import numpy as np
from enum import Enum

class CartNorms(Enum):
    TOP = [0,0,1]
    BOTTOM = [0,0,-1]

class Plane():
    # Point and Normal are default Lists
    # Corners is a numpy array
    def __init__(self, normal, point=None, corners=None):
        self.corners = corners
        self.normal = normal
        if point is None:
            self.point = list(corners[0])
        else:
            self.point = point

class BoundingBox():
    def __init__(self, points):
        if type(points) is np.ndarray:
            self.update_points(points)
    
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

    def get_face_coords(self, face):
        if face == "bottom":
            # 4 coords at lowest z value
            min_z = self.min[2]
            plane = []
            for v in self.vertices:
                if v[2] == min_z:
                    plane.append(v)
            return Plane(CartNorms.BOTTOM, corners=np.array(plane))

        elif face == "top":
            # 4 coords at highest z value
            max_z = self.max[2]
            plane = []
            for v in self.vertices:
                if v[2] == max_z:
                    plane.append(v)
            return Plane(CartNorms.TOP, corners=np.array(plane))

    def get_plane(self, face):
        if face == "bottom":
            return Plane(CartNorms.BOTTOM, point=list(self.min))

        elif face == "top":
            return Plane(CartNorms.TOP, point=list(self.max))


    def translate(self, t):
        # Apply translation and update points
        points = self.vertices + t
        return BoundingBox(points)