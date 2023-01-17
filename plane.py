import numpy as np
from enum import Enum
from skspatial.objects import Plane

class CartNorms(Enum):
    TOP = [0,0,1]
    BOTTOM = [0,0,-1]

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
            # return Plane(point=list(self.min), normal=CartNorms.BOTTOM)
            return Plane(point=list(self.min), normal=[0,0,-1])

        elif face == "top":
            # return Plane(point=list(self.max), normal=CartNorms.TOP)
            return Plane(point=list(self.max), normal=[0,0,1])

    def plane_center(self, face):
        face_coords = self.get_face_coords(face)
        return np.mean(face_coords, axis=0)

    def translate(self, t):
        # Apply translation and update points
        points = self.vertices + t
        return BoundingBox(points)
