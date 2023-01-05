import numpy as np
import open3d as o3d

class Bounds():
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
        vertices = [self.min, self.max]
        for i in range(3):
            # Add each extent to each min for a vertex
            min = self.min
            min[i] += self.extent[i]
            vertices.append(min)

            # Subtract each extent from each max for a vertex
            max = self.max
            max[i] -= self.extent[i]
            vertices.append(max)
        self.vertices = np.array(vertices)

    def plane2D(self, identifier):
        if identifier == "bottom":
            # 4 coords at lowest z value
            min_z = self.min[2]
            plane = []
            for v in self.vertices:
                if v[2] == min_z:
                    plane.append(v)
            return np.array(plane)

    def translate(self, t):
        # Apply translation and update points
        points = self.vertices + t
        return Bounds(points)