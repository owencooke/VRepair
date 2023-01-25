import numpy as np

# Returns 4x4 matrix representing rotation about z-axis
def rotation_around_z(angle):
    cos = np.cos(angle)
    sin = np.sin(angle)
    # Apply rotation
    rot = np.array((
                (cos,	-sin,	0),
                (sin,	cos,	0),
                (0,	    0,  	1)
            ), dtype=np.float64)
    return RigidTransform(rot=rot)

class RigidTransform():
    def __init__(self, mat=None, rot=None, trans=None):
        # Init matrix
        if type(mat) is np.ndarray:
            self.mat = mat
        else:
            self.mat = np.array((
                    (1,	0,	0,	0),
                    (0,	1,	0,	0),
                    (0,	0,	1,	0),
                    (0,	0,	0,	1)
                ), dtype=np.float64)

        # Initial rotation
        if type(rot) is np.ndarray:
            self.mat[:3,:3] = rot
        
        # Initial translation
        if type(trans) is np.ndarray:
            self.mat[:3,3] = trans

    # Get/Set 3x3 rotation matrix
    def getRotationMatrix(self):
        return self.mat[:3,:3]

    def setRotationMatrix(self, rot):
        self.mat[:3,:3] = rot
    
    # Get/Set translation vector
    def getTranslation(self):
        return self.mat[:3,3]
    
    def setTranslation(self, trans):
        self.mat[:3,3] = trans

    def flip(self):
        self.mat[:3,3] *= -1
    
    def inverse(self):
        self.mat = np.linalg.inv(self.mat)
        return self.mat

    def applyToPoints(self, points):
        for i in range(len(points)):
            # Transform point
            pt = np.append(points[i], 1)
            points[i] = self.mat.dot(pt)[:3]
        return points

if __name__ == "__main__":
    x = RigidTransform()
    print(x.mat[:3,3])