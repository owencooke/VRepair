import numpy as np

class RigidTransform():
    def __init__(self, rot=None, trans=None):
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

if __name__ == "__main__":
    x = RigidTransform()
    print(x.mat[:3,3])