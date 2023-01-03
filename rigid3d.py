import numpy as np

class RigidTransform():
    def __init__(self):
        self.mat = np.array((
                    (1,	0,	0,	0),
                    (0,	1,	0,	0),
                    (0,	0,	1,	0),
                    (0,	0,	0,	1)
                ))

if __name__ == "__main__":
    x = RigidTransform()
    print(x.mat)