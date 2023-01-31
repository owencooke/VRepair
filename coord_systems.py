import numpy as np
from enum import Enum

class Coords(Enum):
    CART = 0
    CYL = 1
    SPH = 2

#FIXME: add support for spherical coords
def convert_coords(points, init, final):
    if init == final:
        return points

    if init == Coords.CART:
        if final == Coords.CYL:
            points = cart_to_cyl(points)
        # elif final == Coords.SPH:
        #     points = cart_to_sph(points)
    elif init == Coords.CYL:
        if final == Coords.CART:
            points = cyl_to_cart(points)
    return points

def cyl_to_cart(points):
    # if len(points.shape) == 1:
    #     # Single point
    #     r = points[0]
    #     theta = points[1]
    # else:
    #     r = points[:,0]
    #     theta = points[:,1]

    # # Calculate x and y arrays
    # x = r * np.cos(theta)
    # y = r * np.sin(theta)

    # # Multiply by conversion factors and return
    # if len(points.shape) == 1:
    #     return np.array((x, y, points[2]))
    # else:
    #     return np.column_stack((x, y, points[:,2]))

    #NOTE: discrepancy between cylindrical format order (theta, z, r in FCD.pcd)
    if len(points.shape) == 1:
        # Single point
        r = points[2]
        theta = points[0]
    else:
        r = points[:,2]
        theta = points[:,0]

    # Calculate x and y arrays
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Multiply by conversion factors and return
    if len(points.shape) == 1:
        return np.array((x, y, points[1]))
    else:
        return np.column_stack((x, y, points[:,1]))

def cart_to_cyl(points):
    x = points[:,0]
    y = points[:,1]

    # Calculate r and theta arrays
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y,x)

    # Multiply by conversion factors and return
    return np.column_stack((r, theta, points[:,2]))

if __name__ == "__main__":
    # points = np.array(((3,4,5),(7,8,2),))
    # print(points)
    # n = cart_to_cyl(points)
    # print(n)
    # print(cyl_to_cart(n))
    cyl_to_cart(np.array((148,20,3)))