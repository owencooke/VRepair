import numpy as np

def cyl_to_cart(points, conv_factors=[1,1,1]):
    r = points[:,0]
    theta = points[:,1]

    # Calculate x and y arrays
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Multiply by conversion factors and return
    return np.array((x, y, points[:,2])) * np.array(conv_factors)

def cart_to_cyl(points, conv_factors=[1,1,1]):
    x = points[:,0]
    y = points[:,1]

    # Calculate r and theta arrays
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y,x)

    # Multiply by conversion factors and return
    return np.array((r, theta, points[:,2])) * np.array(conv_factors)