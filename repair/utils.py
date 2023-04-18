# This module contains misc. Pythonic versions of the original MATLAB program
import numpy as np

CLADDING_DEFAULTS = {
    "Clad Height"           : 1,        # (mm) 
    "Clad Width"            : 10,       # (mm) 
    "Max Clad Step"         : 5,        # (mm)  (clads overlap is considered)
    "Clad Orientation"      : 0,        # degrees
    "Clad Cross Hatching"   : 0,        # enable or disable
    "Layer Offset"          : 0,        # (0-1)
    "Max Laser Defocus"     : 3
}

'''
Class interface for the cladding parameters.
'''
class CladParams():
    def __init__(self, clad_height, clad_width, max_clad_step, clad_orientation,
                    clad_cross_hatching, layer_offset, max_laser_defocus):

        self.CLAD_HEIGHT = clad_height 
        self.CLAD_WIDTH = clad_width 
        self.MAX_CLAD_STEP = max_clad_step 
        self.CAL_LAYER_STEP = None
        self.LAYERS = 0

        #  check if vars are as expected
        if clad_orientation>= 0 and clad_orientation <= 180 and ( clad_cross_hatching == 0 or clad_cross_hatching ==1 ):
            self.CLAD_ORIENTATION = clad_orientation 
            self.CLAD_CROSS_HATCHING= clad_cross_hatching
        else:
            print(['WARNING: Clads orientation should be between 0 - 180 degrees, '
                'and 0 or 1 to activate cross paths functionality'])

        self.LAYER_OFFSET = layer_offset
        self.MAX_LASER_DEFOCUS = max_laser_defocus

'''
Class interface for a dict-of-dicts.
Used instead of MATLAB cell arrays to represent repair paths.
'''
class MultiCell():
    def __init__(self):
        self.data = {}

    def __len__(self):
        return len(self.data)

    def len_subset(self, key):
        try: 
            return len(self.data[key])
        except KeyError:
            return 0

    def __getitem__(self, key):
        if isinstance(key, tuple):
            i, j = key
            try:
                return self.data[i][j]
            except KeyError:
                return None
        else:
            return self.data[key]

    def __setitem__(self, key, value):
        if isinstance(key, tuple):
            i, j = key
            if i not in self.data:
                self.data[i] = {}
            self.data[i][j] = value
        else:
            self.data[key] = value

'''
Pythonic version of "interparc" written for MATLAB
Adapted from: https://stackoverflow.com/a/42940234
Purpose: evenly distributes `N` points along a 3D curve defined by `points`
'''
def interpcurve(N, points):
    #equally spaced in arclength
    N=np.transpose(np.linspace(0,1,N))

    #how many points will be uniformly interpolated?
    nt=N.size

    #number of points on the curve
    n=len(points)

    pt=np.zeros((nt,2))

    #Compute the chordal arclength of each segment.
    chordlen = (np.sum(np.diff(points,axis=0)**2,axis=1))**(1/2)
    #Normalize the arclengths to a unit total
    chordlen = chordlen/np.sum(chordlen)
    #cumulative arclength
    cumarc = np.append(0,np.cumsum(chordlen))

    tbins= np.digitize(N,cumarc) # bin index in which each N is in

    #catch any problems at the ends
    tbins[np.where(np.bitwise_or(tbins<=0, N<=0))] = 1
    tbins[np.where(np.bitwise_or(tbins >= n, N >= 1))] = n - 1      

    s = np.divide((N - cumarc[tbins]),chordlen[tbins-1])
    pt = points[tbins,:] + np.multiply((points[tbins,:] - points[tbins-1,:]),(np.vstack([s]*3)).T)

    return pt, tbins - 1
