'''Essentially a struct for the cladding parameters'''
class CladParams():
    def __init__(self, clad_height, clad_width, max_clad_step, 
                clad_orientation, clad_cross_hatching):

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