import sys
import time
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor, MainWindow

# Setting the Qt bindings for QtPy
import os
os.environ["QT_API"] = "pyside6"
from qtpy import QtWidgets

from wear_rebuild import WearRebuild
from clad_params import CladParams
from coord_systems import Coords

class MyMainWindow(MainWindow):

    def __init__(self, parent=None, show=True):
        QtWidgets.QMainWindow.__init__(self, parent)

        # create the frame
        self.frame = QtWidgets.QFrame()
        vlayout = QtWidgets.QVBoxLayout()

        # add the pyvista interactor object
        self.plotter = QtInteractor(self.frame)
        vlayout.addWidget(self.plotter.interactor)
        self.signal_close.connect(self.plotter.close)

        self.frame.setLayout(vlayout)
        self.setCentralWidget(self.frame)

        # simple menu to demo functions
        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('File')
        exitButton = QtWidgets.QAction('Exit', self)
        exitButton.setShortcut('Ctrl+Q')
        exitButton.triggered.connect(self.close)
        fileMenu.addAction(exitButton)

        # allow adding a sphere
        meshMenu = mainMenu.addMenu('Mesh')
        self.add_sphere_action = QtWidgets.QAction('Add Wear', self)
        self.add_sphere_action.triggered.connect(self.add_sphere)
        meshMenu.addAction(self.add_sphere_action)

        if show:
            self.show()

    def add_sphere(self):
        print("Processing")
        start = time.time()

        # Instantiate cladding parameters
        cladding = CladParams(1,                     # clad height (mm)
                                10,                     # clad width  (mm)
                                5,                     # max clad step (mm)  (clads overlap is considered)
                                0,                     # clad orientation (degrees)
                                0,                # clad cross hatching ( enable or disable )
                                0,                     # layer offset (e.g. 0.5) (0-1)
                                3                     # max laser defocus (mm)
        )

        # Run wear rebuild program
        model = WearRebuild('FBH.pcd',           # filename
                                cladding,
                                'delaunay',               # meshing algorithm ( 'delaunay', 'crust' )
                                1,                     # mesh refinement factor (int)
                                Coords.CYL)                  # point cloud cooridante
        model.run()

        end = time.time()
        print(end-start, "seconds")

        # Create paths and normals
        paths = model.Paths.Paths
        for i in range(len(paths)):
            color = np.random.rand(3,)
            for j in range(len(paths[i])):
                for q in range(paths[i].len_subset(j)):
                    # Create path line
                    points = paths[i].get(j,q)
                    # spline = pv.Spline(points, len(points))#.tube(radius=0.05)
                    spline = pv.PolyData()
                    spline.points = points
                    the_cell = np.arange(0, len(points), dtype=np.int_)
                    the_cell = np.insert(the_cell, 0, len(points))
                    spline.lines = the_cell
                    # spline = spline.tube(radius=0.5)

                    # plot normal arrow at each point in opposite color
                    vectors = model.Paths.Normals[i].get(j,q)
                    spline["vectors"] = vectors
                    spline.set_active_vectors("vectors")

                    self.plotter.add_mesh(spline.arrows, lighting=False, color=1-color)
                    self.plotter.add_mesh(spline, smooth_shading=True, color=color, line_width=5)

        self.plotter.set_scale(zscale=10)
        self.plotter.reset_camera() 

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyMainWindow()
    sys.exit(app.exec_())