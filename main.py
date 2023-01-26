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

        # add mesh plot
        meshMenu = mainMenu.addMenu('Mesh')
        self.add_sphere_action = QtWidgets.QAction('Add Wear', self)
        self.add_sphere_action.triggered.connect(self.plot)
        meshMenu.addAction(self.add_sphere_action)

        if show:
            self.show()

    def plot(self):
        print("Processing")
        start = time.time()
        # get path and normal data
        # paths, normals = self.wear() # WEAR PROGRAM
        paths, normals = self.load_test_data() # SAVED CSV DATA
        end = time.time()
        print(end-start, "seconds")

        # Visualize data
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
                    spline.scale([1,1,10],inplace=True)

                    # plot normal arrow at each point in opposite color
                    scale = 2
                    spline["vectors"] = normals[i].get(j,q) * scale
                    spline.set_active_vectors("vectors")

                    self.plotter.add_mesh(spline.arrows, lighting=False, color=1-color)
                    self.plotter.add_mesh(spline, smooth_shading=True, color=color, line_width=10)

        self.plotter.reset_camera()

    def load_test_data(self):
        import csv
        from multicell import MultiCell

        f = "test_output.csv"
        paths = []
        normals = []
        with open(f, 'r') as csvfile:
            reader = csv.reader(csvfile)

            p = []
            n = []
            path_cell = MultiCell()
            norm_cell = MultiCell()
            path_count = 0
            group = 0
            layer = 0
            for idx, row in enumerate(reader):
                if idx != 0: #ignore header row
                    #check for layer change
                    if layer != int(float(row[0])):
                        # store previous layer cell
                        paths.append(path_cell)
                        normals.append(norm_cell)

                        # reset cells to empty
                        path_cell = MultiCell()
                        norm_cell = MultiCell()
                        p = []
                        n = []
                        path_count = 0
                    
                    # check for group change
                    if group != int(float(row[1])):
                        path_count = 0
                        p = []
                        n = []

                    group = int(float(row[1]))

                    p.append(list(float(i) for i in row[4:7]))
                    n.append(list(float(i) for i in row[7:]))

                    # check if its last point in path
                    if int(float(row[2])) == 1:
                        path_cell.set(np.array(p), group, path_count)
                        norm_cell.set(np.array(n), group, path_count)

                        # reset points/norms
                        p = []
                        n = []
                        path_count += 1
                    
                    group = int(float(row[1]))
                    layer = int(float(row[0]))

        return paths, normals

    def wear(self):
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

        return model.Paths.Paths, model.Paths.Normals

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyMainWindow()
    sys.exit(app.exec_())