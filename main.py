import csv
import sys
import time
import numpy as np
import open3d as o3d
import pyvista as pv
from pyvistaqt import QtInteractor, MainWindow

# Setting the Qt bindings for QtPy
import os
os.environ["QT_API"] = "pyside6"

from qtpy import QtWidgets
from qtpy.QtCore import QThreadPool, QRunnable, Signal, Slot, QObject

from clad_params import CladParams
from wear_mesh import WearMesh
from wear_path import WearPath

''' This class runs functions in a separate Python thread than the one
running the main GUI, so that the GUI does not freeze. '''
class MyEmitter(QObject):
    # setting up custom signal
    finished = Signal()

class Thread(QRunnable):
    def __init__(self, function, *args, **kwargs):
        super(Thread, self).__init__()
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.emitter = MyEmitter()

    def run(self):
        self.function(*self.args, **self.kwargs)
        self.emitter.finished.emit()


''' Qt Widget Application '''
class MyMainWindow(MainWindow):
    def __init__(self, parent=None, show=True):
        QtWidgets.QMainWindow.__init__(self, parent)

        # create the frame
        self.frame = QtWidgets.QFrame()
        vlayout = QtWidgets.QVBoxLayout()

        # add the pyvista interactor object
        self.plotter = QtInteractor(self.frame)
        self.plotter.add_axes()
        vlayout.addWidget(self.plotter.interactor)
        self.signal_close.connect(self.plotter.close)

        self.frame.setLayout(vlayout)
        self.setCentralWidget(self.frame)

        # simple menu to demo functions
        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('File')
        meshMenu = mainMenu.addMenu('Repair')
    
        # Status Bar:
        self.statusBar().showMessage('Ready')
        self.progressBar = QtWidgets.QProgressBar()
        self.statusBar().addPermanentWidget(self.progressBar)
        # This is simply to show the bar
        # self.progressBar.setGeometry(30, 40, 200, 25)
        # self.progressBar.setValue(50)

        # point cloud load button
        loadButton = QtWidgets.QAction('Load PointCloud', self)
        loadButton.triggered.connect(self.load_cloud)
        fileMenu.addAction(loadButton)

        csvButton = QtWidgets.QAction('Load From CSV', self)
        csvButton.triggered.connect(self.load_test_data)
        fileMenu.addAction(csvButton)

        # exit app button
        exitButton = QtWidgets.QAction('Exit', self)
        exitButton.setShortcut('Ctrl+Q')
        exitButton.triggered.connect(self.close)
        fileMenu.addAction(exitButton)

        # mesh gen button
        triangleButton = QtWidgets.QAction('Triangulate Mesh', self)
        triangleButton.triggered.connect(self.load_raw_mesh)
        meshMenu.addAction(triangleButton)

        # mesh slice button
        sliceButton = QtWidgets.QAction('Slice Mesh', self)
        sliceButton.triggered.connect(self.load_slice_mesh)
        meshMenu.addAction(sliceButton)

        # path generation button
        pathButton = QtWidgets.QAction('Generate Toolpaths', self)
        pathButton.triggered.connect(self.thread)
        meshMenu.addAction(pathButton)

        # export path to CSV button
        exportButton = QtWidgets.QAction('Export Toolpaths to CSV', self)
        exportButton.triggered.connect(self.export_paths)
        meshMenu.addAction(exportButton)

        # threading for long path generation
        self.threadpool = QThreadPool()
        self.threadpool.setExpiryTimeout(1)

        if show:
            self.show()

    def reset_viewer(self):
        self.plotter.reset_camera()
        self.plotter.set_scale(zscale=10)
        self.plotter.clear()

    def load_cloud(self):
        # init cladding parameters
        self.Clad_Params = CladParams(1,                     # clad height (mm)
                                10,                     # clad width  (mm)
                                5,                     # max clad step (mm)  (clads overlap is considered)
                                0,                     # clad orientation (degrees)
                                0,                # clad cross hatching ( enable or disable )
                                0,                     # layer offset (e.g. 0.5) (0-1)
                                3                     # max laser defocus (mm)
        )
        self.reset_viewer()
        # filename = "FBH.pcd"
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Point Cloud", filter="3D Files (*.pcd *.ply)")
        if filename != "":
            # init wear class with pointcloud file
            cloud = o3d.io.read_point_cloud(filename)
            self.cloud = pv.PolyData(np.asarray(cloud.points))
            self.plotter.add_points(self.cloud.elevation(), cmap="jet")

    def load_raw_mesh(self):
        self.reset_viewer()
        self.Mesh = WearMesh(self.cloud.points, self.Clad_Params)
        self.raw_mesh = pv.PolyData(*self.Mesh.get_raw())
        self.plotter.add_mesh(self.raw_mesh.elevation(), cmap="jet", show_edges=True)

    def load_slice_mesh(self):
        self.reset_viewer()
        self.Mesh.generate_slices()
        # display each layer
        for i in range(self.Mesh.layers):
            self.slice_mesh = pv.PolyData(*self.Mesh.get_slice_layer(i))
            self.plotter.add_mesh(self.slice_mesh.elevation(), color=np.random.rand(3,), show_edges=True)

    def thread(self):
        thread = Thread(self.generate_paths)
        thread.emitter.finished.connect(self.plot_paths_normals)
        self.threadpool.start(thread)

    def generate_paths(self):
        print("Processing")
        start = time.time()
        # generate paths
        self.Paths = WearPath(self.Mesh, self.Mesh.Clad_Params)  
        end = time.time()
        print(end-start, "seconds")

    @Slot()
    def plot_paths_normals(self):
        self.reset_viewer()
        # Visualize data
        paths, _, normals = self.Paths.export()
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
                    # spline.scale([1,1,10],inplace=True)

                    # plot normal arrow at each point in opposite color
                    scale = 1/3
                    spline["vectors"] = normals[i].get(j,q) * scale
                    spline.set_active_vectors("vectors")

                    self.plotter.add_mesh(spline.arrows, color=1-color, line_width=5)
                    self.plotter.add_mesh(spline, smooth_shading=True, color=color, line_width=5)
    
    def export_paths(self):
        out = "test_output.csv"
        out, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save File', filter="CSV (*.csv)")
        if out != "":
            self.Paths.csv_export(out)

    def load_test_data(self):
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

        self.reset_viewer()
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
                    # spline.scale([1,1,10],inplace=True)

                    # plot normal arrow at each point in opposite color
                    scale = 1/3
                    spline["vectors"] = normals[i].get(j,q) * scale
                    spline.set_active_vectors("vectors")

                    self.plotter.add_mesh(spline.arrows, color=1-color, line_width=5)
                    self.plotter.add_mesh(spline, smooth_shading=True, color=color, line_width=5)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyMainWindow()
    sys.exit(app.exec_())