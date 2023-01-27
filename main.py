import csv
import sys
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

        # point cloud load button
        loadButton = QtWidgets.QAction('Load PointCloud', self)
        loadButton.triggered.connect(self.load_cloud)
        fileMenu.addAction(loadButton)

        csvButton = QtWidgets.QAction('Load From CSV', self)
        csvButton.triggered.connect(self.csv_load)
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

        # create threadpool for long processes
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
        # generate paths
        self.Paths = WearPath(self.Mesh, self.Mesh.Clad_Params)
        self.Paths.emitter.progress.connect(self.update_progress)
        self.Paths.generate_clippers()
        self.Paths.generate_paths_and_normals()
        self.paths, _, self.normals = self.Paths.export()

    @Slot(int, int)
    def update_progress(self, layer, total):
        self.progressBar.setValue(100 * layer/total)

    @Slot()
    def plot_paths_normals(self):
        self.reset_viewer()
        # Visualize data
        for i in range(len(self.paths)):
            color = np.random.rand(3,)
            for j in range(len(self.paths[i])):
                for q in range(self.paths[i].len_subset(j)):
                    # Create path line
                    points = self.paths[i].get(j,q)
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
                    spline["vectors"] = self.normals[i].get(j,q) * scale
                    spline.set_active_vectors("vectors")

                    self.plotter.add_mesh(spline.arrows, color=1-color, line_width=5)
                    self.plotter.add_mesh(spline, smooth_shading=True, color=color, line_width=5)
        self.progressBar.reset()
    
    def export_paths(self):
        fout, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save File', filter="CSV (*.csv)")
        if fout != "":
            self.Paths.csv_export(fout)

    def csv_load(self):
        fin, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Path Data", filter="CSV (*.csv)")
        if fin != "":
            self.paths, self.normals = csv_import(fin)
            self.plot_paths_normals()

#FIXME: find a better file to place function
def csv_import(fin):
    from multicell import MultiCell
    paths = []
    normals = []
    with open(fin, 'r') as csvfile:
        reader = csv.reader(csvfile)

        p = []
        n = []
        path_cell = MultiCell()
        norm_cell = MultiCell()
        path_count = group = layer = 0
        for idx, row in enumerate(reader):
            # ignore header row
            if idx != 0:
                # check for layer change
                if layer != int(float(row[0])):
                    # store previous layer cell
                    paths.append(path_cell)
                    normals.append(norm_cell)
                    # reset cells to empty
                    path_cell = MultiCell()
                    norm_cell = MultiCell()
                layer = int(float(row[0]))
                
                # check for group change
                if group != int(float(row[1])):
                    path_count = 0
                group = int(float(row[1]))

                # store point and normal
                p.append(list(float(i) for i in row[4:7]))
                n.append(list(float(i) for i in row[7:]))

                # check if its last point in path
                if int(float(row[2])) == 1:
                    # store path
                    path_cell.set(np.array(p), group, path_count)
                    norm_cell.set(np.array(n), group, path_count)
                    # reset points/norms
                    p = []
                    n = []
                    path_count += 1
    return paths, normals

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyMainWindow()
    sys.exit(app.exec_())