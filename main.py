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
from qtpy.QtCore import QThreadPool, QRunnable, Signal, Slot, QObject, Qt

from clad_params import CladParams
from wear_mesh import WearMesh
from wear_path import WearPath
from coord_systems import cyl_to_cart

CLADDING_DEFAULTS = {
    "Clad Height"           : 1,        # (mm) 
    "Clad Width"            : 10,       # (mm) 
    "Max Clad Step"         : 5,        # (mm)  (clads overlap is considered)
    "Clad Orientation"      : 0,        # degrees
    "Clad Cross Hatching"   : 0,        # enable or disable
    "Layer Offset"          : 0,        # (0-1)
    "Max Laser Defocus"     : 3
}

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
        self.setWindowTitle("VRepair (SMART LAB) BETA")

        # Init UI
        self.frame, self.plotter = self.get_3D_frame()
        self.setCentralWidget(self.frame)
        self.createMainDock()
        self.createCartesianDock()
        self.createMenus()
        # self.cartDock.setMinimumHeight(self.height() - self.mainDock.height() - 300)

        # Status Bar
        self.progressBar = QtWidgets.QProgressBar()
        self.statusBar().addPermanentWidget(self.progressBar)

        # create threadpool for long processes
        self.threadpool = QThreadPool()
        self.threadpool.setExpiryTimeout(1)

        if show:
            self.show()

    # Pass functions to run in thread/after thread
    def thread(self, start, after=None):
        thread = Thread(start)
        thread.emitter.finished.connect(after)
        self.threadpool.start(thread)
    
    def update_status(self, str):
        self.statusBar().showMessage(str)

    @Slot(int, int)
    def update_progress(self, layer, total):
        self.progressBar.setValue(100 * layer/total)

    def createMenus(self):
        mainMenu = self.menuBar()

        # FILE
        fileMenu = mainMenu.addMenu('File')
        
        loadButton = QtWidgets.QAction('Load PointCloud', self)
        loadButton.triggered.connect(self.load_cloud)
        fileMenu.addAction(loadButton)

        csvButton = QtWidgets.QAction('Load Paths From CSV', self)
        csvButton.triggered.connect(self.csv_load)
        fileMenu.addAction(csvButton)

        exportButton = QtWidgets.QAction('Export Paths to CSV', self)
        exportButton.triggered.connect(self.export_paths)
        fileMenu.addAction(exportButton)

        exitButton = QtWidgets.QAction('Exit', self)
        exitButton.setShortcut('Ctrl+Q')
        exitButton.triggered.connect(self.close)
        fileMenu.addAction(exitButton)

        # VIEW
        viewMenu = mainMenu.addMenu("View")

        resetButton = QtWidgets.QAction('Reset Camera', self)
        resetButton.setShortcut('Ctrl+R')
        resetButton.triggered.connect(self.plotter.reset_camera)
        viewMenu.addAction(resetButton)

        restoreMainDock = self.mainDock.toggleViewAction()
        restoreMainDock.setText("Show " + restoreMainDock.text())
        viewMenu.addAction(restoreMainDock)

        restoreCartDock = self.cartDock.toggleViewAction()
        restoreCartDock.setText("Show " + restoreCartDock.text())
        viewMenu.addAction(restoreCartDock)

    def get_3D_frame(self):
        # create the frame
        frame = QtWidgets.QFrame()
        vlayout = QtWidgets.QVBoxLayout()

        # add the pyvista interactor object
        plotter = QtInteractor(frame)
        plotter.add_axes()
        vlayout.addWidget(plotter.interactor)
        self.signal_close.connect(plotter.close)
        frame.setLayout(vlayout)

        return frame, plotter

    def toggleGroup(self, ctrl):
        state = ctrl.isChecked()
        if state:
            ctrl.setFixedHeight(ctrl.sizeHint().height())
        else:
            ctrl.setFixedHeight(30)

    def createCladBtnGroup(self):
        # create cladding buttons
        layout = QtWidgets.QFormLayout()
        for name, value in CLADDING_DEFAULTS.items():
            field = QtWidgets.QDoubleSpinBox()
            field.setValue(value)
            layout.addRow(name, field)
    
        group = QtWidgets.QGroupBox("Show Cladding Parameters")
        group.setCheckable(True)
        group.toggled.connect(lambda: self.toggleGroup(group))
        group.setLayout(layout)
        return group
    
    def createRepairBtnGroup(self):
        # create buttons
        triangulate = QtWidgets.QPushButton('Triangulate Mesh')
        triangulate.clicked.connect(self.load_raw_mesh)
        triangulate.setEnabled(False)

        slice = QtWidgets.QPushButton('Slice Mesh')
        slice.clicked.connect(lambda: self.thread(self.load_slices, self.plot_slices))
        slice.setEnabled(False)

        genPaths = QtWidgets.QPushButton('Generate Toolpaths')
        genPaths.clicked.connect(lambda: self.thread(self.generate_paths, self.plot_paths_normals))
        genPaths.setEnabled(False)

        # add buttons to layout
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(triangulate)
        layout.addWidget(slice)
        layout.addWidget(genPaths)
        layout.addStretch(1)

        self.meshBtn = triangulate
        self.sliceBtn = slice
        self.genPathBtn = genPaths

        group = QtWidgets.QGroupBox("Repair")
        group.setLayout(layout)
        return group

    def createMainDock(self):
        # create groups to be contained in the dock
        self.cladButtons = self.createCladBtnGroup()
        self.repairButtons = self.createRepairBtnGroup()

        # add groups to a layout
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.cladButtons)
        layout.addWidget(self.repairButtons)

        # create docked window
        self.mainDock = QtWidgets.QDockWidget(parent=self, flags=Qt.WindowFlags())
        self.mainDock.setWindowTitle("Path Planning Controls")
        w = QtWidgets.QWidget()
        w.setLayout(layout)
        self.mainDock.setWidget(w)

        # add dock to main window
        self.addDockWidget(Qt.LeftDockWidgetArea, self.mainDock)

    def createCartesianDock(self):
        self.cartDock = QtWidgets.QDockWidget(parent=self, flags=Qt.WindowFlags())
        self.cartDock.setWindowTitle("Cartesian Viewer")
        self.addDockWidget(Qt.LeftDockWidgetArea, self.cartDock)

        # Add 3d frame to Layout
        frame, self.cart_plotter = self.get_3D_frame()
        layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.TopToBottom)
        layout.addWidget(frame)

        # add layout to dock
        w = QtWidgets.QWidget()
        self.cartDock.setWidget(w)
        w.setLayout(layout)

    def reset_viewer(self):
        self.plotter.reset_camera()
        self.plotter.set_scale(zscale=10)
        self.plotter.clear()

    def load_cloud(self):
        self.sliceBtn.setEnabled(False)
        self.genPathBtn.setEnabled(False)
        
        fin, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Point Cloud", filter="3D Files (*.pcd *.ply)")
        if fin != "":
            cloud = o3d.io.read_point_cloud(fin)
            points = np.asarray(cloud.points)

            # plot PC in main cylindrical viewer
            self.reset_viewer()
            self.cloud = pv.PolyData(points)
            self.plotter.add_points(self.cloud.elevation(), cmap="jet")

            # plot PC in cartesian mini viewer
            self.cart_plotter.clear()
            cloud = pv.PolyData(cyl_to_cart(points))
            self.cart_plotter.add_points(cloud.elevation(), cmap="jet")

            self.update_status('Loaded point cloud from: ' + fin)
            self.meshBtn.setEnabled(True)

    def get_clad_params(self):
        fields = self.cladButtons.findChildren(QtWidgets.QDoubleSpinBox)
        params = list(widget.value() for widget in fields)
        return CladParams(*params)

    def load_raw_mesh(self):
        self.genPathBtn.setEnabled(False)
        self.reset_viewer()
        self.Mesh = WearMesh(self.cloud.points, self.get_clad_params())
        self.raw_mesh = pv.PolyData(*self.Mesh.get_raw())
        self.plotter.add_mesh(self.raw_mesh.elevation(), cmap="jet", show_edges=True)
        self.update_status('Generated mesh with ' + str(len(self.Mesh.Delaunay.simplices)) + ' triangles.')
        self.sliceBtn.setEnabled(True)

    def load_slices(self):
        self.update_status('Slicing mesh...')
        self.Mesh.emitter.progress.connect(self.update_progress)
        self.Mesh.generate_slices()
        self.update_status(str(self.Mesh.layers) + ' layers generated.')
        self.genPathBtn.setEnabled(True)
    
    @Slot()
    def plot_slices(self):
        self.reset_viewer()
        # display each layer
        for i in range(self.Mesh.layers):
            self.slice_mesh = pv.PolyData(*self.Mesh.get_slice_layer(i))
            self.plotter.add_mesh(self.slice_mesh.elevation(), color=np.random.rand(3,), show_edges=True)

    def generate_paths(self):
        # generate paths
        self.Paths = WearPath(self.Mesh, self.Mesh.Clad_Params)
        self.Paths.emitter.progress.connect(self.update_progress)
        self.update_status('Generating clippers...')
        self.Paths.generate_clippers()
        self.update_status('Generating paths...')
        self.Paths.generate_paths_and_normals()
        self.paths, _, self.normals = self.Paths.export()
        self.update_status('Path planning complete.')

    @Slot()
    def plot_paths_normals(self):
        self.reset_viewer()
        for i in range(len(self.paths)):
            # Random color for each layer
            color = np.random.rand(3,)
            for j in range(len(self.paths[i])):
                for q in range(self.paths[i].len_subset(j)):
                    points = self.paths[i].get(j,q)

                    # Create spline to represent path
                    spline = pv.PolyData(points)
                    the_cell = np.arange(0, len(points), dtype=np.int_)
                    the_cell = np.insert(the_cell, 0, len(points))
                    spline.lines = the_cell

                    # Assign normal vectors to spline
                    scale = 1/3
                    spline["vectors"] = self.normals[i].get(j,q) * scale
                    spline.set_active_vectors("vectors")

                    # plot paths and normals in opposite colors
                    self.plotter.add_mesh(spline.arrows, color=1-color, line_width=5)
                    self.plotter.add_mesh(spline, smooth_shading=True, color=color, line_width=5)

        self.progressBar.reset()
    
    def export_paths(self):
        fout, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save File', filter="CSV (*.csv)")
        if fout != "":
            self.Paths.csv_export(fout)
            self.update_status('Paths exported as: ' + fout)

    def csv_load(self):
        fin, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Path Data", filter="CSV (*.csv)")
        if fin != "":
            self.paths, self.normals = csv_import(fin)
            self.plot_paths_normals()
            self.update_status('Loaded paths from: ' + fin)

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