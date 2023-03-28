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
import qdarktheme

from clad_params import CladParams, CLADDING_DEFAULTS
from wear_mesh import WearMesh
from wear_path import WearPath
from coord_systems import cyl_to_cart

''' 
Thread runs functions in a separate Python thread than the one
running the main GUI, so that the GUI does not freeze. MyEmitter enables
a signal to be emitted upon thread finish, so that you can connect functions.

To use: 
    1. my_thread = Thread(func_to_run)
    2. (optional) my_thread.emitter.finished.connect(func_to_run_after)
    3. Pass my_thread to `start` method of a QThreadpool instance
'''
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

'''
Main QtPy Widget Application Window
'''
class VRepairWindow(MainWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setWindowTitle("VRepair (SMART LAB) BETA")
        self.createUI()

        # create threadpool for long processes
        self.threadpool = QThreadPool()
        self.threadpool.setExpiryTimeout(1)

        self.show()

    ###################################################################
    '''Initialize UI Methods'''
    ###################################################################
    def createUI(self):
        # Create widgets to display on main window
        sidebar = self.createSidebar()
        frame3D, self.plotter = self.create3DPlot()

        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(sidebar, stretch=1)
        layout.addWidget(frame3D, stretch=2)

        main = QtWidgets.QWidget()
        main.setLayout(layout)
        self.setCentralWidget(main)

        # create application menus
        self.createFileMenu()
        self.createViewMenu()

        # add permananent progress bar widget
        self.progressBar = QtWidgets.QProgressBar()
        self.statusBar().addPermanentWidget(self.progressBar)

    def createSidebar(self):
        # Create sidebar items (use self if reference required later)
        self.cladButtons = self.createCladButtons()
        repairButtons = self.createRunButtons()
        cartFrame, self.cart_plotter = self.create3DPlot()

        # vertically layout items
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.cladButtons)
        layout.addWidget(repairButtons)
        layout.addWidget(cartFrame)

        # create sidebar widget
        sidebar = QtWidgets.QWidget()
        sidebar.setLayout(layout)
        return sidebar

    def createFileMenu(self):
        menu = self.menuBar().addMenu('File')
        
        load = QtWidgets.QAction('Load PointCloud', self)
        load.triggered.connect(self.load_cloud)
        menu.addAction(load)

        loadCSV = QtWidgets.QAction('Load Paths From CSV', self)
        loadCSV.triggered.connect(self.csv_load)
        menu.addAction(loadCSV)

        export = QtWidgets.QAction('Export Paths to CSV', self)
        export.triggered.connect(self.export_paths)
        menu.addAction(export)

        quit = QtWidgets.QAction('Quit', self)
        quit.triggered.connect(self.close)
        menu.addAction(quit)

    def createViewMenu(self):
        menu = self.menuBar().addMenu("View")

        # reset main plotter camera
        resetCamera = QtWidgets.QAction('Reset Camera', self)
        resetCamera.setShortcut('Ctrl+R')
        resetCamera.triggered.connect(self.plotter.reset_camera)
        menu.addAction(resetCamera)

    def create3DPlot(self):
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

    def createCladButtons(self):
        # create cladding buttons
        layout = QtWidgets.QFormLayout()
        for name, value in CLADDING_DEFAULTS.items():
            field = QtWidgets.QDoubleSpinBox()
            field.setValue(value)
            layout.addRow(name, field)
    
        group = QtWidgets.QGroupBox("Cladding Parameters")
        group.setCheckable(True)
        group.toggled.connect(lambda: self.toggle_btn_group(group))
        group.setLayout(layout)
        return group
    
    def createRunButtons(self):
        # create buttons
        self.meshButton = QtWidgets.QPushButton('Triangulate Mesh')
        self.meshButton.clicked.connect(self.load_raw_mesh)
        self.meshButton.setEnabled(False)

        self.sliceButton = QtWidgets.QPushButton('Slice Mesh')
        self.sliceButton.clicked.connect(lambda: self.thread(self.load_slices, self.plot_slices))
        self.sliceButton.setEnabled(False)

        self.pathsButton = QtWidgets.QPushButton('Generate Toolpaths')
        self.pathsButton.clicked.connect(lambda: self.thread(self.generate_paths, self.plot_paths_normals))
        self.pathsButton.setEnabled(False)

        # layout buttons vertically
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.meshButton)
        layout.addWidget(self.sliceButton)
        layout.addWidget(self.pathsButton)

        # group buttons in box with a title
        group = QtWidgets.QGroupBox("Path Planning")
        group.setLayout(layout)
        return group
    
    ###################################################################
    '''Backend helper methods'''
    ###################################################################

    # Toggles visibility of a button group
    def toggle_btn_group(self, ctrl):
        if ctrl.isChecked():
            ctrl.setFixedHeight(ctrl.sizeHint().height())
        else:
            ctrl.setFixedHeight(15)

    # Runs a threaded function (start). Optional run function (after) on completion
    def thread(self, start, after=None):
        thread = Thread(start)
        thread.emitter.finished.connect(after)
        self.threadpool.start(thread)
    
    # Displays message in status bar
    def update_status(self, str):
        self.statusBar().showMessage(str)

    # Changes progress bar percentage
    @Slot(int, int)
    def update_progress(self, layer, total):
        self.progressBar.setValue(100 * layer/total)

    # creates CladParams struct from input values
    def get_clad_params(self):
        fields = self.cladButtons.findChildren(QtWidgets.QDoubleSpinBox)
        params = list(widget.value() for widget in fields)
        return CladParams(*params)

    #FIXME: move me
    def reset_viewer(self):
        self.plotter.reset_camera()
        self.plotter.set_scale(zscale=10)
        self.plotter.clear()

    ###################################################################
    '''Backend functions (connected to buttons)'''
    ###################################################################
    def load_cloud(self):
        self.sliceButton.setEnabled(False)
        self.pathsButton.setEnabled(False)
        
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
            self.meshButton.setEnabled(True)

    def load_raw_mesh(self):
        self.pathsButton.setEnabled(False)
        self.reset_viewer()
        self.Mesh = WearMesh(self.cloud.points, self.get_clad_params())
        self.raw_mesh = pv.PolyData(*self.Mesh.get_raw())
        self.plotter.add_mesh(self.raw_mesh.elevation(), cmap="jet", show_edges=True)
        self.update_status('Generated mesh with ' + str(len(self.Mesh.Delaunay.simplices)) + ' triangles.')
        self.sliceButton.setEnabled(True)

    # NOTE: threaded, can't update plotter from here
    def load_slices(self):
        self.update_status('Slicing mesh...')
        self.Mesh.emitter.progress.connect(self.update_progress)
        self.Mesh.generate_slices()
        self.update_status(str(self.Mesh.layers) + ' layers generated.')
        self.pathsButton.setEnabled(True)

    # NOTE: threaded, can't update plotter from here
    def generate_paths(self):
        self.Paths = WearPath(self.Mesh, self.Mesh.Clad_Params)
        self.Paths.emitter.progress.connect(self.update_progress)
        self.update_status('Generating clippers...')
        self.Paths.generate_clippers()
        self.update_status('Generating paths...')
        self.Paths.generate_paths_and_normals()
        self.paths, _, self.normals = self.Paths.export()
        self.update_status('Path planning complete.')

    def export_paths(self):
        fout, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save File', filter="CSV (*.csv)")
        if fout != "":
            self.Paths.csv_export(fout)
            self.update_status('Paths exported as: ' + fout)

    def csv_load(self):
        fin, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Path Data", filter="CSV (*.csv)")
        if fin != "":
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
            self.paths = paths
            self.normals = normals
            self.plot_paths_normals()
            self.update_status('Loaded paths from: ' + fin)
    
    ###################################################################
    '''3D plotter functions (can't be ran from threaded functions)'''
    ###################################################################
    @Slot()
    def plot_slices(self):
        self.reset_viewer()
        # display each layer
        for i in range(self.Mesh.layers):
            self.slice_mesh = pv.PolyData(*self.Mesh.get_slice_layer(i))
            self.plotter.add_mesh(self.slice_mesh.elevation(), color=np.random.rand(3,), show_edges=True)

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

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    qdarktheme.setup_theme("dark")
    window = VRepairWindow()
    sys.exit(app.exec_())