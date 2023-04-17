from math import ceil, floor
from scipy.spatial import Delaunay # import pyvista as pv
import open3d as o3d
import numpy as np
from qtpy.QtCore import QObject, Signal

from transforms import BoundingBox

'''
WearMesh class takes in 3D points from a pointcloud of a damaged 
cylindrical surface. It then triangulates the points into a mesh in cylindrical
coordinates, and slices the mesh into layers to be filled using the laser cladding.

Usage: instantiate using the points from PC and cladding params. Then call
        `generate_slices` to obtain the layered mesh. 
'''
class WearMesh(QObject):
    progress = Signal(float)

    def __init__(self, points, clad_params):
        super().__init__()
        points = np.asarray(points)
        self.Center = np.mean(points, axis=0)
        self.Clad_Params = clad_params

        # Create raw mesh from points using Delaunay method
        self.Delaunay = Delaunay(points[:,:2])
        self.num_triangles = len(self.Delaunay.simplices)
        self.raw_vertices = points
        self.raw_BoundingBox = BoundingBox(points)
        self.raw_mesh = o3d.t.geometry.TriangleMesh(o3d.core.Tensor(points),
                            o3d.core.Tensor(np.int64(self.Delaunay.simplices)))

        # NOTE: might be possible to eliminate the need for SciPy libary
        # this code uses two already required libraries (open3D and Pyvista)
        # to generate similar results to SciPy's Delaunay. Code below 
        # would replace code in the above block. 

        # cloud = pv.PolyData(points)
        # surf = cloud.reconstruct_surface()
        # faces = surf.faces.reshape((-1, 4))[:,1:]
        # self.num_triangles = len(faces)
        # points = np.asarray(surf.points, dtype=np.float32)
        # self.raw_mesh = o3d.t.geometry.TriangleMesh(o3d.core.Tensor(points),
        #                     o3d.core.Tensor(np.int64(faces)))

    def format_triangles_pv(self, triangles):
        threes = np.full(len(triangles),3)
        return np.column_stack((threes,triangles)).flatten()

    def get_raw(self):
        return self.raw_vertices, self.format_triangles_pv(self.raw_mesh.triangle.indices.numpy())

    def get_slice_layer(self, idx):
        points = self.reconstructed_vertices[idx]
        triangles = self.reconstructed_faces[idx]
        return points, self.format_triangles_pv(triangles)

    def calc_layer_step(self, edge, clad_height):
        #CALC_LAYER_STEP Calculate the layer step distance, and how many layer will fit in
        # edge. `edge` should be of type `edges3d` in the geom3d library.
        # edge: in forms of [X Y Z dX dY dZ] 
        # clad_height: build up distance

        length = edge[2]

        remainder = (length/clad_height) % 1

        # if the layer thickess of the last layer is greater than 0.5 then,
        # cout it as a layer. If it is less then, last layer is not counted
        # unless it is the only layer.
        if remainder >= 0.5:
            nlayers = ceil(length/clad_height)      # Round towards plus infinity
        else:
            nlayers = floor(length/clad_height)     # Round towards minus infinity
            # if length/maxPitch < 1.5 round up
            if nlayers == 1:
                nlayers = ceil(length/clad_height) 

        cal_layer_step = length/nlayers 

        #calculate coverage and output how to much offset to center layers if
        #coverage isnt 1.
        coverage = cal_layer_step * nlayers 
        if coverage/length < 1:              # some uncovered
            offset = (length-coverage)/2 
        else:
            offset = 0     # fully covered

        return cal_layer_step, nlayers, offset

    def generate_slices(self):
        # Create a vertical 3D edge connecting top to bottom of the bounding box
        height = self.raw_BoundingBox.extent[2]
        edge = np.array((0,0,height))

        # Determine layer step value and # of layers
        cal_layer_step, nlayers, offset = self.calc_layer_step(edge, self.Clad_Params.CLAD_HEIGHT)
    
        # mesh slicers initialization
        # Get bottom of bounding box
        bottom = self.raw_BoundingBox.get_face_coords("bottom")
        # Second plane is the first, but translated upwards by the layer step
        plane2 = np.ndarray.copy(bottom)
        for v in plane2:
            v[2] += cal_layer_step

        # Create initial box
        init_box = BoundingBox(np.concatenate((plane2, bottom)))
        # remaining boxes
        boxes = [init_box]
        for layer in range(1,nlayers):
            # Apply translation
            trans = np.array([0,0,(layer) * cal_layer_step])
            b = init_box.translate(trans)
            # Append new translated box
            boxes.append(b) 
        
        # sliced meshes generations
        vertices, faces = self.slice_mesh(boxes, nlayers)

        self.reconstructed_vertices = []
        self.reconstructed_faces = []

        # Store sliced mesh info for each layer
        for layer in range(nlayers):
            self.reconstructed_vertices.append(vertices[layer])
            self.reconstructed_faces.append(faces[layer])

        self.reconstructed_BoundingBoxes = boxes
        self.Clad_Params.CAL_LAYER_STEP = cal_layer_step 
        self.Clad_Params.LAYERS = nlayers 
        self.layers = nlayers

    def slice_mesh(self, boxes, nlayers):
        #SLICEMESH returns the slices of a mesh, sliced using plane of a box.
        # this function followes the bottom-up rebuild approach. It uses the top
        # plane of each layer box to slice the mesh.

        slice_vertices = []
        slice_faces = []
        mesh = self.raw_mesh

        for layer in range(nlayers-1):
            # slicing previous concatenated mesh using the top plane of box
            box = boxes[layer]
            top = box.get_plane("top")

            # extracting lower mesh below top plane of bounding box
            lower = mesh.clip_plane(point=list(top.point), normal=[0,0,-1])                        

            # extracturing upper mesh above top plane
            upper = mesh.clip_plane(point=list(top.point), normal=[0,0,1]) 

            # Convert lower mesh's vertices and triangles to numpy
            lower_verts = lower.vertex.positions.numpy()
            lower_tri = lower.triangle.indices.numpy()

            # layer output:
            slice_vertices.append(lower_verts)
            slice_faces.append(lower_tri)

            # sliced mesh projection:
            proj_vertices = self.project_mesh(lower_verts, top)    # returns projected mesh
            proj_mesh = o3d.geometry.TriangleMesh(
                o3d.utility.Vector3dVector(proj_vertices),
                o3d.utility.Vector3iVector(lower_tri),
            )

            # merge projected mesh and remaining mesh from slicer (upper mesh)
            # into new mesh above the plane of first layer
            # NOTE: can only combine legacy o3d Triangle meshes, not o3d.t (tensors)
            mesh = proj_mesh + upper.to_legacy()
            mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

            # send signal to GUI to update progress bar
            self.progress.emit((layer+1)/(nlayers-1))

        slice_vertices.append(mesh.vertex.positions.numpy())
        slice_faces.append(mesh.triangle.indices.numpy())
        return slice_vertices, slice_faces

    def project_mesh(self, mesh_vertices, plane):
        # Preallocate array for projected vertices
        proj = np.zeros((len(mesh_vertices), 3))
        for idx, v in enumerate(mesh_vertices):
            # Project each vertex onto plane using scikit-spatial
            proj[idx] = plane.project_point(v)
        return proj