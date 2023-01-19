from math import ceil, floor
from scipy.spatial import Delaunay
import open3d as o3d
import numpy as np

from plane import BoundingBox

class WearMesh():
    def __init__(self, point_cloud_object, clad_params):
        self.Center = point_cloud_object.center 
        self.Transformation = point_cloud_object.Transformation 
        self.System = point_cloud_object.System 
        self.Coordinate = point_cloud_object.Coordinate 
        self.Clad_Params = clad_params 

        # Create raw mesh from pointcloud using Delaunay method
        points = np.asarray(point_cloud_object.points, dtype=np.float32)

        self.Delaunay = Delaunay(points[:,:2])
        self.raw_vertices = points
        self.raw_BoundingBox = BoundingBox(points)

        # Create mesh object
        self.raw_mesh = o3d.t.geometry.TriangleMesh(o3d.core.Tensor(points),
                            o3d.core.Tensor(np.int64(self.Delaunay.simplices)))

        # # mesh refinement (not implemented)
        # self.refinement(refinement_factor) 

        # sliced meshes ( reconstructed meshes )
        self.sliced_meshes()

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

    def sliced_meshes(self):
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

            # FIXME: is triangulation of new mesh faces ensured by o3d?    
            # tFb = triangulateFaces(Fb)    
            # tFa = triangulateFaces(Fa) 

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
            # NOTE: can only combine legacy o3d Triangle meshes, not o3d.t
            mesh = proj_mesh + upper.to_legacy()
            mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

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

    #NOTE: if subdividing mesh into more triangles is needed
    # See http://www.open3d.org/docs/release/tutorial/geometry/mesh.html#Mesh-subdivision
    # for open3D subdivide implementations
    #def refinement(self, refinement_factor):
        #:param refinement_factor: number of mesh subdivisions
        #:type refinement_factor: integer
        # mesh_names = {'raw_mesh','reconstructed_meshes'} 
        # if refinement_factor > 1
        #     for n = 1:2
        #         for m = 1:size(this.Meshes.(mesh_names{n}),1)
        #             vertices = this.Meshes.(mesh_names{n})(m).vertices 
        #             faces = this.Meshes.(mesh_names{n})(m).faces 
        #             [new_vertices, new_faces] = subdivideMesh(vertices, faces, subdivisions) 
        #             this.Meshes.(mesh_names{n})(m).vertices = new_vertices 
        #             this.Meshes.(mesh_names{n})(m).faces = new_faces 
        #     this.RefinementFactor = refinement_factor 
        #     fprintf('A refinement of #d has been applied to the mesh', refinement_factor)


#     function self = transform(self, to_system)
#         mesh_names = {'raw_mesh','reconstructed_meshes'} 
#         if ~strcmpi(self.System, to_system) && any(strcmpi({'global', 'local'}, to_system))
#             for n = 1:2
#                 for m = 1:size(self.Meshes.(mesh_names{n}),1)
#                     vertices = self.Meshes.(mesh_names{n})(m).vertices 
#                     transf_vertices = self.Transformation.transformPointsForward(vertices) 
#                     self.Meshes.(mesh_names{n})(m).vertices = transf_vertices 
#                     self.Meshes.(mesh_names{n})(m).BoudingBox = Box(boundingBox3d(transf_vertices))    # update BoundingBox
                    
                
#             self.Center =  self.Transformation.transformPointsForward(self.Center) 
#             self.System = to_system 
#             self.Transformation.Translation = -1 * self.Transformation.Translation          # flipping translation
#             fprintf('A transformation has been applied to meshes, meshes is now in  #d system', to_system)
#         else
#             fprintf('Meshes are already in  #d system', to_system)
            
#     function self = convert(self, to_coord)
#             #convert Summary of self method goes here
#             #   Detailed explanation goes here

#         mesh_names = {'raw_mesh','reconstructed_meshes'} 
#         if ~matches(self.Coordinate, to_coord)
#                 # loop over meshes
#             for n = 1:2
#                 for m = 1:size(self.Meshes.(mesh_names{n}),1)
#                     vertices = self.Meshes.(mesh_names{n})(m).vertices 
#                         # convert from current system to the targeted system:
#                     converted_vertices = coordSystems.convert(vertices, coordSystems(self.Coordinate), coordSystems(to_coord)) 
#                     self.Meshes.(mesh_names{n})(m).vertices = converted_vertices 
#                     self.Meshes.(mesh_names{n})(m).BoudingBox = Box(boundingBox3d(converted_vertices)) 
                    
                
#                 # update meshes params
#             self.Transformation.Translation = coordSystems.convert(self.Transformation.Translation, coordSystems(self.Coordinate), coordSystems(to_coord)) 
#             self.Center =  coordSystems.convert(self.Center, coordSystems(self.Coordinate), coordSystems(to_coord)) 
#             self.Coordinate = to_coord 
#         else
#             fprintf('No convertion applied, meshes are already in  #d', to_coord) 
            
#     function out = export(self, to_system, to_coord)
#             # Example: Mesh.export('local', 'cart')
#             # check if system is different
#         if ~matches(self.System, to_system)
#             temp = transform(self, to_system) 
#                 # check if coordinate is different
#             if ~matches(self.Coordinate, to_coord)
#                 temp = convert(temp, to_coord) 
#                 out = temp.Meshes 
#             else
#                 out = temp.Meshes 
                
#         else
#             if ~matches(self.Coordinate, to_coord)
#                 temp = convert(self, to_coord) 
#                 out = temp.Meshes 
#             else
#                 out = self.Meshes
