from math import ceil, floor
import sys
from scipy.spatial import Delaunay
import open3d as o3d
import numpy as np
from plane import Bounds

class WearMesh():
    RefinementFactor = 1

    def __init__(self, point_cloud_object, method, refinement_factor, clad_params):
        #:param point_cloud: wear point cloud object
        #:param method: algorithm for meshing ('delaunay', 'crust')
        #:param coordinate: coordinate system (none, cart, cyl, sphe)
        #:param system: global or local
        #:param refinement_factor: number of raw mesh subdivisions
        self.Center = point_cloud_object.center 
        self.Transformation = point_cloud_object.Transformation 
        self.MeshingMethod = method 
        self.System = point_cloud_object.System 
        self.Coordinate = point_cloud_object.Coordinate 
        self.RefinementFactor = refinement_factor 
        self.Clad_Params = clad_params 

        # wear point cloud to mesh ( raw mesh )
        points = np.asarray(point_cloud_object.points)
        if method == 'delaunay':
            self.raw_faces = Delaunay(points[:,:2])   # meshing using MATLAB built-in function
            self.raw_vertices = points 
            self.raw_BoundingBox = Bounds(points)
            #FIXME: the bounding box values compared to MATLAB are slightly diff.
            # print(self.raw_BoundingBox)
            # print("min:", np.ndarray.min(points, 0), "max:", np.ndarray.max(points, 0))

        #NOTE: add crust implementation if needed
            # case 'crust'
            #     [~, F] = evalc('MyCrustOpen(points)')      # meshing using the Open Crust library.
            #     self.Meshes.raw_mesh.faces = F 
            #     self.Meshes.raw_mesh.vertices = points 
            #     self.Meshes.raw_mesh.BoundingBox = Box(boundingBox3d(points)) 
        else:
            sys.exit('Triangulation algorithm not implemented. Check function description.')
            
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

        cal_layer_step, nlayers, offset = self.calc_layer_step(edge, self.Clad_Params.CLAD_HEIGHT) 
        # print(cal_layer_step, nlayers, offset) # matches with matlab
        #FIXME:
        # mesh slicers initialization
        # Get bottom of bounding box
        plane1 = self.raw_BoundingBox.plane2D("bottom")
        # Second plane is the first, but translated upwards by the layer step
        plane2 = np.ndarray.copy(plane1)
        for v in plane2:
            v[2] += cal_layer_step

        # Create initial box
        init_box = Bounds(np.concatenate((plane2, plane1)))
        # remaining boxes
        boxes = [init_box]
        for layer in range(1,nlayers+1):
            # Apply translation
            trans = np.array([0,0,(layer) * cal_layer_step])
            b = init_box.translate(trans)
            # Append new translated box
            boxes.append(b) 
        print('ok') 
        #     # sliced meshes generations
        # rMeshes = sliceMesh(raw_mesh, Boxes, nlayers) 
        # for layer = 1: nlayers
        #     self.Meshes.reconstructed_meshes(layer,1).vertices = rMeshes{layer,1}.vertices 
        #     self.Meshes.reconstructed_meshes(layer,1).faces = rMeshes{layer,1}.faces 
        #     self.Meshes.reconstructed_meshes(layer,1).BoundingBox = Boxes(layer,1) 
            

        # self.Clad_Params.CAL_LAYER_STEP = cal_layer_step 
        # self.Clad_Params.LAYERS = nlayers 

    #NOTE: must implement if subdividing mesh into more triangles is needed
    # See http://www.open3d.org/docs/release/tutorial/geometry/mesh.html#Mesh-subdivision
    # for open3D subdivide implementations
    #def refinement(self, refinement_factor):
        #:param refinement_factor: number of mesh subdivisions
        #:type refinement_factor: integer
        # mesh_names = {'raw_mesh','reconstructed_meshes'};
        # if refinement_factor > 1
        #     for n = 1:2
        #         for m = 1:size(this.Meshes.(mesh_names{n}),1)
        #             vertices = this.Meshes.(mesh_names{n})(m).vertices;
        #             faces = this.Meshes.(mesh_names{n})(m).faces;
        #             [new_vertices, new_faces] = subdivideMesh(vertices, faces, subdivisions);
        #             this.Meshes.(mesh_names{n})(m).vertices = new_vertices;
        #             this.Meshes.(mesh_names{n})(m).faces = new_faces;
        #     this.RefinementFactor = refinement_factor;
        #     fprintf('A refinement of %d has been applied to the mesh', refinement_factor)

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
