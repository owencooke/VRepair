# import matplotlib
from rigid3d import RigidTransform
from wear_ptcd import WearPointCloud
from wear_mesh import WearMesh
from coord_systems import Coords
import numpy as np

'''class:`wear_rebuild` is a program for generating toolpaths for the
repairing process of a wear using metal deposition modeling process. '''
class WearRebuild(): 
    # parameters with default values
    Transformation = RigidTransform()    # Model transformation matrix.
    Coordinate = Coords.CYL              # Model coordinate System ('cart', 'cyl', 'sphe').
    System = 'global'           # Model working system ('global', 'local').
    MeshMethod = 'delaunay'          # Model meshing method (delauny, crust).
    RefinementFactor = 1             # Mesh refinement factor

    NEW_FIGURE = True            # display each data in a new window.
    MULTIPLE_FIGURES = True      # display data of muliple layers in multiple figures.
    SHOW_PCD = True              # display point cloud.
    SHOW_MESH = True             # display mesh.
    SHOW_PLANES = True           # display boudning planes.
    SHOW_SLICES = True           # display slices
    NORMALS_SWITCH = True        # display normals
    NORMALS_SCALE = 0.03         # normals scale
    PLOTS_STYLE = 'normal'       # 'normal', 'equal'

     # CONSTRUCTOR 
    def __init__(self, filename, clad_params, layer_offset, max_laser_defocus, mesh_method,
                refinement_factor, point_cloud_coordinate):
            
        self.FILENAME = filename 

        # store cladding parameters
        self.Clad_Params = clad_params

        self.LAYER_OFFSET = layer_offset 
        self.MAX_LASER_DEFOCUS = max_laser_defocus 

        self.MeshMethod = mesh_method 
        self.RefinementFactor = refinement_factor 
        self.Coordinate = point_cloud_coordinate

    # Execute repair process
    def run(self):
        # 1- point cloud object construction
        self.Point_Cloud = WearPointCloud(self.FILENAME, self.Coordinate, self.System) 
        self.System = self.Point_Cloud.System    # update model system depends on point cloud system

        # 2- meshes object construction
        self.Meshes = WearMesh(self.Point_Cloud, self.MeshMethod, self.RefinementFactor, self.Clad_Params) 
        self.Clad_Params = self.Meshes.Clad_Params   # update clad_params

        # # 3- paths object constuction
        # self.Paths = wear_path(self.Meshes, self.Clad_Params) 
        # self.Clad_Params = self.Paths.Clad_Params   # update clad_params

    # export point cloud
    def pcd_export(self,to_system, to_coordinate): 
        # new_name = convertStringsToChars( erase(pwd,'src') + "output\" + erase(self.FILENAME, {'data/','.pcd'}) + ".ply" ) 
        # pcd = self.Point_Cloud.export(to_system, to_coordinate) 
        # pcwrite(pcd, new_name,'Encoding','binary')
        # print('Point Cloud is exported as: ', new_name)
        pass

    # export meshes
    def mesh_export(self,to_system, to_coordinate):
        # meshes = self.Meshes.export(to_system, to_coordinate) 
        # mesh_names = {'raw_mesh','reconstructed_meshes'} 
        # for n = 1:2
        #     for m = 1:size(self.Meshes.Meshes.(mesh_names{n}),1)
        #         new_name = convertStringsToChars( erase(pwd,'src') + "output\" + erase(self.FILENAME, {'data/','.pcd'}) + mesh_names{n} + num2str(m) + ".stl" ) 
        #         stlwrite(new_name, meshes.(mesh_names{n})(m).faces, meshes.(mesh_names{n})(m).vertices)
        #         print('Mesh is exported as: ', new_name)
        pass
 
    # export paths
    def paths_export(self, to_system, to_coordinate):
        # output is arranged as follows
        # [layer, (starting_point, intermediate_point, end_point) , point depth, processing point, point normal]
        # depth is distance from point to top plane of bounding box.

        # conversion system goes here:
        # [paths, depths, normals] = self.Paths.export(to_system, to_coordinate) 

        # DataOut = [] 
        # for layer = 1 : size(paths,1)
        #     for path = 1 : size(paths{layer,1},1)
        #             #for q = 1: size(paths{layer,1}{path,1},1)
        #         sz = size(paths{layer,1}{path,1}(:,1),1) 
        #         if sz >= 2   # if there are more than two points ( because path can not be less than two points )
        #             code = repmat(2,sz,1)  code(1) = 0  code(end) = 1 
        #             DataOut = [DataOut  repmat(layer,sz,1), code, depths{layer,1}{path,1}{1,1}(:), paths{layer,1}{path,1}(:,:), normals{layer,1}{path,1}{1,1}(:,:)] 
        # new_name = convertStringsToChars( erase(pwd,'src') + "output\" + erase(self.FILENAME, {'data/','.pcd'}) + "_" + erase(to_coordinate , '') +".csv" ) 
        # varNames  = {'layer_number', 'point_code','point_depth', 'X','Y','Z','Nx','Ny','Nz'} 
        # T = array2table(DataOut, 'VariableNames', varNames) 
        # writetable(T, new_name)
        # print(['Paths are exported as: ', new_name])
        pass

    '''MATPLOTLIB GRAPHS'''
    # point cloud display
    def display_pcd(self):
        # pcdDisplay(self.Point_Cloud.PCD_Obj, ...
        #     self.NEW_FIGURE, ...
        #     self.SHOW_PCD, ...
        #     self.PLOTS_STYLE) 
        pass 


    # raw mesh display
    def display_raw_mesh(self):
        # meshDisplay(self.Meshes.Meshes.raw_mesh, ...
        #     self.NEW_FIGURE, ...
        #     self.SHOW_MESH, ...
        #     self.PLOTS_STYLE) 
        pass


    # reconstructed meshes display
    def display_reconst_meshes(self):
        # reconst_meshDisplay(self.Meshes.Meshes.reconstructed_meshes, ...
        #     self.NEW_FIGURE, ...
        #     self.SHOW_MESH, ...
        #     self.PLOTS_STYLE) 
        pass


    # toolpaths display
    def display_toolpaths(self):
        # toolpathsDisplay(self.Paths.Paths, ...     # paths
        #     self.Paths.Normals, ...                # normals
        #     self.NEW_FIGURE, ...
        #     self.SHOW_MESH, ...
        #     self.PLOTS_STYLE, ...
        #     self.NORMALS_SWITCH, ...
        #     self.NORMALS_SCALE)
        pass
