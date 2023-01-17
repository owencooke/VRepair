import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.quiver

from rigid3d import RigidTransform
from wear_ptcd import WearPointCloud
from wear_mesh import WearMesh
from wear_path import WearPath

'''class:`wear_rebuild` is a program for generating toolpaths for the
repairing process of a wear using metal deposition modeling process. '''
class WearRebuild(): 
    # parameters with default values
    Transformation = RigidTransform()    # Model transformation matrix.
    # Coordinate = Coords.CYL              # Model coordinate System ('cart', 'cyl', 'sphe').
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
    def __init__(self, filename, clad_params, mesh_method,
                refinement_factor, point_cloud_coordinate):
            
        self.FILENAME = filename 

        # store cladding parameters
        self.Clad_Params = clad_params

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
        self.Paths = WearPath(self.Meshes, self.Clad_Params) 
        self.Clad_Params = self.Paths.Clad_Params   # update clad_params

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
        #         sz = size(paths{layer,1}{path,1}[:,1],1) 
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
        o3d.visualization.draw_geometries([self.Point_Cloud.pcd])

    # raw mesh display
    def display_raw_mesh(self):
        # Using Open3D 
        # o3d.visualization.draw([self.Meshes.raw_mesh])

        # Using matplotlib
        fig = plt.figure(figsize=plt.figaspect(0.5))
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        # Plot data
        vertices = self.Meshes.raw_mesh.vertex.positions.numpy()
        triangles = self.Meshes.raw_mesh.triangle.indices.numpy()
        ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles=triangles, cmap=plt.cm.Spectral)
        plt.show()


    # reconstructed meshes display
    def display_reconst_meshes(self):
        # using matplotlib
        fig = plt.figure(figsize=plt.figaspect(0.5))
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        # mesh plotting:
        vertices = self.Meshes.reconstructed_vertices
        triangles = self.Meshes.reconstructed_faces
        n = len(vertices)
        cmap = plt.cm.get_cmap("hsv", n)
        for i in range(n):
            # drawBox3d(reconst_mesh(i).BoundingBox.bounds);
            f = triangles[i]
            v = vertices[i]
            ax.plot_trisurf(v[:,0], v[:,1], v[:,2], triangles=f, color=cmap(i))
        plt.show()

    # toolpaths display
    def display_toolpaths(self):
        # using matplotlib
        # fig = plt.figure(figsize=plt.figaspect(0.5))
        # ax = fig.add_subplot(1, 1, 1, projection='3d')
        # paths plotting:
        paths = self.Paths.Paths
        n = len(paths)
        # cmap = plt.cm.get_cmap("hsv", n)
        for i in range(n):
            for j in range(len(paths[i])):
                for q in range(paths[i].len_subset(j)):
                    coords = paths[i].get(j,q)
                    # ax.plot_wireframe(coords[:,0], coords[:,1], coords[:,2], color=cmap(i))

        # normal plotting:
        # normals = self.Paths.Normals
        # for i in range(len(normals)):
        #     for j in range(len(normals[i])):
        #         for q in range(len(normals[i][j])):
        #             ax.quiver3D(paths[i][j][q][:,1],
        #                 paths[i][j][q][:,2],
        #                 paths[i][j][q][:,3],
        #                 normals[i][j][q][:,1],
        #                 normals[i][j][q][:,2],
        #                 normals[i][j][q][:,3],
        #                 0.03, colors=cmap(i))
        # plt.show()