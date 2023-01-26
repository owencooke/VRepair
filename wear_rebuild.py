import csv
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from wear_ptcd import WearPointCloud
from wear_mesh import WearMesh
from wear_path import WearPath

'''
A program for generating toolpaths for the repairing process of 
a wear using metal deposition modeling process.
'''
class WearRebuild(): 
    System = 'global'           # Model working system ('global', 'local').

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
        self.Meshes = WearMesh(self.Point_Cloud, self.Clad_Params) 
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
 
    # export paths to CSV
    def paths_export(self, to_system, to_coordinate):
        out = "test_output.csv"

        # get path values according to system/coord type
        paths, depths, normals = self.Paths.export(to_system, to_coordinate)

        # Create CSV output file
        header = ['layer_number', 'point_code','point_depth', 'X','Y','Z','Nx','Ny','Nz']
        with open(out, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)

            for layer in range(len(paths)):
                for g in range(len(paths[layer])):
                    for p in range(paths[layer].len_subset(g)):
                        points = paths[layer].get(g,p)
                        # depth is the distance from point to top plane of bounding box.
                        depth = depths[layer].get(g,p)
                        normal = normals[layer].get(g,p)
                        # path should not be less than two points
                        if points.shape[0] >= 2:
                            # Format info and write to CSV row
                            for i in range(len(points)):
                                # start of path
                                if i == 0:
                                    code = 0
                                # end of path
                                elif i == len(points) - 1:
                                    code = 1 # end of path
                                # intermediate point
                                else:
                                    code = 2
                                row = np.concatenate(([layer], [code], depth[i], points[i], normal[i]))
                                writer.writerow(list(row))

        print('Paths are exported as:', out)

    '''DISPLAY FUNCTIONS (MATPLOTLIB/OPEN3D)'''
    def display_pcd(self):
        # Using Open3D
        # o3d.visualization.draw_geometries([self.Point_Cloud.pcd])

        # Using matplotlib
        fig = plt.figure(figsize=plt.figaspect(0.5))
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        # Plot data
        vertices = np.asarray(self.Point_Cloud.pcd.points)
        ax.scatter(vertices[:,0], vertices[:,1], vertices[:,2])
        plt.show()

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

    def display_toolpaths(self):
        # Plot using matplotlib (paths and normal vectors)
        fig = plt.figure(figsize=plt.figaspect(1))
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        
        paths = self.Paths.Paths
        for i in range(len(paths)):
            color = np.random.rand(3,)
            for j in range(len(paths[i])):
                for q in range(paths[i].len_subset(j)):
                    # plot the 3D paths connecting each point
                    p = paths[i].get(j,q)
                    ax.plot3D(p[:,0], p[:,1], p[:,2], color=color)
                    
                    # plot normal arrow at each point in opposite color
                    n = self.Paths.Normals[i].get(j,q)
                    ax.quiver3D(p[:,0], p[:,1], p[:,2],
                                    n[:,0], n[:,1], n[:,2],
                                    colors=1-color, length=0.3,
                                    normalize=True, linewidths=0.1)
        plt.show()