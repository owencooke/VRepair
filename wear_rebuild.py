import csv
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import time

from clad_params import CladParams
from wear_mesh import WearMesh
from wear_path import WearPath

'''
A program for generating toolpaths for the repairing process of 
a wear using metal deposition modeling process.
'''
class WearRebuild(): 
    def __init__(self, filename):
        self.FILENAME = filename 
        # initialize raw point cloud from file
        self.cloud = o3d.io.read_point_cloud(filename)

        # init cladding parameters
        self.Clad_Params = CladParams(1,                     # clad height (mm)
                                10,                     # clad width  (mm)
                                5,                     # max clad step (mm)  (clads overlap is considered)
                                0,                     # clad orientation (degrees)
                                0,                # clad cross hatching ( enable or disable )
                                0,                     # layer offset (e.g. 0.5) (0-1)
                                3                     # max laser defocus (mm)
        )

    def cloud_points(self):
        return np.asarray(self.cloud.points)
    
    # Execute repair process
    def run(self): 
        self.Meshes = WearMesh(self.cloud_points(), self.Clad_Params) 
        # generate sliced mesh
        self.Meshes.generate_slices()
        self.Clad_Params = self.Meshes.Clad_Params   # update clad_params

        # # 3- paths object constuction
        self.Paths = WearPath(self.Meshes, self.Clad_Params) 
        self.Clad_Params = self.Paths.Clad_Params   # update clad_params
 
    # export paths to CSV
    def paths_export(self):
        out = "test_output.csv"

        # get path values according to system/coord type
        paths, depths, normals = self.Paths.export()

        # Create CSV output file
        header = ['layer_number', 'group', 'point_code','point_depth', 'X','Y','Z','Nx','Ny','Nz']
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
                                row = np.concatenate(([layer], [g], [code], depth[i], points[i], normal[i]))
                                writer.writerow(list(row))

        print('Paths are exported as:', out)

    '''DISPLAY FUNCTIONS (MATPLOTLIB/OPEN3D)'''
    def display_pcd(self):
        # Using Open3D
        # o3d.visualization.draw_geometries([self.cloud])

        # Using matplotlib
        fig = plt.figure(figsize=plt.figaspect(0.5))
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        # Plot data
        vertices = self.cloud_points()
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

if __name__ == "__main__":
    print("Processing")
    start = time.time()

    # Run wear rebuild program
    model = WearRebuild('FBH.pcd')
    model.run()

    end = time.time()
    print(end-start, "seconds")

    # post-processing section
    model.display_pcd()
    model.display_raw_mesh()
    model.display_reconst_meshes()
    model.display_toolpaths()

    # export paths
    model.paths_export()