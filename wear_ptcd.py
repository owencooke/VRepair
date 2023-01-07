import numpy as np
import open3d as o3d
from coord_systems import cart_to_cyl, convert_coords, Coords
from rigid3d import RigidTransform

class WearPointCloud():
    def __init__(self,filename, coordinate, system, conv_factors=np.array([1,1,1])):
        self.Coordinate = coordinate 
        self.System = system 

        # loading point cloud from file in global system
        self.pcd = o3d.io.read_point_cloud(filename)
        # o3d.visualization.draw_geometries([pcd])

        # Get points
        self.points = np.asarray(self.pcd.points)

        # pc formating
        self.format(conv_factors) 

        # local system transformation
        self.Transformation = RigidTransform(trans=-self.center) 


        # switch to local system if the input is in global
        if self.System == 'global':
            self.transform('local') 

        # calculate normals
        # self.normals() 


    def format(self, conv_factors):
        # format point cloud data appropriately to match the chosen system axes
        # NOTE: Hamdan's cylindrical conversion code does not actually convert coordinate values
        # if self.Coordinate == Coords.CYL:
        #     self.points = cart_to_cyl(self.points)
        
        # Multiply by coordinate conversion factor
        self.points *= conv_factors

        self.center, _ = self.pcd.compute_mean_and_covariance()       # point cloud center
        self.pcd.points = o3d.utility.Vector3dVector(self.points)
        # o3d.visualization.draw_geometries([self.pcd])

    def transform(self, to):
        # if ~strcmpi(self.System, to) && any(strcmpi({'global', 'local'}, to))
        self.pcd.transform(self.Transformation.mat)
        self.Transformation.flip()       # flipping translation
        self.System = to 

        # update points and center
        # self.points = np.asarray(self.pcd.points) # shallow copy
        self.center, _ = self.pcd.compute_mean_and_covariance()   

    def convert(self, to):
        #:param to: 'none', 'cart', 'cyl', 'sphe'
        # convert from current system to the targeted system, points, translation
        self.points = convert_coords(self.points, self.Coordinate, to) 
        t = convert_coords(self.Transformation.getTranslation(), self.Coordinate, to)
        self.Transformation.setTranslation(t) 

        # update point cloud object, center and coordinate 
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(self.points)
        self.center, _ = self.pcd.compute_mean_and_covariance()   
        self.Coordinate = to

    def normals(self):
        #NOTE: normals must be computed if needed
        # http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.estimate_normals
        self.Normals = self.pcd.normals

    # #NOTE TO COORD SHOULD BE COORD ENUM
    # def export(self, to_system, to_coord):
    #         # Example: ptcd.export('local', 'cart')
    #         # check if system is different
    #         if self.System != to_system:
    #             temp = self.transform(to_system) 
    #             # check if coordinate is different
    #             if self.Coordinate != to_coord:
    #                 temp = self.convert(temp, to_coord) 
    #                 out = pointCloud(temp.points) 
    #             else
    #                 out = pointCloud(temp.points) 
    #             end
    #         else
    #             if ~matches(self.Coordinate, to_coord)
    #                 temp = convert(self, to_coord) 
    #                 out = pointCloud(temp.points) 
    #             else
    #                 out = pointCloud(self.points) 

if __name__ == "__main__":
    Point_Cloud = WearPointCloud("FBH.pcd", Coords.CYL, 'global')
