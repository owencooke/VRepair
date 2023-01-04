import numpy as np
import open3d as o3d

class WearPointCLoud():
    def __init__(self,filename, coordinate, system, x_conv_factor=1, y_conv_factor=1, z_conv_factor=1):
        self.Coordinate = coordinate 
        self.System = system 

        # loading point cloud from file in global system
        self.pcd = o3d.io.read_point_cloud(filename)

        # o3d.visualization.draw_geometries([pcd])

        # Get points
        self.points = np.asarray(self.pcd.points)

        # pc formating
        self.format(self, x_conv_factor, y_conv_factor, z_conv_factor) 

        # local system transformation
        rot = np.array((
                    (1,	0,	0),
                    (0,	1,	0),
                    (0,	0,	1)
                ))    
        trans = self.Center     # self to switch between global and local system
        self.Transformation = rigid3d(rot, -1 * trans) 


        # switch to local system if the input is in global
        if matches(self.System, 'global')
            self = transform(self, 'local') 
        end

        # calculate normals
        self.normals() 


    def format(self, x_conv_factor, y_conv_factor, z_conv_factor):
            # format point cloud data appropriately to match the chosen system axes
            switch coordSystems.(self.Coordinate)
                case coordSystems.none
                    self.points = coordSystems.formatCart(points(:,1), points(:,2), points(:,3), x_conv_factor, y_conv_factor, z_conv_factor) 
                case coordSystems.cart  # [ X, Y, Z ]
                    self.points = coordSystems.formatCart(points(:,1), points(:,2), points(:,3), x_conv_factor, y_conv_factor, z_conv_factor) 
                case coordSystems.cyl   # pc = [THETA, R, Z]  # kept as it, so that it matches user frame
                    self.points = coordSystems.formatCylindrical(points(:,1), points(:,2), points(:,3), x_conv_factor, y_conv_factor, z_conv_factor) 
                case coordSystems.sphere # pc = [THETA, PHI, RHO]  #  0 < PHI < 180 , with +Z axis
                    self.points = coordSystems.formatSphere(points(:,1), points(:,3), points(:,2), x_conv_factor, y_conv_factor, z_conv_factor)

            # store point cloud into its object for further manipulation
            self.Center = mean(self.points)       # point cloud center
            self.PCD_Obj = pointCloud(self.points) 
        end

        function self = transform(self, to)
            #transform Summary of self method goes here
            #   Detailed explanation goes here

            if ~strcmpi(self.System, to) && any(strcmpi({'global', 'local'}, to))
                self.PCD_Obj = pctransform(self.PCD_Obj,self.Transformation) 
                self.Transformation.Translation = -1 * self.Transformation.Translation         # flipping translation
                self.System = to 

                # update points and center
                self.points = self.PCD_Obj.Location 
                self.Center =  mean(self.points) 
            else
                warning("System is already in #d", to)
            end

        end

        function self = convert(self, to)
            #convert Summary of self method goes here
            #   Detailed explanation goes here

            #:param to: 'none', 'cart', 'cyl', 'sphe'

            # convert from current system to the targeted system, points,
            # and transilation values
            self.points = coordSystems.convert(self.points, coordSystems.(self.Coordinate), coordSystems.(to)) 
            self.Transformation.Translation = coordSystems.convert(self.Transformation.Translation, coordSystems.(self.Coordinate), coordSystems.(to)) 

            # update point cloud object, center and coordinate
            self.PCD_Obj = pointCloud(self.points) 
            self.Center =  mean(self.points) 
            self.Coordinate = to 
        end

        function self = normals(self)
            #normals Summary of self method goes here
            #   Detailed explanation goes here

            init_normals = pcnormals(self.PCD_Obj) 

            self.Normals = init_normals 

        function out = export(self, to_system, to_coord)
            # Example: ptcd.export('local', 'cart')
            # check if system is different
            if ~matches(self.System, to_system)
                temp = transform(self, to_system) 
                # check if coordinate is different
                if ~matches(self.Coordinate, to_coord)
                    temp = convert(temp, to_coord) 
                    out = pointCloud(temp.points) 
                else
                    out = pointCloud(temp.points) 
                end
            else
                if ~matches(self.Coordinate, to_coord)
                    temp = convert(self, to_coord) 
                    out = pointCloud(temp.points) 
                else
                    out = pointCloud(self.points) 
