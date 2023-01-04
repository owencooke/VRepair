class WearPointCLoud():
    Coordinate = 'cyl' 
    System = 'global'            # default input system
    
    # constructor
    def __init__(filename, coordinate, system, x_conv_factor=1, y_conv_factor=1, z_conv_factor=1):
        this.Coordinate = coordinate 
        this.System = system 

        # loading point cloud from file in global system
        this.Points = pcread(filename).Location

        #FIXME: unsure of this purpose
        # # pc formating
        # this = format(this, x_conv_factor, y_conv_factor, z_conv_factor) 

        # local system transformation
        rot = createRotation 
        trans = this.Center     # this to switch between global and local system
        this.Transformation = rigid3d(rot, -1 * trans) 


        # switch to local system if the input is in global
        if matches(this.System, 'global')
            this = transform(this, 'local') 
        end

        # calculate normals
        this = this.normals() 
    end
end


    methods(Access = protected)
        function this = format(this, x_conv_factor, y_conv_factor, z_conv_factor)
            #format Summary of this method goes here
            #   format point cloud data appropriately to match the chosen system axes

            points = this.Points 
            switch coordSystems.(this.Coordinate)
                case coordSystems.none
                    this.Points = coordSystems.formatCart(points(:,1), points(:,2), points(:,3), x_conv_factor, y_conv_factor, z_conv_factor) 
                case coordSystems.cart  # [ X, Y, Z ]
                    this.Points = coordSystems.formatCart(points(:,1), points(:,2), points(:,3), x_conv_factor, y_conv_factor, z_conv_factor) 
                case coordSystems.cyl   # pc = [THETA, R, Z]  # kept as it, so that it matches user frame
                    this.Points = coordSystems.formatCylindrical(points(:,1), points(:,2), points(:,3), x_conv_factor, y_conv_factor, z_conv_factor) 
                case coordSystems.sphere # pc = [THETA, PHI, RHO]  #  0 < PHI < 180 , with +Z axis
                    this.Points = coordSystems.formatSphere(points(:,1), points(:,3), points(:,2), x_conv_factor, y_conv_factor, z_conv_factor) 
            end

            # store point cloud into its object for further manipulation
            this.Center = mean(this.Points)       # point cloud center
            this.PCD_Obj = pointCloud(this.Points) 
        end

        function this = transform(this, to)
            #transform Summary of this method goes here
            #   Detailed explanation goes here

            if ~strcmpi(this.System, to) && any(strcmpi({'global', 'local'}, to))
                this.PCD_Obj = pctransform(this.PCD_Obj,this.Transformation) 
                this.Transformation.Translation = -1 * this.Transformation.Translation         # flipping translation
                this.System = to 

                # update points and center
                this.Points = this.PCD_Obj.Location 
                this.Center =  mean(this.Points) 
            else
                warning("System is already in #d", to)
            end

        end

        function this = convert(this, to)
            #convert Summary of this method goes here
            #   Detailed explanation goes here

            #:param to: 'none', 'cart', 'cyl', 'sphe'

            # convert from current system to the targeted system, points,
            # and transilation values
            this.Points = coordSystems.convert(this.Points, coordSystems.(this.Coordinate), coordSystems.(to)) 
            this.Transformation.Translation = coordSystems.convert(this.Transformation.Translation, coordSystems.(this.Coordinate), coordSystems.(to)) 

            # update point cloud object, center and coordinate
            this.PCD_Obj = pointCloud(this.Points) 
            this.Center =  mean(this.Points) 
            this.Coordinate = to 
        end

        function this = normals(this)
            #normals Summary of this method goes here
            #   Detailed explanation goes here

            init_normals = pcnormals(this.PCD_Obj) 

            this.Normals = init_normals 

        function out = export(this, to_system, to_coord)
            # Example: ptcd.export('local', 'cart')
            # check if system is different
            if ~matches(this.System, to_system)
                temp = transform(this, to_system) 
                # check if coordinate is different
                if ~matches(this.Coordinate, to_coord)
                    temp = convert(temp, to_coord) 
                    out = pointCloud(temp.Points) 
                else
                    out = pointCloud(temp.Points) 
                end
            else
                if ~matches(this.Coordinate, to_coord)
                    temp = convert(this, to_coord) 
                    out = pointCloud(temp.Points) 
                else
                    out = pointCloud(this.Points) 
