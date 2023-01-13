import numpy as np
from math import radians, ceil, floor
from rigid3d import rotation_around_z
from plane import BoundingBox
from skspatial.objects import Plane, Line
import multiprocessing

class WearPath():
    def __init__(self, meshes_obj, clad_params):
        #:param meshes_obj: wear path object
        #:param clad_params: cladding parameters
        
        self.Meshes = meshes_obj

        self.Transformation = meshes_obj.Transformation
        self.System = meshes_obj.System
        self.Coordinate = meshes_obj.Coordinate
        self.Clad_Params = clad_params

        # clippers generation
        self.generate_clippers()

        # processing paths generation
        self.generate_paths_and_normal()

        # normal vectors generation
        #self = generate_normal(self)

    # clippers generation
    def generate_clippers(self):
        # init vars
        mesh = self.Meshes.raw_mesh
        clad_ori = self.Clad_Params.CLAD_ORIENTATION
        clad_width = self.Clad_Params.CLAD_WIDTH
        max_clad_step = self.Clad_Params.MAX_CLAD_STEP
        clad_cross_hatching = self.Clad_Params.CLAD_CROSS_HATCHING
        layer_offset = self.Clad_Params.LAYER_OFFSET
        nlayers = self.Clad_Params.LAYERS

        # preallocating var
        clippers = []
        layers_clad_dirs = []

        # initial angle
        clad_ori = radians(clad_ori)
        delta = np.pi/2 * clad_cross_hatching
        
        # precomputing for speed
        if clad_cross_hatching == 0:
            layer_rot_matrix = rotation_around_z(-clad_ori)
            rot_mesh = mesh.transform(layer_rot_matrix.mat)
        else:
            # main orientation 
            layer_rot_matrix1 = rotation_around_z(-clad_ori)
            rot_mesh1 = mesh.transform(layer_rot_matrix1.mat)
            # perpendicular orientation
            layer_rot_matrix2 = rotation_around_z(-(clad_ori+delta))
            rot_mesh2 = mesh.transform(layer_rot_matrix2.mat)
        
        for layer in range(nlayers):
            if clad_cross_hatching == 1:
                # main orientation
                if layer % 2 == 1:
                    layer_rot_matrix = layer_rot_matrix1
                    rot_mesh = rot_mesh1
                else: # perpendicular orientation
                    layer_rot_matrix = layer_rot_matrix2
                    rot_mesh = rot_mesh2

            # bounding box of rotated mesh
            layer_Box = BoundingBox(rot_mesh.vertex.positions.numpy())

            # create clippped edge
            start_point = layer_Box.plane_center('front')
            end_point = layer_Box.plane_center('back')
            clad_edge = end_point - start_point
            vector = clad_edge/np.linalg.norm(clad_edge)

            # calculating cladding parameters
            cal_clad_step, nClads, offset, cal_clads_overlap = self.calc_clad_step(clad_edge, max_clad_step, clad_width)


            start_point = start_point + vector * offset

            # for layer offset
            if layer % 2 or layer_offset == 0:   # odd
                # same start_point and same number of passes
                start_point1 = start_point
            else:
                # updating the starting point according to layers offset
                start_point1 = start_point + (layer_offset * vector * cal_clad_step)
                nClads -= 1    # removing one clad on self layer
            
            # preallocating
            # layer_planes = np.zeros((nClads,9))
            layer_planes = []

            # generating planes, transform to orginal position
            for i in range(nClads):
                point = start_point1 + vector * cal_clad_step * (i-1)
    
                # apply inverse transform to point and normal
                transform = layer_rot_matrix.inverse()

                point = np.append(point, 1)
                point = transform.dot(point)[:3]

                vector = np.append(vector, 0)
                vector = transform.dot(vector)[:3]

                # Store transformed plane
                layer_planes.append(Plane(point=point, normal=vector))

            clippers.append(layer_planes)

            layers_clad_dirs.append(clad_ori)   # in radian

        self.Clad_Params.CAL_CLAD_STEP = cal_clad_step
        self.Clad_Params.CAL_CALDS_OVERLAP = cal_clads_overlap
        self.Clippers = clippers


    def calc_clad_step(self, edge, max_clad_step, clad_width):
        # CALC_CLAD_STEP Calculate the clad step distance, and how many passes will fit in
        # max_clad_step: max distance between two centers of clads
        # boundary_offset: distance to the outer margin of the boundary passes.

        boundary_offset = clad_width
        length = np.linalg.norm(edge) - boundary_offset
        remainder = (length/max_clad_step) % 1

        # if the last clad is greater than 0.5 then, cout it as a pass. 
        # If it is less then, last pass is not counted unless it is the only layer.
        if remainder >= 0.5:
            nClads = ceil(length/max_clad_step)     # Round towards plus infinity
        else:
            nClads = floor(length/max_clad_step)    # Round towards minus infinity
            # if length/maxPitch < 1.5 round up
            if nClads == 1:
                nClads = ceil(length/max_clad_step)
        

        cal_clad_step = length/nClads

        nClads = nClads + 1  # add on one pass to include first pass

        #calculate coverage and output how to much offset to center passes if
        #coverage isnt 1.
        coverage = cal_clad_step * nClads
        if coverage/length < 1  :            # some uncovered
            offset = boundary_offset + (length-coverage)/2
        else:
            offset = boundary_offset + 0    # fully covered

        # overlap
        clads_overlap = (clad_width - cal_clad_step)/clad_width

        return cal_clad_step, nClads, offset, clads_overlap

    def below_plane(self, points, plane):
        # Preallocate
        results = np.zeros(len(points))
        for i in range(len(points)):
            results[i] = plane.side_point(points[i])
        return results

    def slice_face(self, face, vof, plane):
        # returns the edge of sliced face by plane.
        # Input:
        # face: face vertices in X-Y-Z position
        # vof: vertices location (above, below, on plane)
        # plane: slicing plane

        # Output:
        # pInt: points of intersection (point, edges, nan).
        # Processing

        # ensure last point is the same as the first one for polygon purpose
        if not np.array_equal(face[0], face[-1]):
            face = np.append(face, [face[0]], axis=0)

        # FIXME: not sure what this does, why any would ever equal 2
        # checking if the vertices are on the plane:
        # vof(vof == 2) = ~vof(any(vof ~= 2))
        
        # initialize empty polygon
        # poly2 = zeros(0, 2);

        # indices of edges intersecting the plane
        # ind = find(vof ~= vof([2:end 1]));
        ind = np.where( (vof != np.roll(vof, -1)) == 1)[0]

        # compute intersection points: they are 2 for a convex polygon
        intersections = np.zeros((len(ind),3))
        for count, i in enumerate(ind):
            # Create line of face edge
            line = Line.from_points(face[i], face[i+1])
            # Compute intersection point of line/plane using skspatial
            intersections[count] = plane.intersect_line(line)

        return intersections

    # tool paths generation
    def generate_paths_and_normal(self):
        # tolerance used to identifying identical vertices
        tol = 1.49011611938477e-08 #eps(1e8)
        threshold = 3     # minimum clad path length (3 mm)

        # vars
        # mesh = self.Meshes.reconstructed_meshes
        clippers = self.Clippers
        max_laser_defocus = self.Clad_Params.MAX_LASER_DEFOCUS
        nlayers = self.Clad_Params.LAYERS

        # preallocating
        paths = []
        #layer_paths = {}

        # loop over layers using multi-threaded process
        # parfor layer = 1:nlayers

    #FIXME: add multiprocessing
    #     pool = multiprocessing.Pool()
    #     pool.map(self.path_loop, range(nlayers))
    #     pool.close()
    #     pool.join()   
    #     print('done')

    # def path_loop(self):
    #     print("threading should go here")

        for layer in range(nlayers):
            verts = self.Meshes.reconstructed_vertices[layer]
            faces = self.Meshes.reconstructed_faces[layer]

            layer_clippers = clippers[layer]

            cr = 1
            for c in range(len(layer_clippers)):
                # Get plane clipper
                clipper = layer_clippers[c]
                
                # determine position of each veritix relative to plane
                vo = self.below_plane(verts, clipper)

                # no intersection occurs if points ALL above or ALL below
                if np.all(vo == -1) or np.all(vo == 1):
                    continue

                # otherwise handle intersection
                path = np.empty((0,3))
                grouped_path = {}

                # iterating over faces
                for f in range(len(faces)):
                    fac = faces[f]      # faces indices
                    vof = vo[fac]         # vertices location

                    # processing the faces that intersect with the plane only
                    if sum(vof) == 0:
                        continue        # no intersection with self face

                    path = np.append(path, self.slice_face(verts[fac], vof, clipper), axis=0)     # adding clipped edge to paths

                # checking if no intersection happens, in case of two valleys
                if not path.size:
                    continue
        
                # arrange clipped points
                path.sort(axis=0)

                # clean up paths in case of defected mesh
                # path(isnan(path(:,1)),:) = []      # removing any NAN rows
                # uniq_inds = true(size(path,1),1)
                # iter = false(size(path,1),1)         # keep track of rows
#                     for row = 1:size(path,1)
#                         # skip if points already checked before
#                         if ~iter(row)
#                             # one point at time
#                             inds = ismembertol(path,path(row,:),tol,'ByRows', true)
#                             if sum(inds) > 2  # if same point occurs more than two times
#                                 temp = find(inds == 1)
#                                 uniq_inds(temp(3:)) = false
#                                 iter(inds) = true
#                             else
#                                 iter(inds) = true
                    
#                         else
#                             continue
                
            
#                     path = path(uniq_inds,:)


#                     # search for separation points in paths and group the connected paths
#                     k = 1   # number of disconnected paths
#                     s_p = 1 # starting point of new group
#                     for row = 1:size(path,1)  #2:size(path,1)
#                         inds = ismembertol(path,path(row,:),tol,'ByRows', true)
#                         if sum(inds) == 1 && s_p ~= row
#                             grouped_path{k,1} = path(s_p:row,:)  # grouping connected paths
#                             s_p = row+1
#                             k = k+1
#                         elseif row == size(path,1) && isempty(grouped_path)
#                             temp = path(2:-1,:)  # points in middle
#                             inds1 = ismembertol(temp,path(1,:),tol,'ByRows', true)    # first point
#                             inds2 = ismembertol(temp,path(,:),tol,'ByRows', true)  # last point
#                             if sum(inds1) == 0 
#                                 temp = [path(1,:) temp]
                    
#                             if sum(inds2) == 0
#                                 temp = [temp path(,:)]
                    
#                             grouped_path{k,1} = temp # grouping connected paths
#                             k = k+1
                
            

#                     # remove path if it less than threshold:
#                     cr2 = 1 flag = false
#                     for group = 1:size(grouped_path,1)
#                         d = diff(grouped_path{group,1}(:,:))
#                         length = sum(sqrt(sum(d.^2,2)))
#                         if length >= threshold
#                             layer_paths{cr,1}{cr2,1} = [grouped_path{group,1}(1:2:-1,:) grouped_path{group,1}(,:)]   # removing mid points (connection points)
#                             cr2 = cr2 + 1
#                             flag = true
                
            
            
#                     # if there are paths after checking the length of path
#                     if flag
#                         cr = cr + 1 
            

#                   # (  clipper )


#                 #                 # processing paths using (arch height method)
#                 #                 for group = 1:size(layer_paths,1)
#                 #                     # one group of paths in layer
#                 #                     for path = 1:size(layer_paths{group,1},1)
#                 #                         # all points in path, except first and last
#                 #                         points = layer_paths{group,1}{path,1}(:,:)
#                 #                         sz = size(points,1)
#                 #                         new_points = points(1,:)
#                 #                         inds_points = false(sz,1) # track points
#                 #                         for p = 1:sz
#                 #                             if ~inds_points(p)
#                 #                                 d_max = 0
#                 #                                 cr3 = 1
#                 #                                 pt_s = points(p,:)
#                 #                                 mid_points = []
#                 #                                 while d_max < max_laser_defocus
#                 #                                     # terminate if last point
#                 #                                     if sz == cr3+1
#                 #                                        I = sz
#                 #                                        break
#                 #                                     
#                 #                                     pt_m = points(p+cr3,:)
#                 #                                     mid_points = [mid_points pt_m]
#                 #                                     pt_e = points(p+1+cr3,:)
#                 #                                     line = createLine3d(pt_s,pt_e)
#                 #                                     d = distancePointLine3d(mid_points(:,:), line)
#                 #                                     [d_max, I] = max(d(:))
#                 #                                     cr3 = cr3 + 1
#                 #                                 
#                 #                                 new_points = [new_points points(p-1+I,:)]
#                 #                                 inds_points(p:p-1+I,:) = true
#                 #                             else
#                 #                                 continue
#                 #                             
#                 #
#                 #                         
#                 #                         points = [points layer_paths{group,1}{path,1}(,:)]
#                 #                         layer_paths{group,1}{path,1} = points  # new path
#                 #                     
#                 #                 


#                 ### normals generation:
#                 # first clipper normal in each layer:
#                 cladVertor = planeNormal(clippers{layer,1}(1,:))

#                 # preallocating:
#                 sz = size(layer_paths,1)
#                 normal = cell(sz,1)
#                 midp = cell(sz,1)

#                 for nPaths = 1 : sz
#                     for q = 1: size(layer_paths{nPaths,1},1)
#                         vn = normalizeVector3d( diff(layer_paths{nPaths,1}{q,1}))
#                         temp = crossProduct3d(vn, cladVertor)
#                         normal{nPaths,1}{q,1} = [temp(:,1:2), abs(temp(:,3))]  # making sure it is pointing upwards in the build direction
#                         midp{nPaths,1}{q,1} = midEdge(layer_paths{nPaths,1}{q,1})
            
        


#                 # processing points and normals
#                 scan_step = max_laser_defocus
#                 top_plane = mesh(layer).BoundingBox.planes('top').plane
#                 layer_depth = {}
#                 for i = 1:size(layer_paths,1)    # loop over paths for one layer
#                     for q = 1:size(layer_paths{i,1},1)
#                         # one path and its normals on one layer
#                         one_path = layer_paths{i,1}{q,1}                                                    # one path array
#                         one_path_normals = normal{i,1}{q,1}(:,:)

#                         d = diff(one_path)
#                         pathLength = sum(sqrt(sum(d.^2,2)))
#                         nPoints = fix( pathLength / scan_step)                                         # number of new points in path
#                         if nPoints == 1  # if only one point, take first and  points.
#                             layer_paths{i,1}{q,1} = [one_path(1,:) one_path(,:)]
#                             normal{i,1}{q,1} = [one_path_normals(1,:) one_path_normals(,:)]
#                         else
#                             [layer_paths{i,1}{q,1}, ~, ~, N_indes] = interparc(nPoints,one_path(:,1),one_path(:,2),one_path(:,3), 'linear')
#                             normal{i,1}{q,1} = one_path_normals(N_indes,:)
                

#                         # computing depth of points
#                         jsize = size(layer_paths{i,1}{q,1},1)
#                         path_depths = zeros(jsize,1)
#                         for j = 1:jsize
#                             path_depths(j,1) = distancePointPlane(layer_paths{i,1}{q,1}(j,:), top_plane)
                

#                         layer_depth{i,1}{q,1} = path_depths

            

        


#                 paths{layer,1} = layer_paths
#                 normals{layer,1} = normal
#                 depths{layer,1} = layer_depth
#                 #midpoints{layer,1} = midp




#              # ( of layers)



#             # update object
#             self.Paths = paths
#             self.Normals = normals
#             self.Depths = depths
        

    








#     methods(Access = public)

#         function self = transform(self, to_system)
#             #transform Summary of self method goes here
#             #   Detailed explanation goes here

#             if ~strcmpi(self.System, to_system) && any(strcmpi({'global', 'local'}, to_system))
#                 paths = self.Paths
#                 transf = self.Transformation
#                 for group = 1:size(paths,1)
#                     for path  = 1:size(paths{group,1},1)
#                         points = paths{group,1}{path,1}(:,:)
#                         tf_points = transf.transformPointsForward(points{1,1})
#                         paths{group,1}{path,1} = tf_points
                    
                

#                 # update points and center
#                 self.System = to_system
#                 self.Transformation.Translation = -1 * self.Transformation.Translation        # flipping translation
#                 self.Paths = paths
#             else
#                 warning("System is already in #d", to_system)
            

        






#         function self = convert(self, to_coord)
#             #convert Summary of self method goes here
#             #   Detailed explanation goes here

#             #:param to: 'none', 'cart', 'cyl', 'sphe'

#             # convert from current system to the targeted system
#             paths = self.Paths
#             for group = 1:size(paths,1)
#                 for path  = 1:size(paths{group,1},1)
#                     points = paths{group,1}{path,1}(:,:)
#                     new_points = coordSystems.convert(points, coordSystems.(self.Coordinate), coordSystems.(to_coord))
#                     paths{group,1}{path,1} = new_points
                
            
#             self.Transformation.Translation = coordSystems.convert(self.Transformation.Translation, coordSystems.(self.Coordinate), coordSystems.(to_coord))

#             # update center and coordinate
#             self.Coordinate = to_coord
#             self.Paths = paths
        






#         function [out1, out2, out3] = export(self, to_system, to_coord)
#             # Example: Paths.export('local', 'cart')
#             # check if system is different
#             if ~matches(self.System, to_system)
#                 temp = transform(self, to_system)
#                 # check if coordinate is different
#                 if ~matches(self.Coordinate, to_coord)
#                     temp = convert(temp, to_coord)
#                     out1 = temp.Paths
#                 else
#                     out1 = temp.Paths
                
#             else
#                 if ~matches(self.Coordinate, to_coord)
#                     temp = convert(self, to_coord)
#                     out1 = temp.Paths
#                 else
#                     out1 = self.Paths
                
            

#             out2 = self.Depths
#             out3 = self.Normals