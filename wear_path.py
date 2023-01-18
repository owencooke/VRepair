import numpy as np
from math import radians, ceil, floor
from skspatial.objects import Plane, Line
import multiprocessing

from rigid3d import rotation_around_z
from plane import BoundingBox
from multicell import MultiCell
from interparc import interpcurve

class WearPath():
    def __init__(self, meshes_obj, clad_params):
        self.Meshes = meshes_obj
        self.Transformation = meshes_obj.Transformation
        self.System = meshes_obj.System
        self.Coordinate = meshes_obj.Coordinate
        self.Clad_Params = clad_params

        # clippers generation
        self.generate_clippers()

        # processing paths generation
        self.generate_paths_and_normals()

    def generate_clippers(self):
        # init vars
        mesh = self.Meshes.raw_mesh
        clad_width = self.Clad_Params.CLAD_WIDTH
        max_clad_step = self.Clad_Params.MAX_CLAD_STEP
        clad_cross_hatching = self.Clad_Params.CLAD_CROSS_HATCHING
        layer_offset = self.Clad_Params.LAYER_OFFSET

        # initial angle
        clad_ori = radians(self.Clad_Params.CLAD_ORIENTATION)
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
        
        # begin clipper generation
        clippers = []
        for layer in range(self.Clad_Params.LAYERS):
            if clad_cross_hatching == 1:
                # main orientation
                if layer % 2 == 1:
                    layer_rot_matrix = layer_rot_matrix1
                    rot_mesh = rot_mesh1
                else: # perpendicular orientation
                    layer_rot_matrix = layer_rot_matrix2
                    rot_mesh = rot_mesh2

            # create clippped edge
            layer_Box = BoundingBox(rot_mesh.vertex.positions.numpy())
            start_point = layer_Box.plane_center('front')
            end_point = layer_Box.plane_center('back')
            clad_edge = end_point - start_point
            vector = clad_edge/np.linalg.norm(clad_edge)

            # calculating cladding parameters
            cal_clad_step, nClads, offset, cal_clads_overlap = self.calc_clad_step(clad_edge, max_clad_step, clad_width)

            # add layer offset
            start_point += vector * offset
            if layer % 2 or layer_offset == 0:   # odd
                # same start_point and same number of passes
                start_point1 = start_point
            else:
                # updating the starting point according to layers offset
                start_point1 = start_point + (layer_offset * vector * cal_clad_step)
                nClads -= 1    # removing one clad on self layer

            # generating planes, transform to orginal position
            layer_planes = []
            for i in range(nClads):
                point = start_point1 + vector * cal_clad_step * i
    
                # apply inverse transform to point and normal
                transform = layer_rot_matrix.inverse()
                point = np.append(point, 1)
                point = transform.dot(point)[:3]
                vector = np.append(vector, 0)
                vector = transform.dot(vector)[:3]

                # Store transformed plane
                layer_planes.append(Plane(point=point, normal=vector))

            clippers.append(layer_planes)

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

    #FIXME: verify accuracy compared to MATLAB function
    def ismembertol(self, a, b, tol):
        return np.all(np.abs(a-b) <= tol, axis=1)

    def slice_face(self, face, vof, plane):
        # returns the edge of sliced face by plane.
        # Input:
        # face: face vertices in X-Y-Z position
        # vof: vertices location (above, below, on plane)
        # plane: slicing plane

        # Output:
        # pInt: points of intersection (point, edges, nan).

        # ensure last point is the same as the first one for polygon purpose
        if not np.array_equal(face[0], face[-1]):
            face = np.append(face, [face[0]], axis=0)

        # FIXME: not sure what this does, why any would ever equal 2
        # checking if the vertices are on the plane:
        # vof(vof == 2) = ~vof(any(vof ~= 2))

        # indices of edges intersecting the plane
        ind = np.where( (vof != np.roll(vof, -1)) == 1)[0]

        # compute intersection points: they are 2 for a convex polygon
        intersections = np.zeros((len(ind),3))
        for count, i in enumerate(ind):
            # Create line of face edge
            line = Line.from_points(face[i], face[i+1])
            # Compute intersection point of line/plane using skspatial
            intersections[count] = plane.intersect_line(line)

        return intersections

    # return numpy array of the differences between subsequent points
    # verified by unit testing
    def diff(self, points):
        # Return vectorized subtraction, ignoring first index
        return (points - np.roll(points,1, axis=0))[1:]

    def path_length(self, points):
        return sum(np.sqrt(np.sum(points**2,axis=1)))

    # tool paths generation
    def generate_paths_and_normals(self):
        paths = []
        normals = []
        depths = []
        # Call subprocesses for each layer
        for layer in range(self.Clad_Params.LAYERS):
            layer_paths = self.get_paths(layer)
            normal = self.get_normals(layer, layer_paths)
            layer_paths, normal, layer_depth = self.process_paths_normals(layer, layer_paths, normal)
     
            paths.append(layer_paths)
            normals.append(normal)
            depths.append(layer_depth)

        # update object
        self.Paths = paths
        self.Normals = normals
        self.Depths = depths

    def get_paths(self, layer):
        # tolerance used to identifying identical vertices
        tol = 1.49011611938477e-08 #eps(1e8)
        threshold = 3     # minimum clad path length (3 mm)

        # preallocating
        layer_paths = MultiCell()

        verts = self.Meshes.reconstructed_vertices[layer]
        faces = self.Meshes.reconstructed_faces[layer]

        layer_clippers = self.Clippers[layer]

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
            grouped_path = []

            # iterating over faces
            for f in range(len(faces)):
                fac = faces[f]      # faces indices
                vof = vo[fac]         # vertices location

                # processing the faces that intersect with the plane only
                if sum(vof) == 0:
                    continue        # no intersection with self face
                
                # adding clipped edge to paths
                path = np.append(path, self.slice_face(verts[fac], vof, clipper), axis=0)

            # checking if no intersection happens, in case of two valleys
            if not path.size:
                continue
    
            # arrange clipped points
            path.sort(axis=0)

            # clean up paths in case of defected mesh
            path = path[~np.isnan(path).any(axis=1)]       # removing any NAN rows
            uniq_inds = np.full(len(path), True)
            iter = np.full(len(path), False)         # keep track of rows

            # Compute relative tolerance
            rel_tol = tol*np.max(np.abs(path))

            for row in range(len(path)):
                # skip if points already checked before
                if ~iter[row]:
                    # one point at time
                    inds = self.ismembertol(path, path[row], rel_tol)

                    if sum(inds) > 2:  # if same point occurs more than two times
                        temp = np.where(inds == 1)[0]
                        uniq_inds[temp[3:]] = False
                        iter[inds] = True
                    else:
                        iter[inds] = True

            path = path[uniq_inds]

            # search for separation points in paths and group the connected paths
            k = 1   # number of disconnected paths
            s_p = 1 # starting point of new group
            for row in range(len(path)): #2:size(path,1)
                inds = self.ismembertol(path, path[row], rel_tol)
                if sum(inds) == 1 and s_p != row:
                    grouped_path.append(path[s_p:row])  # grouping connected paths
                    s_p = row+1
                    k += 1
                elif row == len(path)-1 and not grouped_path:
                    temp = path[1:-1] # points w/o first and final
                    inds1 = self.ismembertol(temp, path[0], rel_tol) # first point
                    inds2 = self.ismembertol(temp, path[-1],rel_tol) # last point
                    if sum(inds1) == 0:
                        temp = np.append(temp, path[0], axis=0)
                
                    if sum(inds2) == 0:
                        temp = np.append(temp, path[-1], axis=0)
                
                    grouped_path.append(temp) # grouping connected paths
                    k += 1
            
            # remove path if it less than threshold:
            cr2 = 1
            flag = False
            for g in range(len(grouped_path)):
                d = self.diff(grouped_path[g])
                if self.path_length(d) >= threshold:
                    # removing mid points (connection points)
                    x = np.append(grouped_path[g][0:len(grouped_path[g])-1:2], [grouped_path[g][-1]], axis=0)
                    # if x.shape[1] != 3:
                    #     continue
                    layer_paths.set(x, cr, cr2)
                    cr2 += 1
                    flag = True
            
            # if there are paths after checking the length of path
            if flag:
                cr += 1 
        return layer_paths

    def get_normals(self, layer, layer_paths):
        ### normals generation:
        sz = len(layer_paths)
        normal = MultiCell()
        for nPaths in range(sz):
            for q in range(layer_paths.len_subset(nPaths)):
                lp = layer_paths.get(nPaths,q)
                if lp is None:
                    continue
                
                d = self.diff(lp)
                vn = d / np.linalg.norm(d)
                cross = np.cross(vn, self.Clippers[layer][0].normal)

                # make sure it is pointing upwards in the build direction
                np.abs(cross[:,2], out=cross[:,2])
                normal.set(cross, nPaths, q)

        return normal
        
    def process_paths_normals(self, layer, layer_paths, normal):
        # processing points and normals
        top_plane = self.Meshes.reconstructed_BoundingBoxes[layer].get_plane("top")
        layer_depth = MultiCell()
        for i in range(len(layer_paths)):  # loop over paths for one layer
            for q in range(layer_paths.len_subset(i)):
                # one path and its normals on one layer
                one_path = layer_paths.get(i,q)
                if one_path is None:
                    continue
                one_path_normals = normal.get(i,q)

                d = self.diff(one_path)
                pathLength = self.path_length(d)

                # number of new points in path
                nPoints = int(np.fix(pathLength / self.Clad_Params.MAX_LASER_DEFOCUS))
                if nPoints == 1:
                    # if only one point, take first and end points
                    layer_paths.set(one_path[[0,-1]], i, q)
                    normal.set(one_path_normals[[0,-1]], i, q)
                # else:
                    p, N_indes = interpcurve(nPoints,one_path)
                    layer_paths.set(p, i, q)
                    normal.set(one_path_normals[N_indes], i, q)
            
                # computing depth of points
                point = layer_paths.get(i,q)
                jsize = len(point)
                path_depths = np.zeros((jsize,1))
                for j in range(jsize):
                    path_depths[j] = top_plane.distance_point_signed(point[j])
        
                layer_depth.set(path_depths, i, q)

        return layer_paths, normal, layer_depth