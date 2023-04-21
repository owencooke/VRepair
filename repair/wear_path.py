import csv
import numpy as np
from math import radians, ceil, floor
from skspatial.objects import Plane, Line
from qtpy.QtCore import Signal, QObject

from repair.transforms import rotation_around_z, BoundingBox
from repair.utils import MultiCell, interpcurve

'''
WearPath class takes in a sliced 3D mesh. It then performs the path planning
for the laser cladding repair process. The output is the paths, along with their
depths and normals, to be used to direct the robotic repair process. 

Usage: instantiate using the sliced mesh and clad params. Then call:
        `generate_clippers()` and `generate_paths_and_normals`. The results
        can be accessed using the results function.
'''
class WearPath(QObject):
    progress = Signal(float)

    def __init__(self, meshes_obj, clad_params):
        super().__init__()
        self.Meshes = meshes_obj
        self.Clad_Params = clad_params

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
        if not clad_cross_hatching:
            layer_rot_matrix = rotation_around_z(-clad_ori)
            rot_mesh = mesh.transform(layer_rot_matrix)
        else:
            # main orientation 
            layer_rot_matrix1 = rotation_around_z(-clad_ori)
            rot_mesh1 = mesh.transform(layer_rot_matrix1)
            # perpendicular orientation
            layer_rot_matrix2 = rotation_around_z(-(clad_ori+delta))
            rot_mesh2 = mesh.transform(layer_rot_matrix2)
        
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
                transform = np.linalg.inv(layer_rot_matrix)
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

    # CALC_CLAD_STEP Calculate the clad step distance, and how many passes will fit in
    def calc_clad_step(self, edge, max_clad_step, clad_width):
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

        # calculate coverage and output how to much offset to
        # center passes if coverage isnt 1.
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

    def ismembertol(self, a, b, tol):
        return np.all(np.abs(a-b) <= tol, axis=1)

    # returns intersection points of sliced face by plane.
    def slice_face(self, face, vof, plane):
        # ensure last point is the same as the first one for polygon purpose
        if not np.array_equal(face[0], face[-1]):
            face = np.append(face, [face[0]], axis=0)

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
    def diff(self, points):
        return (points - np.roll(points,1, axis=0))[1:]

    def path_length(self, points):
        return sum(np.sqrt(np.sum(points**2,axis=1)))

    def remove_duplicate_points(self, path, tol):
        uniq_inds = np.full(len(path), True)
        iter = np.full(len(path), False)
        for row in range(len(path)):
            # skip if points already checked before
            if ~iter[row]:
                # compare each point to others
                inds = self.ismembertol(path, path[row], tol)
                if sum(inds) > 2:
                    # classify 3rd+ occurences as non-unique points
                    temp = np.where(inds == 1)[0]
                    uniq_inds[temp[2:]] = False
                    iter[inds] = True
                else:
                    iter[inds] = True

        return path[uniq_inds]

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
     
            # store
            paths.append(layer_paths)
            normals.append(normal)
            depths.append(layer_depth)

            # send signal to GUI to update progress bar
            self.progress.emit((layer+1)/self.Clad_Params.LAYERS)

        # update object
        self.Paths = paths
        self.Normals = normals
        self.Depths = depths

    def get_paths(self, layer, debug=False, layer_clippers=None, verts=None, faces=None):
        layer_paths = MultiCell()

        if not debug:
            verts = self.Meshes.reconstructed_vertices[layer]
            faces = self.Meshes.reconstructed_faces[layer]
            layer_clippers = self.Clippers[layer]

        cr = 0
        for c in range(len(layer_clippers)):
            # determine position of each vertex relative to plane clipper
            clipper = layer_clippers[c]
            vo = self.below_plane(verts, clipper)

            # no intersection occurs if points ALL above or ALL below
            if np.all(vo == -1) or np.all(vo == 1):
                continue

            # handle intersection
            path = np.empty((0,3))
            grouped_path = []
            for f in range(len(faces)):
                fac = faces[f]
                vof = vo[fac]
                # processing the faces that intersect with the plane only
                if sum(vof) == 0:
                    continue
                # add clipped edge to paths
                path = np.append(path, self.slice_face(verts[fac], vof, clipper), axis=0)

            # check if no intersection, in case of two valleys
            if not path.size:
                continue
    
            # sort points acscending by X
            ind = np.lexsort((path[:,2],path[:,1],path[:,0])) 
            path = path[ind]

            # removing any NAN rows in case of defected mesh
            path = path[~np.isnan(path).any(axis=1)]

            # Remove duplicate points
            tol = 1.49011611938477e-08
            rel_tol = tol*np.max(np.abs(path))
            path = self.remove_duplicate_points(path, rel_tol)

            # search for separation points in paths and group the connected paths
            s_p = 0 # starting point of new group
            for row in range(len(path)):
                inds = self.ismembertol(path, path[row], rel_tol)

                if sum(inds) == 1 and s_p != row:
                    grouped_path.append(path[s_p:row+1])  # grouping connected paths
                    s_p = row+1
                elif row == len(path)-1 and not grouped_path:
                    temp = path[1:-1] # points w/o first and final
                    inds1 = self.ismembertol(temp, path[0], rel_tol) # first point
                    inds2 = self.ismembertol(temp, path[-1],rel_tol) # last point
                    if sum(inds1) == 0:
                        temp = np.append(temp, [path[0]], axis=0)
                
                    if sum(inds2) == 0:
                        temp = np.append(temp, [path[-1]], axis=0)
                
                    grouped_path.append(temp) # grouping connected paths
            
            # remove path if it less than threshold:
            threshold = 3     # minimum clad path length (3 mm)
            cr2 = 0
            flag = False
            for g in range(len(grouped_path)):
                d = self.diff(grouped_path[g])
                length = self.path_length(d)
                if length >= threshold:
                    # removing mid points (connection points)
                    x = np.append(grouped_path[g][0:len(grouped_path[g])-1:2], [grouped_path[g][-1]], axis=0)
                    layer_paths[cr, cr2] = x
                    cr2 += 1
                    flag = True
            
            # if there are paths after checking the length of path
            if flag:
                cr += 1 
        return layer_paths

    def get_normals(self, layer, layer_paths):
        normals = MultiCell()
        for nPaths in range(len(layer_paths)):
            for q in range(layer_paths.len_subset(nPaths)):
                # Calculate normal vectors
                d = self.diff(layer_paths[nPaths, q])
                norm = np.linalg.norm(d, axis=-1, keepdims=True)
                cross = np.cross(d/norm, self.Clippers[layer][0].normal)

                # make sure they are pointing upwards in the build direction
                np.abs(cross[:,2], out=cross[:,2])
                normals[nPaths, q] = cross

        return normals
        
    def process_paths_normals(self, layer, layer_paths, normal):
        # processing points and normals
        top_plane = self.Meshes.reconstructed_BoundingBoxes[layer].get_plane("top")
        layer_depth = MultiCell()
        for i in range(len(layer_paths)):  # loop over paths for one layer
            for q in range(layer_paths.len_subset(i)):
                # one path and its normals on one layer
                one_path = layer_paths[i,q]
                one_path_normals = normal[i,q]

                d = self.diff(one_path)
                pathLength = self.path_length(d)

                # number of new points in path
                nPoints = int(np.fix(pathLength / self.Clad_Params.MAX_LASER_DEFOCUS))
                if nPoints == 1:
                    # if only one point, take first and end points
                    layer_paths[i,q] = one_path[[0,-1]]
                    normal[i,q] = one_path_normals[[0,-1]]
                else:
                    p, N_indes = interpcurve(nPoints,one_path)
                    layer_paths[i,q] = p
                    normal[i,q] = one_path_normals[N_indes]
            
                # computing depth of points
                point = layer_paths[i,q]
                jsize = len(point)
                path_depths = np.zeros((jsize,1))
                for j in range(jsize):
                    path_depths[j] = top_plane.distance_point_signed(point[j])
        
                layer_depth[i,q] = path_depths

        return layer_paths, normal, layer_depth

    def results(self):
        return self.Paths, self.Depths, self.Normals

    def csv_export(self, outfile):
        # NOTE: may want to convert these variables' coordinates, 
        # depending on desired coord output for the robotic cell
        paths, depths, normals = self.results()

        # Create CSV output file
        header = ['layer_number', 'group', 'point_code','point_depth', 'X','Y','Z','Nx','Ny','Nz']
        with open(outfile, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)

            for layer in range(len(paths)):
                for g in range(len(paths[layer])):
                    for p in range(paths[layer].len_subset(g)):
                        points = paths[layer][g,p]
                        # depth is the distance from point to top plane of bounding box.
                        depth = depths[layer][g,p]
                        normal = normals[layer][g,p]
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

        print('Paths are exported as:', outfile)
