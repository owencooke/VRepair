import time
from wear_rebuild import WearRebuild
from clad_params import CladParams

print("Processing ")
start = time.time()

# Instantiate cladding parameters
cladding = CladParams(1,                     # clad height (mm)
                        10,                     # clad width  (mm)
                        5,                     # max clad step (mm)  (clads overlap is considered)
                        0,                     # clad orientation (degrees)
                        0)                # clad cross hatching ( enable or disable )

# Run wear rebuild program
model = WearRebuild('data/FBH.pcd',           # filename
                        cladding,
                        0,                     # layer offset (e.g. 0.5) (0-1)
                        3,                     # max laser defocus (mm)
                        'crust',               # meshing algorithm ( 'delaunay', 'crust' )
                        1,                     # mesh refinement factor (int)
                        'cyl')                  # point cloud cooridante


end = time.time()
print(end-start, "seconds")

# post-processing section
# model.display_pcd;
# model.display_raw_mesh;
# model.display_reconst_meshes;
# model.display_toolpaths;

# exporting data:
# # model.pcd_export('global', 'cyl')     # (to_system, to_coordinate)
# # model.mesh_export('global', 'cyl')     
# # model.paths_export('global', 'cyl')  