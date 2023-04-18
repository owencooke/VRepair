# VRepair
A desktop application for automated repair path planning of cylindrical metal components in a robotic cell. Supports the following procedure:
1. Defect detection using computer vision technologies/deep learning models.
2. Localization of the defects using a laser time-of-flight sensor, generating 3D point clouds of the damaged surface of a part
3. Path planning for the surface repair of the defects using a robot-assisted laser cladding process

### Source Literature
- Detection/Localization: [Vision-based spatial damage localization method for autonomous robotic laser cladding repair processes](https://www.sciencedirect.com/science/article/pii/S073658452200134X)
- Repair Path Planning: [A path planning method for surface damage repair using a robot-assisted laser cladding process](https://link.springer.com/article/10.1007/s00170-022-09933-3)

### Future Work
- Fix issue with simulated point cloud result (only storing one curve of points, not multiple). Occuring somewhere in detect module. Simulated point cloud result should look similar to the tests provided in `test_clouds`
- Replace simulation with actual robotic cell integration (getting data from cell's camera/scanner, and then outputting paths to the cell for laser-cladding)
  > **Note** 
  > Last bullet is very big picture. Getting access to an actual robotic cell or making one in the lab will be difficult. Up to Dr. Rafiq.

## Installation
1. Install CoppeliaSim (Player or Edu) from [here](https://www.coppeliarobotics.com/downloads.html)
2. Create a Python virtual environment and install all the required dependencies:
  ```batch
  python -m venv .venv
  .venv/Scripts/Activate
  pip install -r requirements.txt
  ```
  > **Note**
  > if using VSCode, you should be able to just open the `requirements.txt` file and there will be a "Create Environment" button that will do this for you.

## Run Method #1: Simulation of Damage Detection/Localization
1. Run CoppeliaSim and open the simulation scene file `robot_cell_sim.ttt`.
2. Open the VRepair application by running the `app.py` file. 
3. Run the simulation through VRepair by going to File menu and clicking "Run CoppeliaSim".
  > **Note**
  > If you modify the workpiece in the simulation, make sure you change the parameters defined in the "start_sim" method of `app.py`.
  > More realistic tests (damaged workpieces) can supposedly be created in Blender using a "displacement map", although I never tried this.
5. Set the simulation to its maximum speed by repeatedly pressing the "bunny" button in the top right corner of the CoppeliaSim window.
6. Now, relax and enjoy the show. :smile:
  > **Warning**
  > The detected point cloud is successfully passed to the VRepair app, however you will notice that the detected cloud itself is not correct.
  > The graduate student who developed the detection/localization believes there might be a bug in the scanning part of the program. It seems to me as if its only outputting one of the scanned lines of 3D points, not all of them. Load `test_clouds/FBH.pcd` into VRepair for an example of how the simulated scan **should** look.

### What CoppeliaSim Should Look Like
- Inspecting the workpiece for damage:

https://user-images.githubusercontent.com/80288863/230035814-6055d11a-7aa9-4f53-ae6f-a64136830531.mp4

- 3D Scanning of the defective region:

https://user-images.githubusercontent.com/80288863/230035877-b6f8966f-5711-459a-897d-78e4d7ab81e4.mp4

## Run Method #2: Repair Path Planning on Test Point Clouds
Since the simulated point cloud result is buggy, you can test the repair path planning process using the point clouds provided in the `test_clouds` directory. I would recommend `FBH.pcd`, it is the one I used for the majority of my testing.

1. Open the VRepair application by running the `app.py` file. 
2. Load desired test (File > Load PointCloud)
3. Perform each "Path Planning" step (Triangulate Mesh > Slice Mesh > Generate Toolpaths)
4. The resulting toolpaths can then be exported into CSV format (File > Export Paths to CSV)

The resulting toolpath information can then be used to direct the robotic cell to perform the laser-cladding repair process. Supposedly the paths can be loaded into robot simulation software such as RoboDK, although I never had the chance to try this part. 
