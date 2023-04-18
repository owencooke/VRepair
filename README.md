# VRepair
A desktop application for automated repair path planning of cylindrical metal components in a robotic cell. Supports the following procedure:
1. defect detection using computer vision technologies/deep learning models.
2. localization of the defects using a laser time-of-flight sensor, generating 3D point clouds of the damaged surface of a part
3. path planning for the surface repair of the defects using a robot-assisted laser cladding process

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

## Run Simulation of Damage Detection/Localization
1. Run CoppeliaSim and open the simulation scene file `robot_cell_sim.ttt`.
  > **Note**
  > Don't start the simulation from CoppeliaSim, it will be started automatically from the VRepair app. 
  > If you modify the workpiece in the simulation, make sure you change the parameters defined in the "start_sim" method of the `main.py` file.
2. Open the VRepair application by running the `main.py` file. 
3. Run the simulation through VRepair by going to File menu and clicking "Run CoppeliaSim".
4. Make sure you speed up the simulation to its maximum speed by repeatedly pressing the "bunny" button in the top right corner of the CoppeliaSim window.
5. Now, relax and enjoy the show. :smile:
  > **Warning**
  > The detected point cloud should be passed to the VRepair app, however you will notice that it is not correct.
  > The graduate student who developed the detection/localization believes there might be a bug in the scanning part of the program. It seems to me as if its only outputting one of the lines of 3D points, not all of the lines.

## Run Sample
- Inspecting workpiece step:
https://user-images.githubusercontent.com/80288863/230035814-6055d11a-7aa9-4f53-ae6f-a64136830531.mp4

- 3D Scanning of the defective regoin step:
https://user-images.githubusercontent.com/80288863/230035877-b6f8966f-5711-459a-897d-78e4d7ab81e4.mp4

## Run Repair Path Planning on Test Point Clouds
Since the simulated point cloud result is buggy, you can test the repair path planning process using the point clouds provided in the `test_clouds` directory. I would recommend `FBH.pcd`, it is the one I used for the majority of my testing.

1. Open the VRepair application by running the `main.py` file. 
2. Load desired test (File > Load PointCloud)
3. Perform each "Path Planning" step (Triangulate Mesh > Slice Mesh > Generate Toolpaths)
4. The resulting toolpaths can then be exported into CSV format (File > Export Paths to CSV)

The resulting toolpath information can then be used to direct the robotic cell to perform the laser-cladding repair process.