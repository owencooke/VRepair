# VRepair
A desktop application written in Python for automated repair path planning of cylindrical metal components in a robotic cell. Supports the following procedure:
1. Defect detection using computer vision technologies/deep learning models.
2. Localization of the defects using a laser time-of-flight sensor, generating 3D point clouds of the damaged surface of a part
3. Path planning for the surface repair of the defects using a robot-assisted laser cladding process

### Source Literature
The majority of the work contributed to this project was done as part of Hamdan Al-Musaibeli's masters' thesis. He originally designed the robotic simulation and Python program used here. He also created the repair path planning program in MATLAB, which Owen converted to Python for integration with the first contribution and designed a GUI for. Hamdan's papers explaining this project are provided below:
- Detection/Localization: [Vision-based spatial damage localization method for autonomous robotic laser cladding repair processes](https://www.sciencedirect.com/science/article/pii/S073658452200134X)
- Repair Path Planning: [A path planning method for surface damage repair using a robot-assisted laser cladding process](https://link.springer.com/article/10.1007/s00170-022-09933-3)

### Key GUI Libraries
- [PySide6](https://wiki.qt.io/Qt_for_Python): the open-source Python-wrapper for building Qt apps. You will want to learn about the QtWidget framework, since that is what this application is built on (how to create QWidgets, add them to the window, create buttons/link them to functions, etc.)
- [QtPy](https://github.com/spyder-ide/qtpy): Instead of using PySide6 directly, I used QtPy, which basically just allows you to switch between Qt versions more easily. If you are looking for docs like QtPy.QWidget, it's equivalent to the docs for whatever Qt version we are using (PySide6, in this case)
- [PyVista](https://docs.pyvista.org/version/stable/index.html): 3D plotting and mesh analysis through a streamlined interface for the Visualization Toolkit (VTK), used to plot the 3D models seen in the GUI.

### Key Backend Libraries
- NumPy (for manipulating the 3D point cloud coordinates)
- [Open3D](http://www.open3d.org/docs/release/) (for loading point cloud files, performing mesh operations, etc.)

### Future Work (IMO)
- Create a realistic simulation workpiece that allows you to demo the whole damage detection, localization, and repair process in one go.
- Replace simulation with actual robotic cell integration (getting data from cell's camera/scanner, and then outputting paths to the cell for laser-cladding)
  > **Note** 
  > Last bullet is very big picture. Getting access to an actual robotic cell or making one in the lab will be difficult. Up to Dr. Rafiq.
- Bundle as a commercial application (might have to fix some absolute path calls in the code for this to be bundled)

## Installation
1. Install CoppeliaSim Edu from [here](https://www.coppeliarobotics.com/downloads.html)
2. Clone the repo, create a Python virtual environment and install all the required dependencies:
  ```bash
  git clone git@github.com:owencooke/VRepair.git
  cd VRepair
  python -m venv .venv
  .venv/Scripts/Activate
  pip install -r requirements.txt
  ```
  > **Note**
  > if using VSCode, you should be able to just open the `requirements.txt` file and there will be a "Create Environment" button that will automatically do it for you.
3. Replace the empty `detect/yolov7_model` folder with the Yolo V7 model downloaded from [here](https://drive.google.com/file/d/1CgfYtPn5pjLpaiEPHAd3aWUaFBpKEa_o/view?usp=sharing).

## Run Method #1: Simulation of Damage Detection/Localization
1. Run CoppeliaSim and open the simulation scene file `simulations/blender_v2.ttt`.
2. Open the VRepair application by running the `app.py` file. 
3. Run the simulation through VRepair by going to "File > Run CoppeliaSim".
  > **Note**
  > If you modify the SIZE of the workpiece in the simulation, make sure you change the parameters defined in the "start_sim" method of `app.py`.
  > More realistic tests (damaged workpieces) can supposedly be created in Blender using a "displacement map", although I never tried this.
5. Set the simulation to its maximum speed by repeatedly pressing the "bunny" button in the top right corner of the CoppeliaSim window.
6. Now, relax and enjoy the show. :smile:
  > **Warning**
  > The detected point cloud is successfully passed to the VRepair app, however you will notice that the detected cloud itself is not as similar to the tests provided in `test_clouds`. 
  > I attempted to create a more realistic simulation workpiece using Blender by using Boolean operations to cut small pieces of the cylinder surface out. You can see that they provided much beter results than the `simulations/original.ttt` file that Hamdan provided me, but there is still improvement to be made towards the realistic surface wear.

### CoppeliaSim Demos
- Inspecting the workpiece for damage:

https://user-images.githubusercontent.com/80288863/230035814-6055d11a-7aa9-4f53-ae6f-a64136830531.mp4

- 3D Scanning of the defective region:

https://user-images.githubusercontent.com/80288863/230035877-b6f8966f-5711-459a-897d-78e4d7ab81e4.mp4

### How to Modify the Simulations With Different Test Workpieces
1. Create a cylinder mesh with the simulated dmaage using a 3D modelling program. I tried using Blender by following this [tutorial](https://youtu.be/kfUmcRwQRps) (only the Boolean operation cut-out part, not textures). Export it to .STL file.
2. Create a copy of one of the scene files in `simulations` directory.
3. To import your .STL, go to "File > Import > Mesh" and click "Import" when the dialog box pops up.
4. Open the "Scene Hierarchy" where it lists all the elements. Find the element called "positioner_tcp". Within it, there should be an element called "workpiece" (which is the old test subject).
5. Activate the "Object Move" tool, select the old "workpiece" element and write down its position in the scene relative to parent.
6. Delete the old "workpiece", rename your new test element "workpiece", and move it into the hierarchy where the old "workpiece" was. Apply the coordinates you wrote down to move the test piece to where the old test was.
7. Double click on the model icon next to your workpiece, it should open a scene properties dialog. Click "Texture > Select texture from existing textures" and there should be a "tiledConcrete" texture you can add to give your model some texture for the image detection.
8. Done! Should be able to run the new test simulation exactly as before. 

## Run Method #2: Repair Path Planning on Test Point Clouds
Since the simulated point cloud result is buggy, you can test the repair path planning process using the point clouds provided in the `test_clouds` directory. I would recommend `FBH.pcd`, it is the one I used for the majority of my testing.

1. Open the VRepair application by running the `app.py` file. 
2. Load desired test (File > Load PointCloud)
3. Perform each "Path Planning" step (Triangulate Mesh > Slice Mesh > Generate Toolpaths)
4. The resulting toolpaths can then be exported into CSV format (File > Export Paths to CSV)

The resulting toolpath information can then be used to direct the robotic cell to perform the laser-cladding repair process. Supposedly the paths can be loaded into robot simulation software such as RoboDK, although I never had the chance to try this part. 

### Repair Demo
The point cloud loaded here is what the simulated point cloud **should** look like

https://user-images.githubusercontent.com/90405643/233168941-8c0ec87f-bc0b-4bc2-9372-51b0996423e4.mp4

### After Exporting the Repair Toolpaths
I never got to test or work with this step, but below I will share the information that Hamdan shared with me:
1. Download RoboDK, another robot simulation software.
2. Open the `simulations/repair_workcell.rdk`. It is only the first version of the RDK cell and is still missing the laser head. You can search for a CAD file of any laser head and attach it to the robot.
3. We use the inbuilt functionality for the "path following project" to allow the robot cell to follow the generated toolpaths and complete the repair process.

