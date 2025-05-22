# MEng Project - Autonomous Path Planner for Aerial Photography

The path planning application aims to improve the methodologies of path planning systems, primarily taking into account constraints such as lowest cost altitude adjustments and environmental conditions for energy performance, reducing the overall need for users to interact and intervene with the UAV whilst planning out a mission.

### Code and Implementation Files
The main application should be run from the PathPlannerPy folder, where `PathPlannerApp.py` is the main entry point to the program. 
Code within the PathPlanner_MATLAB folder contains all the back-end algorithms and flight path calculation code. This has been exported as a python package, which is called from the python implementation utilising the MATLAB Engine API.

### How To Run
- Ensure you have a valid installation of the MATLAB Engine API, or a full installation of the MATLAB runtime environment.
- Ensure you have python 3.12 installed (Version 3.12.2 was used in this project)
- This project was developed in PyCharm so should be runnable directly after opening the `PathPlannerPy` folder and project files in PyCharm.
- If PyCharm is not available, ensure the venv is activated, and then run the python file (`PathPlannerApp.py`) as normal via the terminal.
- If there are errors related to the MATLAB Engine API or similar issues, this will be due to not having the correct version of MATLAB Engine or runtime environment installed. An installer can be found in `/PathPlanner_MATLAB/PythonPackage/PathPlannerTSPSolver_3D/for_redistribution` which should install the relevant API required. 

### Running The MATLAB Algorithms Inidivdually
The MATLAB code can be executed separately using the `setTSPSample.m` file, which contains a number of example configurations, including flat terrain, a gentle slope, and more mountainous terrain. By modifying the variables input into the `setupTSP.m` function, these different configurations can be experimented with.
