#!/usr/bin/env python
"""
Sample script that uses the PathPlannerTSPSolver_3D module created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""

import PathPlannerTSPSolver_3D
# Import the matlab module only after you have imported
# MATLAB Compiler SDK generated Python modules.
import matlab

my_PathPlannerTSPSolver_3D = PathPlannerTSPSolver_3D.initialize()

start_posIn = matlab.double([53.95375719839131, -1.0404824359104623, 100.0], size=(1, 3))
goal_posIn = matlab.double([53.94537235295549, -1.0177802188084115, 100.0], size=(1, 3))
polygon_verticesIn = matlab.double([53.95375719839131, 53.953479414248854, 53.955802642820856, 53.95610566309161, 53.95251978206389, 53.94810013827757, 53.94537235295549, 53.94403365231053, 53.94630689209062, 53.94802436887196, 53.948504239449356, 53.950019583972534, 53.95375719839131, 53.95375719839131, -1.0404824359104623, -1.0314702136204232, -1.017866049496888, -1.012201224057435, -1.0120295626804818, -1.0145615679905404, -1.0177802188084115, -1.0391819005778586, -1.0445463186076438, -1.0405122762492454, -1.0390531545451438, -1.0408770566752708, -1.0434519773295676, -1.0404824359104623, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0], size=(14, 3))
wind_directionIn = matlab.double([-1.5707963267948966], size=(1, 1))
altitude_limitsIn = {"min": matlab.double([50.0], size=(1, 1)), "max": matlab.double([70.0], size=(1, 1))}
uav_turning_radiusIn = matlab.double([100.0], size=(1, 1))
uav_airspeedIn = matlab.double([20.0], size=(1, 1))
num_divisionsIn = {"x": matlab.double([12.0], size=(1, 1)), "y": matlab.double([12.0], size=(1, 1)), "z": matlab.double([3.0], size=(1, 1))}
plot_resultsIn = matlab.logical([True], size=(1, 1))
coordinate_pathOut, dubins_path_waypointsOut, total_path_costOut = my_PathPlannerTSPSolver_3D.setupTSP(start_posIn, goal_posIn, polygon_verticesIn, wind_directionIn, altitude_limitsIn, uav_turning_radiusIn, uav_airspeedIn, num_divisionsIn, plot_resultsIn, nargout=3)
print(coordinate_pathOut, dubins_path_waypointsOut, total_path_costOut, sep='\n')

my_PathPlannerTSPSolver_3D.terminate()
