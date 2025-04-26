#!/usr/bin/env python
"""
Sample script that uses the TSPSolver module created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""

import TSPSolver
# Import the matlab module only after you have imported
# MATLAB Compiler SDK generated Python modules.
import matlab

my_TSPSolver = TSPSolver.initialize()

polygon_verticesIn = matlab.double([53.950974807525206, 53.946024591620926, 53.945898302917456, 53.95089904334211, 53.950974807525206, -1.0329672619323844, -1.033224753997814, -1.0243412777404899, -1.0251566692810172, -1.0329672619323844, 50.0, 50.0, 50.0, 50.0, 50.0], size=(5, 3))
coordinate_pathOut, dubins_path_waypointsOut = my_TSPSolver.setupTSP(polygon_verticesIn, nargout=2)
print(coordinate_pathOut, dubins_path_waypointsOut, sep='\n')

my_TSPSolver.terminate()
