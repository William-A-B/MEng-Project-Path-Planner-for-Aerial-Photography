#!/usr/bin/env python
"""
Sample script that uses the PathPlanner2D module created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""

import PathPlanner2D
# Import the matlab module only after you have imported
# MATLAB Compiler SDK generated Python modules.
import matlab

my_PathPlanner2D = PathPlanner2D.initialize()

polygon_verticesIn = matlab.double([53.950974807525206, 53.946024591620926, 53.945898302917456, 53.95089904334211, 53.950974807525206, -1.0329672619323844, -1.033224753997814, -1.0243412777404899, -1.0251566692810172, -1.0329672619323844], size=(5, 2))
waypoint_pathOut, square_cornersOut = my_PathPlanner2D.PathPlanner2D(polygon_verticesIn, nargout=2)
print(waypoint_pathOut, square_cornersOut, sep='\n')

my_PathPlanner2D.terminate()
