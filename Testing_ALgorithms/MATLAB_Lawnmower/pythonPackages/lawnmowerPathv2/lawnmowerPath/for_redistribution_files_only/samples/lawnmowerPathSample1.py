#!/usr/bin/env python
"""
Sample script that uses the lawnmowerPath module created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""

import lawnmowerPath
# Import the matlab module only after you have imported
# MATLAB Compiler SDK generated Python modules.
import matlab

my_lawnmowerPath = lawnmowerPath.initialize()

areaWidthIn = matlab.double([100.0], size=(1, 1))
areaHeightIn = matlab.double([200.0], size=(1, 1))
swathWidthIn = matlab.double([20.0], size=(1, 1))
startXIn = matlab.double([0.0], size=(1, 1))
startYIn = matlab.double([0.0], size=(1, 1))
waypointsOut = my_lawnmowerPath.lawnmowerPath(areaWidthIn, areaHeightIn, swathWidthIn, startXIn, startYIn)
print(waypointsOut, sep='\n')

my_lawnmowerPath.terminate()
