import sys

import dgpython_azurekinect.kinect
from .kinect import K4A

# Assign kinect symbols into our namespace for convenience
for key in kinect.syms:
    globals()[key] = kinect.syms[key]
    pass


