#####################################
# Imports
#####################################
# Python native imports
import math
import urllib
import PIL.Image
from io import StringIO, BytesIO
import os
import time

#####################################
# Constants
#####################################
file_pointer = open('key', 'w')
_KEY = file_pointer.read().rstrip()
file_pointer.close()

# Number of pixels in half the earth's circumference at zoom = 21
_EARTHPIX = 268435456
# Number of decimal places for rounding coordinates
_DEGREE_PRECISION = 4 
# Larget tile we can grab without paying
_TILESIZE = 640
# Fastest rate at which we can download tiles without paying
_GRABRATE = 4
# Pixel Radius of Earth for calculations
_pixrad = _EARTHPIX / math.pi