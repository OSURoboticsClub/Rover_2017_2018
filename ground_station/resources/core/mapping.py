'''
Mapping.py: Objected Orientated Google Maps for Python
ReWritten by Chris Pham

Copyright OSURC, orginal code from GooMPy by Alec Singer and Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.

'''

#####################################
# Imports
#####################################
# Python native imports
import math
import urllib2
from io import StringIO, BytesIO
import os
import time
import PIL.Image

#####################################
# Constants
#####################################
<<<<<<< 666f522bf5947794d802e8cbbf221ac004fc4eed
fp = open('key', w)
_KEY = fp.read().rstrip()
fp.close()
=======
file_pointer = open('key', 'w')
_KEY = file_pointer.read().rstrip()
file_pointer.close()
>>>>>>> fixed fp naming conventions

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


class GMapsStitcher(object):
    def __init__(self, width, height,
                 latitude, longitude, zoom,
                 maptype, radius_meters=None, num_tiles=4):
        self.latitude = latitude
        self.longitude = longitude
        self.width = width
        self.height = height
        self.zoom = zoom
        self.maptype = maptype
        self.radius_meters = radius_meters
        self.num_tiles = num_tiles

    def _new_image(self):
        return PIL.Image.new('RGB', (self.width, self.height))

    def _fast_round(self, value, precision):
        return int(value * 10 ** precision) / 10. ** precision

    def _pixels_to_degrees(self, pixels, zoom):
        return pixels * 2 ** (21-zoom)

    def _grab_tile(self, sleeptime=0):
        # Make the url string for polling
        # GET request header gets appended to the string
        urlbase = 'https://maps.googleapis.com/maps/api/staticmap?'
        urlbase += 'center=%f%f&zoom=%d&maptype=%s&size=%dx%d&format=jpg&key=%s'

        # Fill the formatting
        specs = self.latitude, self.longitude, self.zoom, self.maptype, _TILESIZE, _KEY
        filename = 'Resources/Maps/' + ('%f_%f_%d_%s_%d_%d_%s' % specs) + '.jpg'

        # Tile Image object
        tile_object = None

        if os.path.isfile(filename):
            tile_object = PIL.Image.open(filename)

        # If file on filesystem
        else:
            # make the url
            url = urlbase % specs

            result = urllib2.Request(url).read()
            tile_object = PIL.Image.open(BytesIO(result))
            if not os.path.exists('Resources/Maps'):
                os.mkdir('Resources/Maps')
            tile_object.save(filename)
            #Added to prevent timeouts on Google Servers
            time.sleep(sleeptime)

        return tile_object

    def _pixels_to_lon(self, iterator, lon_pixels):
        # Magic Lines, no idea
        degrees = self._pixels_to_degrees(((iterator) - self.num_tiles / 2) * _TILESIZE, self.zoom)
        return math.degrees((lon_pixels + degrees - _EARTHPIX) / _pixrad)

    def _pixels_to_lat(self, iterator, lat_pixels):
        # Magic Lines
        degree = self._pixels_to_degrees((iterator - self.num_tiles / 2) * _TILESIZE, self.zoom)
        return math.degrees(math.pi / 2 - 2 * math.atan(math.exp(((lat_pixels + degree) - _EARTHPIX) / _pixrad)))
