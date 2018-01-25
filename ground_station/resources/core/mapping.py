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
import signing

#####################################
# Constants
#####################################
_KEYS = []

# Number of pixels in half the earth's circumference at zoom = 21
_EARTHPIX = 268435456
# Number of decimal places for rounding coordinates
_DEGREE_PRECISION = 4
# Larget tile we can grab without paying
_TILESIZE = 640
# Fastest rate at which we can download tiles without paying
_GRABRATE = 4
# Pixel Radius of Earth for calculations
_PIXRAD = _EARTHPIX / math.pi

file_pointer = open('key', 'r')
for i in file_pointer:
    _KEYS.append(i.rstrip())
file_pointer.close()

print _KEYS


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

    def _new_image(self, width, height):
        return PIL.Image.new('RGB', (width, height))

    def _fast_round(self, value, precision):
        return int(value * 10 ** precision) / 10. ** precision

    def _pixels_to_degrees(self, pixels, zoom):
        return pixels * 2 ** (21-zoom)

    def _pixels_to_meters(self):
          # https://groups.google.com/forum/#!topic/google-maps-js-api-v3/hDRO4oHVSeM
        return 2 ** self.zoom / (156543.03392 * math.cos(math.radians(self.latitude)))

    def _grab_tile(self, longitude, latitude, sleeptime=0):
        # Make the url string for polling
        # GET request header gets appended to the string
        urlbase = 'https://maps.googleapis.com/maps/api/staticmap?'
        urlbase += 'center=%f,%f&zoom=%d&maptype=%s&size=%dx%d&format=jpg&key=%s'

        # Fill the formatting
        specs = latitude, longitude, self.zoom, self.maptype, _TILESIZE, _TILESIZE, _KEYS[0]
        filename = 'Resources/Maps/' + ('%f_%f_%d_%s_%d_%d_%s' % specs) + '.jpg'

        # Tile Image object
        tile_object = None

        if os.path.isfile(filename):
            tile_object = PIL.Image.open(filename)

        # If file on filesystem
        else:
            # make the url
            url = urlbase % specs
            print url
            url = signing.sign_url(url, _KEYS[1])
            print url
            result = urllib2.urlopen(urllib2.Request(url)).read()
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
        return math.degrees((lon_pixels + degrees - _EARTHPIX) / _PIXRAD)

    def _pixels_to_lat(self, iterator, lat_pixels):
        # Magic Lines
        return math.degrees(math.pi / 2 - 2 * math.atan(
            math.exp(((lat_pixels + self._pixels_to_degrees((iterator - self.num_tiles / 2) * _TILESIZE, self.zoom)) - _EARTHPIX) / _PIXRAD)))

    def fetch_tiles(self):
        # cap floats to precision amount
        self.latitude = self._fast_round(self.latitude, _DEGREE_PRECISION)
        self.longitude = self._fast_round(self.longitude, _DEGREE_PRECISION)

        # number of tiles required to go from center latitude to desired radius in meters
        if self.radius_meters is not None:
            self.num_tiles = int(round(2*self._pixels_to_meters() / (_TILESIZE / 2. / self.radius_meters)))

        lon_pixels = _EARTHPIX + self.longitude * math.radians(_PIXRAD)

        sin_lat = math.sin(math.radians(self.latitude))
        lat_pixels = _EARTHPIX - _PIXRAD * math.log((1+sin_lat)/(1-sin_lat))/2
        big_size = self.num_tiles * _TILESIZE

        big_image = self._new_image(big_size, big_size)

        for j in range(self.num_tiles):
            lon = self._pixels_to_lon(j, lon_pixels)
            for k in range(self.num_tiles):
                lat = self._pixels_to_lat(k, lat_pixels)
                tile = self._grab_tile(lon, lat)
                big_image.paste(tile, (j * _TILESIZE, k * _TILESIZE))

        big_image.save("testimage.jpg")
