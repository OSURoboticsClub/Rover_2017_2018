import PIL.Image
import math

class MapHelper(object):

    @staticmethod
    def new_image(width, height):
        return PIL.Image.new('RGBA', (width, height))

    @staticmethod
    def fast_round(value, precision):
        return int(value * 10 ** precision) / 10. ** precision

    @staticmethod
    def pixels_to_degrees(pixels, zoom):
        return pixels * 2 ** (21-zoom)

    @staticmethod
    def pixels_to_meters(latitude, zoom):
        # https://groups.google.com/forum/#!topic/google-maps-js-api-v3/hDRO4oHVSeM
        return 2 ** zoom / (156543.03392 * math.cos(math.radians(latitude)))