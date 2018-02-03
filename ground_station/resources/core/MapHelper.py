import PIL.Image

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