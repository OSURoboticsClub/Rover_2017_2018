import PIL.Image

class MapHelper(object):
    def __init__(self):
        return

    def new_image(self, width, height):
        return PIL.Image.new('RGBA', (width, height))

    def fast_round(self, value, precision):
        return int(value * 10 ** precision) / 10. ** precision
