#!/usr/bin/python

import mapping
import PIL

obj = mapping.GMapsStitcher(2000, 2000, 44.567161, -123.278432, 18, 'terrain', None, 20)

obj.display_image.save("unzoomed.png")
# draw = PIL.ImageDraw.ImageDraw(obj.big_image)
# draw.rectangle([950, 950, 1050, 1050], fill=128)
# lat, lng = obj.move_latlon(44.559919, -123.280723)
# draw.rectangle([lng-300, lat-300, lng+300, lat+300], fill=328)
# obj.big_image.save("toobig.jpg")
# obj.display_image.save("zoomed.jpg")

obj.add_gps_location(44.559919, -123.280723, "square", 50, (225,225,225,225))
obj.add_gps_location(44.565094, -123.276110, "square", 50, (225,225,225,225))
obj.add_gps_location(44.565777, -123.278902, "square", 50, (225,225,225,225))
obj.display_image.save("box.png")
obj.center_display(44.567161, -123.278432)
obj.display_image.save("centered.png")
obj.big_image.save("toobig.png")

obj.obj_print()
