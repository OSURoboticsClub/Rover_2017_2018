import mapping

obj = mapping.GMapsStitcher(2000, 2000, 44.57078, -123.275998, 18, 'terrain', None, 20)

obj.useZoom(1.2)

obj.display_image.save("display_image.jpg")
