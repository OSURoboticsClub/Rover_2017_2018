# Note that the lat and lon positions below correspond to the center point of the maps you want to download
# Proper zoom level selection determines total viewable area

MAPPING_LOCATIONS = {
    "Graf Hall": {
        "latitude": 44.5675721667,
        "longitude": -123.2750535,
        "default_zoom": 18,
        "valid_zoom_options": [15, 16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    },

    "Crystal Lake": {
        "latitude": 44.547155,
        "longitude": -123.251438,
        "default_zoom": 18,
        "valid_zoom_options": [15, 16, 17, 18, 19, 20],
        "pre_cache_map_zoom_levels": [18, 19, 20]
    }
}