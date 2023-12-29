
def map_coord(x, y, Map ):
    real_x = (x*Map.filter_size - Map.center_before_reducing) * Map.resolution_before_reducing
    real_y = (y*Map.filter_size - Map.center_before_reducing) * Map.resolution_before_reducing
    return [real_x, real_y, 0]
 
def box_coord(x, y, Map ):
    box_x = int( x / Map.resolution_before_reducing) + Map.center_before_reducing
    box_y = int( y / Map.resolution_before_reducing) + Map.center_before_reducing
    return [int(box_x/Map.filter_size), int(box_y/Map.filter_size)]
