# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------------
# UAV Pathfinder
# Advanced GIS Model Builder Project
# Trevor Stanhope
# Takes inputs for hazard features to generate waypoint-based flight plans for
# safely navigating UAV systems to pick-up and drop-off points in populated areas.
# ---------------------------------------------------------------------------

# Set the necessary product code
# import arcinfo

# Import arcpy module
import arcpy, os, json
from datetime import datetime
import numpy as np

# Load Settings file
settings_file = arcpy.GetParameterAsText(0)
if settings_file == '':
    settings_file = "settings.txt"
with open(settings_file, 'r') as jsonfile:
    settings = json.loads(jsonfile.read())

# Useful function definitions
def log_error(err, msg):
    date = datetime.strftime(datetime.now(), "%Y-%m-%d %H:%M:%S")
    error_path = os.path.join(settings['log_path'], settings['error_file'])
    log_entry = "%s\t%s\t%s" % (date, msg, str(err))
    print log_entry
    with open(error_path, 'a') as errorfile:
        errorfile.write(log_entry + '\n')
        
def pretty_print(header, msg):
    date = datetime.strftime(datetime.now(), "%Y-%m-%d %H:%M:%S")
    log_entry = "%s\t%s\t%s" % (date, header, msg)
    print log_entry
    
def find_route_length(waypoints):
    length = 0
    for p in range(len(waypoints)-1):
        (x1, y1) = waypoints[p]
        (x2, y2) = waypoints[p+1]
        length += np.hypot(x2-x1, y2-y1)
    return length

# Setting Environment Variables
pretty_print('SETUP', "Setting environment variables ... ")
arcpy.CheckOutExtension("spatial")
arcpy.env.extent = "-71.539412701456 46.5201343421344 -70.960812701456 47.2218901654276"
arcpy.env.overwriteOutput = True

# Setting Input Variables
pretty_print('SETUP', "Setting input variables ...")
pipelines = os.path.join(settings['features_path'], settings['pipelines_file'])
pipeline_buffer_dist = settings['pipeline_buffer'] # "1 Kilometers"
pickup_point = os.path.join(settings['features_path'], settings['pickup_file'])
railyway_buffer_dist = settings['railway_buffer'] # "1 Kilometers"
railways = os.path.join(settings['features_path'], settings['railways_file'])
start_point = os.path.join(settings['features_path'], settings['start_file'])
dem = os.path.join(settings['dem_path'], settings['dem_file'])
temp = settings['temp_path']
roads = os.path.join(settings['features_path'], settings['roads_file'])
road_buffer_dist = settings['road_buffer'] # "0.1 Kilometers"
dropoff_point = os.path.join(settings['features_path'], settings['dropoff_file'])

# Setting Temporary Variables
pretty_print('SETUP', "Setting temporary variables ...")
pipeline_buffer_shp = os.path.join(settings['temp_path'], "pipeline_buffer.shp")
dropoff_as_pt_shp = os.path.join(settings['temp_path'], 'dropoff_as_pt.shp')
pipeline_rast = os.path.join(settings['temp_path'], 'pipeline_rast')
start_as_pt_shp = os.path.join(settings['temp_path'], 'start_as_pt.shp')
weighted_over = os.path.join(settings['temp_path'], 'weighted_over')
cost_dist = os.path.join(settings['temp_path'], 'cd')
cost_backlink = os.path.join(settings['temp_path'], 'cl')
cost_path = os.path.join(settings['temp_path'], 'cp')
rail_buffer_shp = os.path.join(settings['temp_path'], 'rail_buffer.shp')
rail_rast = os.path.join(settings['temp_path'], 'rail_rast')
dem_mosaic = os.path.join(settings['temp_path'], 'dem_mosaic')
dem_reclass = os.path.join(settings['temp_path'], 'dem_reclass')
road_buffer_shp = os.path.join(settings['temp_path'], 'road_buffer.shp')
road_raster = os.path.join(settings['temp_path'], 'road_raster')
cost_polyline = os.path.join(settings['temp_path'], 'pl')
polyline_points = os.path.join(settings['temp_path'], 'wp')
route_polyline = os.path.join(settings['output_path'], 'route')

# Convert Shapes to Points
pretty_print("SETUP", "Converting start to points ...")
arcpy.FeatureToPoint_management(start_point, start_as_pt_shp, "CENTROID")
arcpy.AddXY_management(start_as_pt_shp)

pretty_print("SETUP", "Converting dropoffs to points ...")
arcpy.FeatureToPoint_management(dropoff_point, dropoff_as_pt_shp, "CENTROID")
arcpy.AddXY_management(dropoff_as_pt_shp)

pretty_print("SETUP", "Converting dropoffs to points ...")
arcpy.AddXY_management(pickup_point)

# Create Hazards Map
pretty_print("SETUP", "Buffering railways ...")
arcpy.Buffer_analysis(railways, rail_buffer_shp, railyway_buffer_dist, "FULL", "ROUND", "NONE", "")
pretty_print("SETUP", "Converting railway buffer to raster ...")
arcpy.PolygonToRaster_conversion(rail_buffer_shp, "ATG", rail_rast, "CELL_CENTER", "NONE", "0.00094")
pretty_print("SETUP", "Buffering pipelines ...")
arcpy.Buffer_analysis(pipelines, pipeline_buffer_shp, pipeline_buffer_dist, "FULL", "ROUND", "NONE", "")
pretty_print("SETUP", "Converting pipeline buffer to raster ...")
arcpy.PolygonToRaster_conversion(pipeline_buffer_shp, "ATG", pipeline_rast, "CELL_CENTER", "NONE", "0.001")
pretty_print("SETUP", "Buffering roads ...")
arcpy.Buffer_analysis(roads, road_buffer_shp, road_buffer_dist, "FULL", "ROUND", "NONE", "")
pretty_print("SETUP", "Converting roads buffer to raster ...")
arcpy.PolygonToRaster_conversion(road_buffer_shp, "ATG", road_raster, "CELL_CENTER", "NONE", "0.001")
pretty_print("SETUP", "Reclassifying DEM ...")
arcpy.gp.Reclassify_sa(dem, "VALUE", "0 47 1;47 116 2;116 175 3;175 221 4;221 272 5;272 329 6;329 392 7;392 473 8;473 641 9;641 65535 10", dem_reclass, "DATA")
pretty_print("SETUP", "Generating weighted overlay ... ")
arcpy.gp.WeightedOverlay_sa("('C:\\Users\\Trevor\\Documents\\ArcGIS\\Projects\\Model Builder Project\\temp\\rail_rast' 25 'VALUE' (1 10;NODATA 0); 'C:\\Users\\Trevor\\Documents\\ArcGIS\\Projects\\Model Builder Project\\temp\\pipeline_rast' 25 'VALUE' (1 10;NODATA 0); 'C:\\Users\\Trevor\\Documents\\ArcGIS\\Projects\\Model Builder Project\\temp\\dem_reclass' 25 'VALUE' (1 1; 2 2; 3 3; 4 4; 5 5; 6 6; 7 7; 8 8; 9 9; 10 10;NODATA NODATA); 'C:\\Users\\Trevor\\Documents\\ArcGIS\\Projects\\Model Builder Project\\temp\\road_raster' 25 'VALUE' (1 1;NODATA 0));0 10 1", weighted_over)

# Cost distance from start point
pretty_print("S0", "Generating cost distance from START ...")
cost_dist_from_s = cost_dist + '_s'
cost_backlink_from_s = cost_backlink + '_s'
arcpy.gp.CostDistance_sa(start_as_pt_shp, weighted_over, cost_dist_from_s, "", cost_backlink_from_s)

# Cost Distance from each pick-up point
for pickup_row in arcpy.SearchCursor(pickup_point):
    try:
        pickup_fid = pickup_row.getValue('FID')
        pickup_x = pickup_row.getValue('POINT_X')
        pickup_y = pickup_row.getValue('POINT_Y')
        pickup_pt = arcpy.Point(pickup_x, pickup_y)
        pickup_geom = arcpy.PointGeometry(pickup_pt)
        
        cost_dist_from_p = cost_dist + '_p%d' % pickup_fid
        cost_backlink_from_p = cost_backlink + '_p%d' % pickup_fid
        
        pretty_print("P%d" % pickup_fid, "Generating cost distance from PICKUP --> %s" % cost_dist_from_p)
        arcpy.gp.CostDistance_sa(pickup_geom, weighted_over, cost_dist_from_p, "", cost_backlink_from_p)
    except Exception as error:
        log_error(error.message, "P%d" % pickup_fid)
    
# Cost Distance and Path from each drop-off point       
for dropoff in arcpy.SearchCursor(dropoff_as_pt_shp):
    
    try:
        dropoff_fid = dropoff.getValue('FID')
        dropoff_x = dropoff.getValue('POINT_X')
        dropoff_y = dropoff.getValue('POINT_Y')
        dropoff_pt = arcpy.Point(dropoff_x, dropoff_y)
        dropoff_geom = arcpy.PointGeometry(dropoff_pt)
        
        cost_dist_from_d = cost_dist + '_d%d' % dropoff_fid
        cost_backlink_from_d = cost_backlink + '_d%d' % dropoff_fid
        cost_path_d_to_s = cost_path + '_d%d_s' % dropoff_fid
        cost_polyline_d_to_s = cost_polyline + '_d%d_s.shp' % dropoff_fid
        polyline_points_d_to_s = polyline_points + '_d%d_s.shp' % dropoff_fid
        
        pretty_print("D%d" % dropoff_fid, "Generating cost distance from DROPOFF --> %s" % cost_dist_from_d)
        arcpy.gp.CostDistance_sa(dropoff_geom, weighted_over, cost_dist_from_d, "", cost_backlink_from_d)
        
        pretty_print("D%d" % dropoff_fid, "Generating cost path DROPOFF TO START --> %s" % cost_path_d_to_s)
        arcpy.gp.CostPath_sa(start_as_pt_shp, cost_dist_from_d, cost_backlink_from_d, cost_path_d_to_s, "EACH_CELL")
        
        pretty_print("D%d" % dropoff_fid, "Converting dropoff cost path to polyline --> %s" % cost_polyline_d_to_s)
        arcpy.RasterToPolyline_conversion(cost_path_d_to_s, cost_polyline_d_to_s, "ZERO", "0", "SIMPLIFY", "VALUE")

        pretty_print("D%d" % dropoff_fid, "Converting polyline to points --> %s" % polyline_points_d_to_s)
        arcpy.FeatureVerticesToPoints_management(cost_polyline_d_to_s, polyline_points_d_to_s, "ALL")
        
        pretty_print("D%d" % dropoff_fid, "Adding XY data to points ...")
        arcpy.AddXY_management(polyline_points_d_to_s)      
    except Exception as error:
        log_error(error.message, "D%d" % dropoff_fid)
        
# List all silos
pretty_print("", "Finding optimal paths ... ")
all_routes = {}
for pickup_row in arcpy.SearchCursor(pickup_point):

    try:
        pickup_fid = pickup_row.getValue('FID')
        pickup_x = pickup_row.getValue('POINT_X')
        pickup_y = pickup_row.getValue('POINT_Y')
        pickup_pt = arcpy.Point(pickup_x, pickup_y)
        pickup_geom = arcpy.PointGeometry(pickup_pt)
        pretty_print("P%d" % pickup_fid, "Pickup at %f, %f" % (pickup_x, pickup_y))
                
        cost_dist_from_p = cost_dist + '_p%d' % pickup_fid
        cost_backlink_from_p = cost_backlink + '_p%d' % pickup_fid
        cost_path_s_to_p = cost_path + '_s_p%d' % pickup_fid
        cost_polyline_s_to_p = cost_polyline + '_s_p%d.shp' % pickup_fid
        polyline_points_s_to_p = polyline_points + '_s_p%d.shp' % pickup_fid
        
        pretty_print("P%d" % pickup_fid, "Generating cost path START TO PICKUP --> %s" % cost_path_s_to_p)
        arcpy.gp.CostPath_sa(pickup_geom, cost_dist_from_s, cost_backlink_from_s, cost_path_s_to_p, "EACH_CELL")
        
        pretty_print("P%d" % pickup_fid, "Converting cost path to polyline --> %s" % cost_polyline_s_to_p)
        arcpy.RasterToPolyline_conversion(cost_path_s_to_p, cost_polyline_s_to_p, "ZERO", "0", "SIMPLIFY", "VALUE")

        pretty_print("P%d" % pickup_fid, "Converting polyline vertices to points --> %s" % polyline_points_s_to_p)
        arcpy.FeatureVerticesToPoints_management(cost_polyline_s_to_p, polyline_points_s_to_p, "ALL")

        pretty_print("P%d" % pickup_fid, "Adding XY data to points ...")
        arcpy.AddXY_management(polyline_points_s_to_p)
        
    except Exception as error:
        msg = "P%d" % pickup_fid
        log_error(error.message, msg)
        
    # Process: Lumber Yard to Point
    all_routes[str(pickup_fid)] = {}
    for dropoff in arcpy.SearchCursor(dropoff_as_pt_shp):
        
        try:
            dropoff_fid = dropoff.getValue('FID')
            dropoff_x = dropoff.getValue('POINT_X')
            dropoff_y = dropoff.getValue('POINT_Y')
            dropoff_pt = arcpy.Point(dropoff_x, dropoff_y)
            dropoff_geom = arcpy.PointGeometry(dropoff_pt)
            pretty_print("P%d D%d" % (pickup_fid, dropoff_fid), "Dropoff at %f, %f" % (dropoff_x, dropoff_y))
            
            cost_path_p_to_d = cost_path + '_p%d_to_d%d' % (pickup_fid, dropoff_fid)
            cost_polyline_p_to_d = cost_polyline + '_p%d_d%d.shp' % (pickup_fid, dropoff_fid)
            cost_polyline_d_to_s = cost_polyline + '_d%d_s.shp' % dropoff_fid
            cost_path_d_to_s = cost_path + '_d%d_s' % dropoff_fid
            polyline_points_p_to_d = polyline_points + '_p%d_d%d.shp' % (pickup_fid, dropoff_fid)
            
            pretty_print("P%d D%d" % (pickup_fid, dropoff_fid),  "Generating cost path from PICKUP --> %s" % cost_path_p_to_d)
            arcpy.gp.CostPath_sa(dropoff_geom, cost_dist_from_p, cost_backlink_from_p, cost_path_p_to_d, "EACH_CELL")

            pretty_print("P%d D%d" % (pickup_fid, dropoff_fid), "Converting cost path to polyline --> %s" % cost_polyline_p_to_d)
            arcpy.RasterToPolyline_conversion(cost_path_p_to_d, cost_polyline_p_to_d, "ZERO", "0", "SIMPLIFY", "VALUE")

            pretty_print("P%d D%d" % (pickup_fid, dropoff_fid), "Converting polyline vertices to points --> %s" % polyline_points_p_to_d)
            arcpy.FeatureVerticesToPoints_management(cost_polyline_p_to_d, polyline_points_p_to_d, "ALL")

            pretty_print("P%d D%d" % (pickup_fid, dropoff_fid), "Adding XY data to points ...")
            arcpy.AddXY_management(polyline_points_p_to_d)
            
            pretty_print("P%d D%d" % (pickup_fid, dropoff_fid), "Generating waypoints ... ")
            waypoints = []
            for point in arcpy.SearchCursor(polyline_points_s_to_p):
                x = point.getValue('POINT_X')
                y = point.getValue('POINT_Y')
                waypoints.append((x,y))
            for point in arcpy.SearchCursor(polyline_points_p_to_d):
                x = point.getValue('POINT_X')
                y = point.getValue('POINT_Y')
                waypoints.append((x,y))
            for point in arcpy.SearchCursor(polyline_points_d_to_s):
                x = point.getValue('POINT_X')
                y = point.getValue('POINT_Y')
                waypoints.append((x,y))
 
            pretty_print("P%d D%d" % (pickup_fid, dropoff_fid), "Adding route to dictionary ...")
            all_routes[str(pickup_fid)][str(dropoff_fid)] = waypoints
            
        except Exception as error:
            msg = "P%d D%d" % (pickup_fid, dropoff_fid)
            log_error(error.message, msg)

# Find Best routes
pretty_print('', "Finding best route for each pickup location ...")
best_routes = {}
for p in all_routes:
    try:
        best_routes[p] = None
        shortest_route = 1000
        for d in all_routes[p]:
            waypoints = all_routes[p][d]
            l = find_route_length(waypoints)
            if (shortest_route > l):
                shortest_route = l
                best_routes[p] = d
        pretty_print('', "For Pickup #%s, the route via Dropoff #%s is shortest" % (p, best_routes[p])) 
    except Exception as error:
        log_error(error.message, '')

# Create output layers of for each of the best routes
for p in best_routes:
    for d in best_routes[p]:
        cost_polyline_s_to_p = cost_polyline + '_s_p%s.shp' % p
        cost_polyline_p_to_d = cost_polyline + '_p%s_d%s.shp' % (p, d)
        cost_polyline_d_to_s = cost_polyline + '_d%s_s.shp' % d
        route_polyline_p_d = route_polyline + '_p%s_d%s.shp' % (p, d)
        arcpy.Merge_management([cost_polyline_s_to_p, cost_polyline_p_to_d, cost_polyline_d_to_s], route_polyline_p_d)
        
# Write waypoints to file
pretty_print("CSV", "Writing waypoints to file")
for pickup_fid in best_routes:
    try:
        dropoff_fid = best_routes[pickup_fid]
        log_file = 'waypoints_p%s_d%s.csv' % (pickup_fid, dropoff_fid)
        log_path = os.path.join(settings['log_path'], log_file)
        with open(log_path, 'w') as csvfile:
            waypoints = all_routes[pickup_fid][dropoff_fid]
            for (x,y) in waypoints:
                csvfile.write(','.join([str(x),str(y),'\n']))
    except Exception as error:
        log_error(error.message, '')
