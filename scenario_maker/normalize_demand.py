import geopandas as gpd


# open demand file
demand_polygons = gpd.read_file("demand.gpkg")

# open airspace polygon and get the airspace boundary
airspace_polygon = gpd.read_file("airspace.gpkg")
airspace_polygon = airspace_polygon.to_crs(epsg='28992')
airspace_boundary = airspace_polygon.geometry.boundary
airspace_boundary = gpd.GeoDataFrame(geometry=[airspace_boundary.values[0]], crs='EPSG:28992')

# # get intersection between demand polygons and airspace polygons
# intersecting_polygons = gpd.sjoin(demand_polygons, airspace_boundary, how="inner", predicate="intersects")

# Iterate over each polygon
new_demands = []
for row in demand_polygons.itertuples():

    # get geometry
    demand_polygon = row.geometry
    airspace = airspace_polygon.geometry.values[0]

    # Get difference the polygon by the LineString
    difference_polygon = demand_polygon.difference(airspace)

    # get demand_polygon_area and difference area
    demand_polygon_area = demand_polygon.area
    difference_area = difference_polygon.area

    # get ratio of area that falls outside airspace
    outside_area_ratio = difference_area/demand_polygon_area

    # skip cases where outside ratio is less than 10 percent
    if outside_area_ratio < 0.1:
        new_demands.append(row.Parcels_sum)
        continue
    
    # reduce the demand level in the area by this value
    demand_reduction_ratio = 1 - outside_area_ratio

    # multiply the demand by this value
    new_demand = round(row.Parcels_sum * demand_reduction_ratio)

    new_demands.append(new_demand)

# assign new_demands
demand_polygons['new_demand'] = new_demands

# normalize
total_demand = demand_polygons['new_demand'].sum()
demand_polygons['normalized_demand'] = demand_polygons['new_demand'] / total_demand


# save file
demand_polygons.to_file('normalized_demand.gpkg')

