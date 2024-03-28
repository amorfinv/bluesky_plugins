import geopandas as gpd
import pandas as pd
import numpy as np
from copy import deepcopy
from collections import defaultdict
import pickle

# open demand file
demand_polygons = gpd.read_file("Rotterdam/normalized_demand.gpkg")

# open airspace polygon and get the airspace boundary
nodes = gpd.read_file("Rotterdam/updated.gpkg", layer='nodes')
nodes.set_index(['osmid'], inplace=True)
nodes = nodes.to_crs(epsg='28992')

# perform spatial join
nodes_join = gpd.sjoin(nodes, demand_polygons, how='left', predicate='intersects')
nodes_check = deepcopy(nodes_join)

# loop through nodes that don't have
indices = []
indices_right = []
normalized_demands = []
parcels_sum = []
new_demands = []
for row in nodes_join.itertuples():
    if not pd.isna(row.index_right):
        continue
    
    # get the point of the value wit nan
    point_geom = row.geometry

    # this row
    nearest_distance = 1000

    # loop again through points to check nearest
    for row_2 in nodes_check.itertuples():
        if pd.notna(row_2.index_right):
            distance = point_geom.distance(row_2.geometry)
            if distance < nearest_distance:
                nearest_idx = row_2.index_right
                normalized_demand = row_2.normalized_demand
                parcel_sum = row_2.Parcels_sum
                nearest_distance = distance
                new_demand = row.new_demand
    
    indices_right.append(nearest_idx)
    normalized_demands.append(normalized_demand)
    indices.append(row.Index)
    parcels_sum.append(parcel_sum)
    new_demands.append(new_demand)

# replace the geodatframe with this value
nodes_join.loc[indices, 'index_right'] = indices_right
nodes_join.loc[indices, 'normalized_demand'] = normalized_demands
nodes_join.loc[indices, 'Parcels_sum'] = parcels_sum
nodes_join.loc[indices, 'new_demand'] = new_demands


# load the origin and destination pickle
with open('Rotterdam/orig_dest_dict_demand.pickle', 'rb') as f:
    orig_dest_dict = pickle.load(f)

flipped_dict = defaultdict(list)
for key, values in orig_dest_dict.items():
    for value in values:
        flipped_dict[value].append(key)

# get the destinations
destinations = list(flipped_dict.keys())

# get the origins
origins = list(orig_dest_dict.keys())

# remove them from node_gdf
nodes_join = nodes_join.drop(origins, axis=0)


#  ok now we create a new column to give actual probability without origins
# we do this by dividing the normalized_demand with the number of nodes with that demand
location_counts = nodes_join.groupby('index_right').size()

# # # get actual demand
# nodes_join['split_demand'] = nodes_join.apply(lambda row: row.normalized_demand / location_counts[row['index_right']],  axis=0)
nodes_join['split_demand'] = nodes_join.apply(lambda row: row.normalized_demand/location_counts[row.index_right] ,  axis=1)

# save the nodes with the updated demand
nodes_join.to_file('Rotterdam/node_demands.gpkg')
