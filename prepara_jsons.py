# %%
import geopandas as gpd
import pandas as pd
import osmnx as ox
import json

G = ox.load_graphml('scenario_maker/Vienna/updated.graphml')
# G = ox.load_graphml('scenario_maker/Rotterdam/updated.graphml')

nodes, edges = ox.graph_to_gdfs(G)

# create a flow group column equal to stroke group and add a speed limit entr
edges['flow_group'] = edges['stroke_group']
edges['speed_limit'] = 30

edges['new_index'] = edges.index.get_level_values('u').astype(str) + '-' + edges.index.get_level_values('v').astype(str)
edges.set_index('new_index', inplace=True)
edges.drop(columns=['geometry'], inplace=True)
edges_dict = edges.to_dict(orient='index')


# Create a new column 'result' with lists of [y, x]
nodes['result'] = nodes.apply(lambda row: [row['y'], row['x']], axis=1)

# Convert the DataFrame to a dictionary where keys are indices and values are 'result' column
result_dict = nodes['result'].to_dict()

# Write the dictionary to a JSON file
with open('scenario_maker/Vienna/nodes.json', 'w') as json_file:
    json.dump(result_dict, json_file, indent=4)

# Write the dictionary to a JSON file
with open('scenario_maker/Vienna/edges.json', 'w') as json_file:
    json.dump(edges_dict, json_file, indent=4)
# %%
