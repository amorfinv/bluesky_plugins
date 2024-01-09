import numpy as np
from scipy.cluster.vq import whiten
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, Point
import json
import geopandas as gpd
import pandas as pd
from pyproj import Transformer
import matplotlib.pyplot as plt

import bluesky as bs
from bluesky.core.simtime import timed_function
from bluesky import core
from bluesky.tools import datalog
from bluesky.tools.aero import nm
from bluesky.tools import areafilter as af
from bluesky.stack import command
from plugins.clusters import cluster_algorithms as calgos

clusterheader = \
    '#######################################################\n' + \
    'Cluster log LOG\n' + \
    'Cluster density statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'Conflict count [-], ' + \
    'Geometry [wkt], ' + \
    'Summed edged length [m], ' + \
    'Cluster linear densities  [m], ' + \
    'Cluster area densities [-]\n'

clustering = None

def init_plugin():
    global clustering

    # Addtional initilisation code
    bs.traf.clustering = Clustering()
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'RANDOMCLUSTERING',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class Clustering(core.Entity):
    def __init__(self):
        super().__init__() 



    @timed_function(dt=10)
    def clustering(self):

        print('clustering')

        if bs.traf.ntraf == 0:
            return
        
        edges_df = bs.traf.TrafficSpawner.edges

        # Here we only randomly apply weights to the graph for replanning
        potential_new_weights = [1, 1.5 , 2]
        probabilities = [0.5, 0.25, 0.25]

        random_values = np.random.choice(potential_new_weights, size=len(edges_df), p=probabilities)

        # Apply conditions based on 'density_category'
        edges_df['adjusted_length'] = edges_df['length'] * random_values

        # update the TrafficSpawner graph
        # # Update edge attributes in the graph
        edge_lengths = {row.Index: row.adjusted_length for row in edges_df.itertuples()}

        for edge_label, adjusted_length in edge_lengths.items():
            bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length'] = adjusted_length 




