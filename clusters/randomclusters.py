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

        # set the density weights
        self.low_density_weight = 1.0
        self.medium_density_weight = 1.5
        self.high_density_weight = 2.0

    @timed_function(dt=10)
    def clustering(self):

        if bs.traf.ntraf == 0:
            return
        
        edges_df = bs.traf.TrafficSpawner.edges

        # Here we only randomly apply weights to the graph for replanning
        potential_new_weights = [self.low_density_weight, self.medium_density_weight, self.high_density_weight]
        probabilities = [0.5, 0.25, 0.25]

        random_values = np.random.choice(potential_new_weights, size=len(edges_df), p=probabilities)

        # Apply conditions based on 'density_category'
        edges_df['adjusted_length'] = edges_df['length'] * random_values

        # update the TrafficSpawner graph
        # # Update edge attributes in the graph
        cluster_edge_lengths = {row.Index: row.adjusted_length for row in edges_df.itertuples()}

        for edge_label, adjusted_length in cluster_edge_lengths.items():
            bs.traf.TrafficSpawner.graph[edge_label[0]][edge_label[1]][edge_label[2]]['length'] = adjusted_length 


    @command 
    def SETGRAPHWEIGHTS(self,low_density_weight:float, medium_density_weight:float, high_density_weight:float):
        # set the weights of the graph
        self.low_density_weight = low_density_weight
        self.medium_density_weight = medium_density_weight
        self.high_density_weight = high_density_weight
