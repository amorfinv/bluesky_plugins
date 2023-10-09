import json
import numpy as np
from collections import Counter
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.spatial import KDTree
import networkx as nx

import bluesky as bs
from bluesky import core, stack, traf, scr, sim
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools import geo
from bluesky.core import Entity, Replaceable
from bluesky.traffic import Route
from bluesky.tools.misc import degto180, txt2tim, txt2alt, txt2spd, lat2txt

from plugins.utils import pluginutils

# # create classes
# edge_traffic = None
# path_plans = None

def init_plugin():
    
    global path_plans, edge_traffic

    # # Initialize EdgeTraffic
    # edge_traffic = EdgeTraffic(bs.settings.graph_dir)

    # # Initialize Path Plans
    # path_plans = PathPlans(bs.settings.graph_dir)

    config = {
        'plugin_name'      : 'flowcontrol',
        'plugin_type'      : 'sim',
        # 'update':          update,
        # 'reset':           reset
        }
    
    return config

######################## FLOW CONTROL FUNCTIONS #########################

@core.timed_function(dt=10)
def do_flowcontrol():

    aircraft_to_replan()

def aircraft_to_replan():
    
    # TODO: choose stroke groups to check replan
    # TODO: select only from current edge to end
    # select which aircraft should replan

    acid_to_replan = []

    for acidx, acid in enumerate(bs.traf.id):
        unique_edges = bs.traf.TrafficSpawner.unique_edges[acidx]

        if np.any(np.isin(unique_edges,bs.traf.clustering.cluster_edges)):
            acid_to_replan.append(acid)

    

# @core.timed_function(dt=10)
# def checkclu():

