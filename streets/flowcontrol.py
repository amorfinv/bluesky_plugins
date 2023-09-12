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

from plugins.streets.streets import edge_traffic, flight_layers, path_plans, use_flow_control

######################## FLOW CONTROL FUNCTIONS #########################

@core.timed_function(dt=10)
def do_flowcontrol():

    if use_flow_control:

        edge_count_dict = dict(Counter(edge_traffic.actedge.wpedgeid))
        group_count_dict = dict(Counter(edge_traffic.actedge.group_number)) 
        flow_count_dict = dict(Counter(edge_traffic.actedge.flow_number))