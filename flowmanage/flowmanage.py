from random import randint
import numpy as np

import bluesky as bs
from bluesky import core, stack, traf  
from bluesky.tools import aero, areafilter, geo

# bs.settings.set_variable_defaults(geofence_dtlookahead=30)

def init_plugin():
    config = {
        'plugin_name': 'flowmanage',
        'plugin_type': 'sim',
    }
    return config


'''

TODO: decide for vienna or orthogonal city?

PLugin format

Plugin A:

    - create traffic patterns based on a point distribution
    - in first implementation the origin/destination points are attempted to be uniformly distributed as much as possible
    - not always possible due to the city arrangement?
    - perhaps work on 
    - in future create different patterns
    - types of missions
        1) spawn: constrained airspace. delete: open airspace
        2) spawn: open airspace. delete: constrained airspace
        3) spawn: constrained airspace. delete: constrained airsapce
        4) spawn: open airspace. delete: open airspace
    - all missions should go through constrained airspace so remove them
    - maintain a given number of aircraft in the air
    - spawn aircraft above their origin at cruise speed
    - delete aircraft above their destination. 

Plugin B:

    - This plugin should aggregate geo-statistcs every so often
    - statistics should conflicts/intrusions/density
    - conflicts intrusions should have 'memory' perhaps statistics are saved for the last 10 minutes or so

Plugin C

    - identify clusters based on data
    - use k-means or other strategies

Plugin D

    - modify the weight of the graph given the live cluster analysis. this plugin should be active every 10 seconds or so

Plugin E

    - given the new graph weights identify which aircraft should attempt a replan and then replan


Considerations: use cuda graphs instead of networkx if you can get ahold of a gpu?


Future:
    - add wind for weights of the graph
    - add battery consumption for graph
    - consider non-uniform point distribution..converging or diverging traffic flows
    - use ML methods for clustering
    - use ML methods for graph weight setting

'''
# what should plugin do
# 1 spawn aircraft
# 2 types of spawning