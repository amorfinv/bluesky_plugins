from pathlib import Path
import ast

import numpy as np
import geopandas as gpd
from shapely import wkt
from shapely.geometry import LineString, Point
import networkx as nx
import pandas as pd

import bluesky as bs
from bluesky.tools import geo
from bluesky.core.varexplorer import varlist

def lat_lon_from_nx_route(G, route):
    """
    Get lat lon, and edges from a route (list of nodes) and Graph.
    The function returns two numpy arrays with the lat and lon of route.
    Also return a shapely linestring.
    Parameters
    ----------
    G : nx.MultiGraph or nx.MultiDiGraph
        Graph to get lat and lon from. Graph should be built
        with osmnx.get_undirected.
    route : list
        List of nodes to build edge and to get lat lon from
    Returns
    -------
    lat : numpy.ndarray
        Array with latitudes of route.
    lon : numpy.ndarray
        Array with longitudes of route
    edges : list
        List of edges that are part of the route.
    linestring : shapely.LineString.
        LineString with lat and lon of route.
    """
    # initalize variables
    lons = np.array([])
    lats = np.array([])
    edges = []

    # loop through the rest for loop only adds from second point of edge
    for u, v in zip(route[:-1], route[1:]):
        # if there are parallel edges, select the shortest in length
        data = list(G.get_edge_data(u, v).values())[0]

        # extract coords from linestring
        xs, ys = data["geometry"].xy

        # Check if geometry of edge is in correct order
        if G.nodes[u]["x"] != data["geometry"].coords[0][0]:

            # flip if in wrong order
            xs = xs[::-1]
            ys = ys[::-1]
        
        edges += [f'{u}-{v}']*len(xs[1:])
        lons = np.append(lons, xs[1:])
        lats = np.append(lats, ys[1:])

    # make a linestring from the coords
    linestring = LineString(zip(lons, lats))
    
    return lats, lons, edges, linestring

def get_turn_arrays(lats, lons, cutoff_angle=25):
    """
    Get turn arrays from latitude and longitude arrays.
    The function returns three arrays with the turn boolean, turn speed and turn coordinates.
    Turn speed depends on the turn angle.
        - Speed set to 0 for no turns.
        - Speed is 10 knots for angles between 25 and 100 degrees.
        - Speed is 5 knots for angles between 100 and 150 degrees.
        - Speed is 2 knots for angles larger than 150 degrees.
    Parameters
    ----------
    lat : numpy.ndarray
        Array with latitudes of route
    lon : numpy.ndarray
        Array with longitudes of route
    cutoff_angle : int
        Cutoff angle for turning. Default is 25.
    Returns
    -------
    turn_bool : numpy.ndarray
        Array with boolean values for turns.
    turn_speed : numpy.ndarray
        Array with turn speed. If no turn, speed is 0.
    turn_coords : numpy.ndarray
        Array with turn coordinates. If no turn then it has (-9999.9, -9999.9)
    """

    # Define empty arrays that are same size as lat and lon
    turn_speed = np.zeros(len(lats))
    turn_bool = np.array([False] * len(lats), dtype=np.bool_)
    turn_coords = np.array([(-9999.9, -9999.9)] * len(lats), dtype="f,f")

    # Initialize variables for the loop
    lat_prev = lats[0]
    lon_prev = lons[0]

    # loop thru the points to calculate turn angles
    for i in range(1, len(lats) - 1):
        # reset some values for the loop
        lat_cur = lats[i]
        lon_cur = lons[i]
        lat_next = lats[i + 1]
        lon_next = lons[i + 1]

        # calculate angle between points
        a1, _ = geo.qdrdist(lat_prev, lon_prev, lat_cur, lon_cur)
        a2, _ = geo.qdrdist(lat_cur, lon_cur, lat_next, lon_next)

        # fix angles that are larger than 180 degrees
        angle = abs(a1 - a2)
        angle = 360 - angle if angle > 180 else angle

        # give the turn speeds based on the angle
        if angle > cutoff_angle and i != 0:

            # set turn bool to true and get the turn coordinates
            turn_bool[i] = True
            turn_coords[i] = (lat_cur, lon_cur)

            # calculate the turn speed based on the angle.
            turn_speed[i] = 5

        else:
            turn_coords[i] = (-9999.9, -9999.9)

        # update the previous values at the end of the loop
        lat_prev = lat_cur
        lon_prev = lon_cur

    return turn_bool, turn_speed, turn_coords

def access_plugin_object(plugin_name):
    """
    Access a BlueSky plugin module by name. This can be used to access
    the objects in a plugin module.
    Parameters
    ----------
    plugin_name : str
    Returns
    -------
    plugin module : module
        The plugin module object.
    """
    # first make everything lowercase
    plugin_name = plugin_name.lower()

    # get the plugin module
    try:
        return varlist[plugin_name][0]

    except:
        bs.scr.echo(f'Plugin {plugin_name} not found')
        return
