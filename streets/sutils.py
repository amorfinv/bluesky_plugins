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
    # add first node to route
    lons = np.array(G.nodes[route[0]]["x"])
    lats = np.array(G.nodes[route[0]]["y"])
    edges = [f'{route[0]}-{route[1]}']

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
    turn_bool = np.array([False] * len(lats), dtype=np.bool8)
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
        _, d1 = geo.qdrdist(lat_prev, lon_prev, lat_cur, lon_cur)
        _, d2 = geo.qdrdist(lat_cur, lon_cur, lat_next, lon_next)

        # fix angles that are larger than 180 degrees
        angle = abs(d2 - d1)
        angle = 360 - angle if angle > 180 else angle

        # give the turn speeds based on the angle
        if angle > cutoff_angle and i != 0:

            # set turn bool to true and get the turn coordinates
            turn_bool[i] = True
            turn_coords[i] = (lat_cur, lon_cur)

            # calculate the turn speed based on the angle.
            if angle < 100:
                turn_speed[i] = 10
            elif angle < 150:
                turn_speed[i] = 5
            else:
                turn_speed[i] = 2
        else:
            turn_coords[i] = (-9999.9, -9999.9)

        # update the previous values at the end of the loop
        lat_prev = lat_cur
        lon_prev = lon_cur

    return turn_bool, turn_speed, turn_coords

""" 
THE CODE BELOW WAS TAKEN FROM OSMNX AND MODIFIED TO WORK WITH BLUESKY 
See: https://github.com/gboeing/osmnx for more details.
"""

def graph_to_gdfs(G):
    """
    Adapted from osmnx code: https://github.com/gboeing/osmnx
    Convert a MultiDiGraph to node and edge DataFrames.
    Parameters
    ----------
    G : networkx.MultiDiGraph
        input graph
    Returns
    -------
    pandas.GeoDataFrame tuple
        gdf_nodes and gdf_edges
    """

    # create node dataframe
    nodes, data = zip(*G.nodes(data=True))

    # convert node x/y attributes to Points for geometry column
    geom = (Point(d["x"], d["y"]) for d in data)
    df_nodes = pd.DataFrame(data, index=nodes)
    df_nodes['geometry'] = list(geom)

    df_nodes.index.rename("osmid", inplace=True)

    # create edge dataframe 
    u, v, k, data = zip(*G.edges(keys=True, data=True))

    # subroutine to get geometry for every edge: if edge already has
    # geometry return it, otherwise create it using the incident nodes
    x_lookup = nx.get_node_attributes(G, "x")
    y_lookup = nx.get_node_attributes(G, "y")

    def make_geom(u, v, data, x=x_lookup, y=y_lookup):
        if "geometry" in data:
            return data["geometry"]
        else:
            return LineString((Point((x[u], y[u])), Point((x[v], y[v]))))

    geom = map(make_geom, u, v, data)
    df_edges = pd.DataFrame(data)
    df_edges['geometry'] = list(geom)

    # add u, v, key attributes as index
    df_edges["u"] = u
    df_edges["v"] = v
    df_edges.set_index(["u", "v"], inplace=True)

    # convert to geodataframe
    gdf_nodes = gpd.GeoDataFrame(df_nodes, geometry="geometry", crs="epsg:4326")
    gdf_edges = gpd.GeoDataFrame(df_edges, geometry="geometry", crs="epsg:4326")

    return gdf_nodes, gdf_edges


def load_graphml(filepath, node_dtypes=None, edge_dtypes=None, graph_dtypes=None):
    """
    Load an OSMnx-saved GraphML file from disk.

    This converts node, edge, and graph-level attributes (serialized as
    strings) to their appropriate data types. These can be customized as
    needed by passing in dtypes arguments providing types or custom converter
    functions. For example, if you want to convert some attribute's values to
    `bool`, consider using the built-in `ox.io._convert_bool_string` function
    to properly handle "True"/"False" string literals as True/False booleans:
    `ox.load_graphml(fp, node_dtypes={my_attr: ox.io._convert_bool_string})`

    If you manually configured the `all_oneway=True` setting, you may need to
    manually specify here that edge `oneway` attributes should be type `str`.

    Parameters
    ----------
    filepath : string or pathlib.Path
        path to the GraphML file
    node_dtypes : dict
        dict of node attribute names:types to convert values' data types. the
        type can be a python type or a custom string converter function.
    edge_dtypes : dict
        dict of edge attribute names:types to convert values' data types. the
        type can be a python type or a custom string converter function.
    graph_dtypes : dict
        dict of graph-level attribute names:types to convert values' data
        types. the type can be a python type or a custom string converter
        function.

    Returns
    -------
    G : networkx.MultiDiGraph
    """
    filepath = Path(filepath)

    # specify default graph/node/edge attribute values' data types
    default_graph_dtypes = {"simplified": _convert_bool_string}
    default_node_dtypes = {
        "elevation": float,
        "elevation_res": float,
        "lat": float,
        "lon": float,
        "osmid": int,
        "street_count": int,
        "x": float,
        "y": float,
    }
    default_edge_dtypes = {
        "bearing": float,
        "grade": float,
        "grade_abs": float,
        "length": float,
        "oneway": _convert_bool_string,
        "osmid": int,
        "speed_kph": float,
        "travel_time": float,
    }

    # override default graph/node/edge attr types with user-passed types, if any
    if graph_dtypes is not None:
        default_graph_dtypes.update(graph_dtypes)
    if node_dtypes is not None:
        default_node_dtypes.update(node_dtypes)
    if edge_dtypes is not None:
        default_edge_dtypes.update(edge_dtypes)

    # read the graphml file from disk
    G = nx.read_graphml(filepath, node_type=default_node_dtypes["osmid"], force_multigraph=True)

    # convert graph/node/edge attribute data types
    G = _convert_graph_attr_types(G, default_graph_dtypes)
    G = _convert_node_attr_types(G, default_node_dtypes)
    G = _convert_edge_attr_types(G, default_edge_dtypes)

    return G

def _convert_graph_attr_types(G, dtypes=None):
    """
    Convert graph-level attributes using a dict of data types.

    Parameters
    ----------
    G : networkx.MultiDiGraph
        input graph
    dtypes : dict
        dict of graph-level attribute names:types

    Returns
    -------
    G : networkx.MultiDiGraph
    """
    # remove node_default and edge_default metadata keys if they exist
    G.graph.pop("node_default", None)
    G.graph.pop("edge_default", None)

    for attr in G.graph.keys() & dtypes.keys():
        G.graph[attr] = dtypes[attr](G.graph[attr])

    return G


def _convert_node_attr_types(G, dtypes=None):
    """
    Convert graph nodes' attributes using a dict of data types.

    Parameters
    ----------
    G : networkx.MultiDiGraph
        input graph
    dtypes : dict
        dict of node attribute names:types

    Returns
    -------
    G : networkx.MultiDiGraph
    """
    for _, data in G.nodes(data=True):
        for attr in data.keys() & dtypes.keys():
            data[attr] = dtypes[attr](data[attr])
    return G


def _convert_edge_attr_types(G, dtypes=None):
    """
    Convert graph edges' attributes using a dict of data types.

    Parameters
    ----------
    G : networkx.MultiDiGraph
        input graph
    dtypes : dict
        dict of edge attribute names:types

    Returns
    -------
    G : networkx.MultiDiGraph
    """
    # for each edge in the graph, eval attribute value lists and convert types
    for _, _, data in G.edges(data=True, keys=False):

        # remove extraneous "id" attribute added by graphml saving
        data.pop("id", None)

        # first, eval stringified lists to convert them to list objects
        # edge attributes might have a single value, or a list if simplified
        for attr, value in data.items():
            if value.startswith("[") and value.endswith("]"):
                try:
                    data[attr] = ast.literal_eval(value)
                except (SyntaxError, ValueError):
                    pass

        # next, convert attribute value types if attribute appears in dtypes
        for attr in data.keys() & dtypes.keys():
            if isinstance(data[attr], list):
                # if it's a list, eval it then convert each item
                data[attr] = [dtypes[attr](item) for item in data[attr]]
            else:
                # otherwise, just convert the single value
                data[attr] = dtypes[attr](data[attr])

        # if "geometry" attr exists, convert its well-known text to LineString
        if "geometry" in data:
            data["geometry"] = wkt.loads(data["geometry"])

    return G

def _convert_bool_string(value):
    """
    Convert a "True" or "False" string literal to corresponding boolean type.

    This is necessary because Python will otherwise parse the string "False"
    to the boolean value True, that is, `bool("False") == True`. This function
    raises a ValueError if a value other than "True" or "False" is passed.

    If the value is already a boolean, this function just returns it, to
    accommodate usage when the value was originally inside a stringified list.

    Parameters
    ----------
    value : string {"True", "False"}
        the value to convert

    Returns
    -------
    bool
    """
    if value in {"True", "False"}:
        return value == "True"
    elif isinstance(value, bool):
        return value
    else:  # pragma: no cover
        raise ValueError(f'invalid literal for boolean: "{value}"')


def nearest_nodes(kdtree, node_gdf, X, Y):
    """
    Find the nearest node to a point or to each of several points.

    If `X` and `Y` are single coordinate values, this will return the nearest
    node to that point. If `X` and `Y` are lists of coordinate values, this
    will return the nearest node to each point.

    If the graph is projected, this uses a k-d tree for euclidean nearest
    neighbor search, which requires that scipy is installed as an optional
    dependency. If it is unprojected, this uses a ball tree for haversine
    nearest neighbor search, which requires that scikit-learn is installed as
    an optional dependency.

    Parameters
    ----------
    G : networkx.MultiDiGraph
        graph in which to find nearest nodes
    X : float or list
        points' x (longitude) coordinates, in same CRS/units as graph and
        containing no nulls
    Y : float or list
        points' y (latitude) coordinates, in same CRS/units as graph and
        containing no nulls
    return_dist : bool
        optionally also return distance between points and nearest nodes

    Returns
    -------
    nn or (nn, dist) : int/list or tuple
        nearest node IDs or optionally a tuple where `dist` contains distances
        between the points and their nearest nodes
    """
    is_scalar = False
    if not (hasattr(X, "__iter__") and hasattr(Y, "__iter__")):
        # make coordinates arrays if user passed non-iterable values
        is_scalar = True
        X = np.array([X])
        Y = np.array([Y])

    # drop the geometry column if it exists
    nodes = node_gdf[["x", "y"]]
    dist, pos = kdtree.query(np.array([X, Y]).T, k=1)
    nn = nodes.index[pos]

    # convert results to correct types for return
    nn = nn.tolist()
    dist = dist.tolist()
    if is_scalar:
        nn = nn[0]
        dist = dist[0]
        
    return nn

