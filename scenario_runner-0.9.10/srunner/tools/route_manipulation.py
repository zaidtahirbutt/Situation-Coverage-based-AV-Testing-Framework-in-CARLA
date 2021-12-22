#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Labs.
# authors: German Ros (german.ros@intel.com), Felipe Codevilla (felipe.alcm@gmail.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Module to manipulate the routes, by making then more or less dense (Up to a certain parameter).
It also contains functions to convert the CARLA world location do GPS coordinates.
"""

import math
import xml.etree.ElementTree as ET

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from agents.navigation.local_planner import RoadOption
import sys


def _location_to_gps(lat_ref, lon_ref, location):
    """
    Convert from world coordinates to GPS coordinates
    :param lat_ref: latitude reference for the current map
    :param lon_ref: longitude reference for the current map
    :param location: location to translate
    :return: dictionary with lat, lon and height
    """

    EARTH_RADIUS_EQUA = 6378137.0   # pylint: disable=invalid-name
    scale = math.cos(lat_ref * math.pi / 180.0)
    mx = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
    my = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat_ref) * math.pi / 360.0))
    mx += location.x
    my -= location.y

    lon = mx * 180.0 / (math.pi * EARTH_RADIUS_EQUA * scale)
    lat = 360.0 * math.atan(math.exp(my / (EARTH_RADIUS_EQUA * scale))) / math.pi - 90.0
    z = location.z

    return {'lat': lat, 'lon': lon, 'z': z}


def location_route_to_gps(route, lat_ref, lon_ref):
    """
        Locate each waypoint of the route into gps, (lat long ) representations.
    :param route:
    :param lat_ref:
    :param lon_ref:
    :return:
    """
    gps_route = []

    for transform, connection in route:
        gps_point = _location_to_gps(lat_ref, lon_ref, transform.location)

        #print(connection)  # RoadOption.LANEFOLLOW
        #print(gps_point)  # {'lat': -0.0016101365263807565, 'lon': -3.104682434048109e-05, 'z': 0.0}
        #sys.exit("it")

        gps_route.append((gps_point, connection))

    return gps_route


def _get_latlon_ref(world):
    """
    Convert from waypoints world coordinates to CARLA GPS coordinates
    :return: tuple with lat and lon coordinates
    """
    xodr = world.get_map().to_opendrive()  # Returns the .xodr OpenDRIVe file of the current map as string.

    tree = ET.ElementTree(ET.fromstring(xodr))

    # default reference
    lat_ref = 42.0
    lon_ref = 2.0

    for opendrive in tree.iter("OpenDRIVE"):
        for header in opendrive.iter("header"):
            for georef in header.iter("geoReference"):
                if georef.text:
                    str_list = georef.text.split(' ')
                    for item in str_list:
                        if '+lat_0' in item:
                            lat_ref = float(item.split('=')[1])
                        if '+lon_0' in item:
                            lon_ref = float(item.split('=')[1])
    return lat_ref, lon_ref


def downsample_route(route, sample_factor):
    """
    Downsample the route by some factor.
    :param route: the trajectory , has to contain the waypoints and the road options
    :param sample_factor: Maximum distance between samples
    :return: returns the ids of the final route that can
    """

    ids_to_sample = []
    prev_option = None
    dist = 0

    for i, point in enumerate(route):
        curr_option = point[1]

        # Lane changing
        if curr_option in (RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT):
            ids_to_sample.append(i)
            dist = 0

        # When road option changes
        elif prev_option != curr_option and prev_option not in (RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT):
            ids_to_sample.append(i)
            dist = 0

        # After a certain max distance
        elif dist > sample_factor:
            ids_to_sample.append(i)
            dist = 0

        # At the end
        elif i == len(route) - 1:
            ids_to_sample.append(i)
            dist = 0

        # Compute the distance traveled
        else:
            curr_location = point[0].location
            prev_location = route[i - 1][0].location
            dist += curr_location.distance(prev_location)

        prev_option = curr_option

    return ids_to_sample


def interpolate_trajectory(world, waypoints_trajectory, hop_resolution=1.0):  # gps_route, route = interpolate_trajectory(world, config.trajectory)  # in config.trajectory we have  # carla location waypoints here from the routes_debug.xml routes> id = 0, town= Town02 xml!
    """
        Given some raw keypoints interpolate a full dense trajectory to be used by the user.  # meaning the route we have in the routes_drbug.xml file has many sparsly populated waypoints throughout the map and we will densly populate them for our AV to drive on with hop_res of 1. :3 
    :param world: an reference to the CARLA world so we can use the planner
    :param waypoints_trajectory: the current coarse trajectory
    :param hop_resolution: is the resolution, how dense is the provided trajectory going to be made
    :return: the full interpolated route both in GPS coordinates and also in its original form.
    """

    dao = GlobalRoutePlannerDAO(world.get_map(), hop_resolution)  # hop_resolution=1.0



    grp = GlobalRoutePlanner(dao)



    grp.setup()  # THIS WAS DAMNNNNNNNNNNNNNN HEAVY  # Graph is built in this bish. Of the whole map, with nodes and edges and heavily populated waypoints and attributes of the edges with vectors and length as cost in them and zero cost for lane changes and we get info from carla server for lane changes and everything else also
    # Obtain route plan
    route = []  # so this is where we extract the ROUTE plan out of the WHOLE densly populated graph we built.
    for i in range(len(waypoints_trajectory) - 1):   # Goes until the one before the last. # gps_route, route = interpolate_trajectory(world, config.trajectory)  # in config.trajectory we have  # carla location waypoints here from the routes_debug.xml routes> id = 0, town= Town02 xml!

        waypoint = waypoints_trajectory[i]
        waypoint_next = waypoints_trajectory[i + 1]
        interpolated_trace = grp.trace_route(waypoint, waypoint_next)  # search the graph? Will look at this when I'm back
        
        # def trace_route(self, origin, destination):  return route_trace i.e., route_trace.append((current_waypoint, road_option))
        """
        This method returns list of (carla.Waypoint, RoadOption)
        from origin to destination

            inside this is the following method that uses A* heuristic
            def _path_search(self, origin, destination):
                This function finds the shortest path connecting origin and destination
                using A* search with distance heuristic.

                    class RoadOption(Enum):
                    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
                        VOID = -1
                        LEFT = 1
                        RIGHT = 2
                        STRAIGHT = 3
                        LANEFOLLOW = 4
                        CHANGELANELEFT = 5
                        CHANGELANERIGHT = 6


        """

        #print(interpolated_trace)  # we have a fully populated routed with densly spaced waypoints along with roadoptions such as lane follow of change left or left etc
        #sys.exit("damn")  # printed was a lot of these just for the first two waypoints in our GPS trajectory
                          # [(<carla.libcarla.Waypoint object at 0x000001A52D8F13F0>, <RoadOption.LANEFOLLOW: 4>), 



        for wp_tuple in interpolated_trace:
            route.append((wp_tuple[0].transform, wp_tuple[1]))  # append waypoint transform (i.e., the locations and rotations) and the RoadOptions for all the newly densly populated waypoints and RoadOptions between the each GPS loc pair.

    # Increase the route position to avoid fails

    lat_ref, lon_ref = _get_latlon_ref(world)  # of the entire map

    #print(lat_ref)  # 0
    #print(len(lat_ref))
    #print(lon_ref)  # 0
    #sys.exit("it")



    return location_route_to_gps(route, lat_ref, lon_ref), route  # Locate each waypoint of the route into gps, (lat long ) representations. # return route
    # following are returned:
    # print(connection)  # RoadOption.LANEFOLLOW
    # print(gps_point)  # {'lat': -0.0016101365263807565, 'lon': -3.104682434048109e-05, 'z': 0.0} 
    # and the route with densly populated waypoints and RoadOptions