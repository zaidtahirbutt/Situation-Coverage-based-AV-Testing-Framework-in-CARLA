#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the key configuration parameters for a route-based scenario
"""

import carla
from agents.navigation.local_planner import RoadOption

from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration


class RouteConfiguration(object):

    """
    This class provides the basic  configuration for a route
    """

    def __init__(self, route=None):
        self.data = route

    def parse_xml(self, node):
        """
        Parse route config XML
        #example of a route
                    #<routes>
                        #<route id="0" town="Town02"> 
                            #<waypoint x="-3.6973562240600586" y="179.0623321533203" z="0.0" pitch="0.0" roll="0.0" yaw="-90.06791687011719" />

        """
        self.data = []

        for waypoint in node.iter("waypoint"):  # So my guess is this iterates through each route id and goes through all the waypoints
            x = float(waypoint.attrib.get('x', 0))
            y = float(waypoint.attrib.get('y', 0))
            z = float(waypoint.attrib.get('z', 0))
            c = waypoint.attrib.get('connection', '')
            connection = RoadOption[c.split('.')[1]]

            self.data.append((carla.Location(x, y, z), connection))


class RouteScenarioConfiguration(ScenarioConfiguration):

    """
    Basic configuration of a RouteScenario
    """

    trajectory = None  # carla location waypoints here from xml!
    scenario_file = None  # the json file
