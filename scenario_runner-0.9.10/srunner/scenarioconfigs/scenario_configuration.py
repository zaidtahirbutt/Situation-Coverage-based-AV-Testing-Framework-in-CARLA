#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the key configuration parameters for an XML-based scenario
"""

import carla


class ActorConfigurationData(object):

    """
    This is a configuration base class to hold model and transform attributes
    """

    def __init__(self, model, transform, rolename='other', speed=0, autopilot=False,  # ROLENAME IS HERO FOR EGO VEHICLES ZZZZZZZZZZZZ
                 random=False, color=None, category="car", args=None):
        self.model = model
        self.rolename = rolename
        self.transform = transform
        self.speed = speed
        self.autopilot = autopilot
        self.random_location = random
        self.color = color
        self.category = category
        self.args = args

    @staticmethod
    def parse_from_node(node, rolename):  # ActorConfigurationData.parse_from_node(node, 'simulation') for antagonist actors
        """
        static method to initialize an ActorConfigurationData from a given ET tree
        """

        model = node.attrib.get('model', 'vehicle.*')  # <ego_vehicle x="-74.32" y="-50" z="0.5" yaw="270" model="vehicle.lincoln.mkz2017" />
        # I think the star after the vehicle means to add only model name after vehicle.. I might be wrong. I think 'vehicle.*' is default, if no named attribute is found
        pos_x = float(node.attrib.get('x', 0))  # same here for default values
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))

        transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z), carla.Rotation(yaw=yaw))

        rolename = node.attrib.get('rolename', rolename)  # same here for default value :3   # # ROLENAME IS HERO FOR EGO VEHICLES ZZZZZZZZZZZZ Hero, scenario for the other bullit actor 

        speed = node.attrib.get('speed', 0)  # I guess 0 is the default value, yes xD

        autopilot = False
        if 'autopilot' in node.keys():  # No autopilot here in nosigjunc file
            autopilot = True

        random_location = False
        if 'random_location' in node.keys():
            random_location = True

        color = node.attrib.get('color', None)

        return ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color)


class ScenarioConfiguration(object):

    """
    This class provides a basic scenario configuration incl.:
    - configurations for all actors
    - town, where the scenario should be executed
    - name of the scenario (e.g. ControlLoss_1)
    - type is the class of scenario (e.g. ControlLoss)
    """

    trigger_points = []
    ego_vehicles = [] # ActorConfigurationData(object) of ego veh inside this
    other_actors = []  # ActorConfigurationData(object) of other veh inside this
    town = None
    name = None
    type = None
    route = None
    agent = None
    weather = carla.WeatherParameters()
    friction = None
    subtype = None
    route_var_name = None
