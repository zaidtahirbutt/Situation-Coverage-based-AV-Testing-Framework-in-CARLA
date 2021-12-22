#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides access to a scenario configuration parser
"""

import glob
import os
import xml.etree.ElementTree as ET

from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData
from srunner.scenarioconfigs.route_scenario_configuration import RouteConfiguration


class ScenarioConfigurationParser(object):

    """
    Pure static class providing access to parser methods for scenario configuration files (*.xml)
    """

    @staticmethod
    def parse_scenario_configuration(scenario_name, config_file_name):
        """
        Parse all scenario configuration files at srunner/examples and the additional
        config files, providing a list of ScenarioConfigurations @return

        If scenario_name starts with "group:" all scenarios that
        have that type are parsed and returned. Otherwise only the
        scenario that matches the scenario_name is parsed and returned.
        """

        list_of_config_files = glob.glob("{}/srunner/examples/*.xml".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))

        if config_file_name != '':
            list_of_config_files.append(config_file_name)
            #print("yo")    # Nothing happend
            #print(config_file_name)
            #return 0

        #for x in list_of_config_files:
            #print(x)  # D:\scenario_runner-0.9.10/srunner/examples\ChangeLane.xml
            #return 0

        single_scenario_only = True
        if scenario_name.startswith("group:"):
            single_scenario_only = False
            scenario_name = scenario_name[6:]

        scenario_configurations = []

        for file_name in list_of_config_files:
            tree = ET.parse(file_name)  # ET is from XMLELementTree, list_of_config_files

            #print("yoo")   # Object returned
            #print(tree)
            #return 0

            for scenario in tree.iter("scenario"):  # NoSignalJunctionCrossing is the scenario here

                scenario_config_name = scenario.attrib.get('name', None)    # <scenario name="NoSignalJunctionCrossing" type="NoSignalJunctionCrossing" town="Town03">
                scenario_config_type = scenario.attrib.get('type', None)

                #print("yoo")
                #print(scenario_config_name)  # ChangeLane_1
                #return 0

                if single_scenario_only:
                    # Check the scenario is the correct one
                    if scenario_config_name != scenario_name:
                        continue
                else:
                    # Check the scenario is of the correct type
                    if scenario_config_type != scenario_name:
                        continue

                #print("yoo")
                #print(scenario_config_name)  # NoSignalJunctionCrossing 
                # so this command  scenario.attrib.get('name', None) is making the names equal to scenario_config_name, one by one and
                #we keep going back to the start of the for loop till it is equal to our scenario_name and the the for loop 
                #further proceeds
                #return 0

                new_config = ScenarioConfiguration()
                new_config.town = scenario.attrib.get('town', None)
                new_config.name = scenario_config_name  # NoSignalJunctionCrossing
                new_config.type = scenario_config_type  # NoSignalJunctionCrossing
                new_config.other_actors = []
                new_config.ego_vehicles = []
                new_config.trigger_points = []  # trigger points only for spawn loc of ego veh

                for weather in scenario.iter("weather"):    # inside the particular XML file
                    new_config.weather.cloudiness = float(weather.attrib.get("cloudiness", 0))  # get these values from scenario xml
                    new_config.weather.precipitation = float(weather.attrib.get("precipitation", 0))
                    new_config.weather.precipitation_deposits = float(weather.attrib.get("precipitation_deposits", 0))
                    new_config.weather.wind_intensity = float(weather.attrib.get("wind_intensity", 0.35))
                    new_config.weather.sun_azimuth_angle = float(weather.attrib.get("sun_azimuth_angle", 0.0))
                    new_config.weather.sun_altitude_angle = float(weather.attrib.get("sun_altitude_angle", 15.0))
                    new_config.weather.fog_density = float(weather.attrib.get("fog_density", 0.0))
                    new_config.weather.fog_distance = float(weather.attrib.get("fog_distance", 0.0))
                    new_config.weather.wetness = float(weather.attrib.get("wetness", 0.0))

                for ego_vehicle in scenario.iter("ego_vehicle"):  # Iterating through the xml file I believe yes Alhamdulillah 
                    # inside the particular XML file
                    #<ego_vehicle x="-74.32" y="-50" z="0.5" yaw="270" model="vehicle.lincoln.mkz2017" /> ego veh on the left most side :P 
                    new_config.ego_vehicles.append(ActorConfigurationData.parse_from_node(ego_vehicle, 'hero')) # object is returned # here we give the name hero to ego veh # Object from class ActorConfigurationData(object): appeneded here
                    #parse_from_node is a static method to initialize an ActorConfigurationData from a given ET tree
                    #^
                    #|    
                    # ROLENAME IS hero FOR EGO VEHICLES ZZZZZZZZZZZZ# ROLENAME IS HERO FOR EGO VEHICLES ZZZZZZZZZZZZ# ROLENAME IS HERO FOR EGO VEHICLES ZZZZZZZZZZZZ# ROLENAME IS HERO FOR EGO VEHICLES ZZZZZZZZZZZZ    

                    # print("yoo")
                    # print(new_config.ego_vehicles[-1].transform.location.x) # -74.31999969482422 was the output meaning spawn loc of ego vehicle which in this case is only one
                    # return 0

                    new_config.trigger_points.append(new_config.ego_vehicles[-1].transform) #spawn loc from last obj included in list, added to the config class object

                for route in scenario.iter("route"):  # so the main header names is routes while each route id is route so this iterator is going through the route i.e., each route id
                    #example of a route
                    #<routes>
                        #<route id="0" town="Town02"> 
                            #<waypoint x="-3.6973562240600586" y="179.0623321533203" z="0.0" pitch="0.0" roll="0.0" yaw="-90.06791687011719" />                  
                    route_conf = RouteConfiguration()
                    route_conf.parse_xml(route)  # func of RouteConfiguration() class!... self.data.append((carla.Location(x, y, z), connection)) is happening, data base of the rout points in the order in which they come is being made 
                    new_config.route = route_conf  # each route (not each as new_config.route is not being appened here, it's just being equated to here, maybe it only takes one route?) id is stored her one after the other using tree structure of the ET xml tree guy 
                    #no routes in the examples folder of the xmls either so I don't think this not appending route here is a thing to worry about
                    # so this variable new_config.route isn't even created when there aren't any routes mentioned!!
                    #print("yoo")
                    #return 0


                for other_actor in scenario.iter("other_actor"):
                    new_config.other_actors.append(ActorConfigurationData.parse_from_node(other_actor, 'scenario'))

                    # print("yoo")
                    # print(new_config.other_actors[-1].transform.location.x) # -105.0 was the output meaning spawn loc of other_actors which in this case is only one
                    # print(new_config.other_actors[-1].rolename) # scenario was the output 
                    # return 0

                #print("yoo2")  
                #return 0

                scenario_configurations.append(new_config)

        return scenario_configurations

    @staticmethod
    def get_list_of_scenarios(config_file_name):
        """
        Parse *all* config files and provide a list with all scenarios @return
        """

        list_of_config_files = glob.glob("{}/srunner/examples/*.xml".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))
        list_of_config_files += glob.glob("{}/srunner/examples/*.xosc".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))

        if config_file_name != '':
            list_of_config_files.append(config_file_name)

        scenarios = []
        for file_name in list_of_config_files:
            if ".xosc" in file_name:
                tree = ET.parse(file_name)
                scenarios.append("{} (OpenSCENARIO)".format(tree.find("FileHeader").attrib.get('description', None)))
            else:
                tree = ET.parse(file_name)
                for scenario in tree.iter("scenario"):
                    scenarios.append(scenario.attrib.get('name', None))

        return scenarios
