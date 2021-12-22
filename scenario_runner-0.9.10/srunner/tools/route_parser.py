#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Module used to parse all the route and scenario configuration parameters.
"""

import json
import math
import xml.etree.ElementTree as ET
import sys

import carla
from agents.navigation.local_planner import RoadOption
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration

# TODO  check this threshold, it could be a bit larger but not so large that we cluster scenarios.
TRIGGER_THRESHOLD = 2.0  # Threshold to say if a trigger position is new or repeated, works for matching positions
TRIGGER_ANGLE_THRESHOLD = 10  # Threshold to say if two angles can be considering matching when matching transforms.


class RouteParser(object):

    """
    Pure static class used to parse all the route and scenario configuration parameters.
    """

    @staticmethod
    def parse_annotations_file(annotation_filename):  # world_annotations = RouteParser.parse_annotations_file(config.scenario_file)  # scenario file is the JSON file # return final_dict  # the file has a current maps name that is an one element vec
        """
        Return the annotations of which positions where the scenarios are going to happen.
        :param annotation_filename: the filename for the anotations file
        :return:
        """

        with open(annotation_filename, 'r') as f:  # The JSON file -> scenario_file -> srunner/data/all_towns_traffic_scenarios1_3_4.json
            annotation_dict = json.loads(f.read())
            #print(annotation_filename)  # srunner/data/all_towns_traffic_scenarios1_3_4.json

        final_dict = {}

        for town_dict in annotation_dict['available_scenarios']:
            final_dict.update(town_dict)

        #print(final_dict)
        
        #print(next(iter(final_dict)))  # prints Town01

        #for keys in final_dict.keys():
            #print(keys)  
            '''
            prints
            Town01
            Town02
            Town03
            Town04
            Town05
            Town06
            '''
        '''
        for key, value in final_dict.items():
            print(key)  # All town numbers printed before all the array values of each town were printed.
            print(f"{key}-value are as {value}")  #Town01-value are as, then all the values of that object array, i.e., te available event configurations, transforms, other vehicle transforms, left right front transforms. 
        '''
    
        return final_dict  # the file has a current maps name that is an one element vec

    @staticmethod
    def parse_routes_file(route_filename, scenario_file, single_route=None):
        """
        Returns a list of route elements.
        :param route_filename: the path to a set of routes. 
        :param single_route: If set, only this route shall be returned
        :return: List of dicts containing the waypoints, id and town of the routes
        """

        list_route_descriptions = []
        tree = ET.parse(route_filename)  # srunner/data/routes_debug.xml 
        for route in tree.iter("route"):  # <route id="0" town="Town02"> 

            route_id = route.attrib['id']  # <route id="0" town="Town02"> 
            if single_route and route_id != single_route:  #  single_route = 0, route_id = 0.
                continue


            # For route id = 0 we doing this below: <route id="0" town="Town02"> 

            new_config = RouteScenarioConfiguration()  # RouteScenarioConfiguration/scenario config object
            new_config.town = route.attrib['town']  # s/o ScenarioConfiguration
            new_config.name = "RouteScenario_{}".format(route_id)  # RouteScenario_0
            new_config.weather = RouteParser.parse_weather(route)  # route -> <route id="0" town="Town02">  
            new_config.scenario_file = scenario_file  # The JSON file -> scenario_file -> srunner/data/all_towns_traffic_scenarios1_3_4.json

            waypoint_list = []  # the list of waypoints that can be found on this route  # NAAAAAAAAICCCCCCCCCCCCEEEEEEEEEE
             # For route id = 0 we doing this below: <route id="0" town="Town02"> 
            for waypoint in route.iter('waypoint'):   # For route id = 0 we doing this below: <route id="0" town="Town02"> ,
            # and now iterating through the route for waypoints!!! and adding them in the order they come, I can even draw them here :3 
            # go through all the points in the route!!!!!!!!!!    

                waypoint_list.append(carla.Location(x=float(waypoint.attrib['x']),  # converting the waypoints to a valid cala location here and appending that carla location to the waypoint list variable!
                                                    y=float(waypoint.attrib['y']),
                                                    z=float(waypoint.attrib['z'])))

            new_config.trajectory = waypoint_list  # added them (all the waypoint carla locations) to the trajectory variable of the scenario/route configuration object!

            list_route_descriptions.append(new_config)  # only one route cx of one route ID. This list has only one object hence.

        return list_route_descriptions  

    @staticmethod
    def parse_weather(route):  # set weather, might NEED it later!
        """
        Returns a carla.WeatherParameters with the corresponding weather for that route. If the route
        has no weather attribute, the default one is triggered.
        """

        route_weather = route.find("weather")
        if route_weather is None:

            weather = carla.WeatherParameters(sun_altitude_angle=70)

        else:
            weather = carla.WeatherParameters()
            for weather_attrib in route.iter("weather"):

                if 'cloudiness' in weather_attrib.attrib:  # child.tag, child.attrib
                    weather.cloudiness = float(weather_attrib.attrib['cloudiness'])  # pass those decimal values
                if 'precipitation' in weather_attrib.attrib:
                    weather.precipitation = float(weather_attrib.attrib['precipitation'])
                if 'precipitation_deposits' in weather_attrib.attrib:
                    weather.precipitation_deposits = float(weather_attrib.attrib['precipitation_deposits'])
                if 'wind_intensity' in weather_attrib.attrib:
                    weather.wind_intensity = float(weather_attrib.attrib['wind_intensity'])
                if 'sun_azimuth_angle' in weather_attrib.attrib:
                    weather.sun_azimuth_angle = float(weather_attrib.attrib['sun_azimuth_angle'])
                if 'sun_altitude_angle' in weather_attrib.attrib:
                    weather.sun_altitude_angle = float(weather_attrib.attrib['sun_altitude_angle'])
                if 'wetness' in weather_attrib.attrib:
                    weather.wetness = float(weather_attrib.attrib['wetness'])
                if 'fog_distance' in weather_attrib.attrib:
                    weather.fog_distance = float(weather_attrib.attrib['fog_distance'])
                if 'fog_density' in weather_attrib.attrib:
                    weather.fog_density = float(weather_attrib.attrib['fog_density'])

                    '''
                    <route id="0" town="Town01">
                       <weather 
                          cloudiness="0"
                          precipitation="0"
                          precipitation_deposits="0"
                          wind_intensity="0"
                          sun_azimuth_angle="0"
                          sun_altitude_angle="70"
                          fog_density="0"
                          fog_distance="0"
                          wetness="0"
                       />

                    '''

        return weather

    @staticmethod
    def check_trigger_position(new_trigger, existing_triggers):  # trigger_id = RouteParser.check_trigger_position(waypoint, existent_triggers)  # waypoint is the trigger transform  that has matched with a waypoint inside our route traj  # existent_triggers is empty for our first iteration
        """
        Check if this trigger position already exists or if it is a new one.
        :param new_trigger:
        :param existing_triggers:
        :return:
        """

        for trigger_id in existing_triggers.keys():
            trigger = existing_triggers[trigger_id]
            dx = trigger['x'] - new_trigger['x']
            dy = trigger['y'] - new_trigger['y']
            distance = math.sqrt(dx * dx + dy * dy)

            dyaw = (trigger['yaw'] - new_trigger['yaw']) % 360
            
            # TRIGGER_THRESHOLD = 2.0
            # TRIGGER_ANGLE_THRESHOLD = 10
            
            if distance < TRIGGER_THRESHOLD \
                    and (dyaw < TRIGGER_ANGLE_THRESHOLD or dyaw > (360 - TRIGGER_ANGLE_THRESHOLD)):
                return trigger_id  # if the triger is already in the existing_triggers, it will return the trigger_id else it will return none i.e., implying that there are none matches xP

        return None

    @staticmethod
    def convert_waypoint_float(waypoint):
        """
        Convert waypoint values to float
        """
        waypoint['x'] = float(waypoint['x'])
        waypoint['y'] = float(waypoint['y'])
        waypoint['z'] = float(waypoint['z'])
        waypoint['yaw'] = float(waypoint['yaw'])

    @staticmethod
    def match_world_location_to_route(world_location, route_description):  # (waypoint, trajectory)  # waypoint is the trigger transform and traj is our route
        """
        We match this location to a given route.
            world_location:
            route_description:
        """
        def match_waypoints(waypoint1, wtransform):
            """
            Check if waypoint1 and wtransform are similar
            """

            #print("shiz")  # shiz
            #sys.exit("off")

            dx = float(waypoint1['x']) - wtransform.location.x  # transform x - route x
            dy = float(waypoint1['y']) - wtransform.location.y
            dz = float(waypoint1['z']) - wtransform.location.z
            dpos = math.sqrt(dx * dx + dy * dy + dz * dz)

            dyaw = (float(waypoint1['yaw']) - wtransform.rotation.yaw) % 360

            # TRIGGER_THRESHOLD = 2.0
            # TRIGGER_ANGLE_THRESHOLD = 10
            
            return dpos < TRIGGER_THRESHOLD \
                and (dyaw < TRIGGER_ANGLE_THRESHOLD or dyaw > (360 - TRIGGER_ANGLE_THRESHOLD))

        # *********** outer function starts here merroooo ***************

        match_position = 0
        # TODO this function can be optimized to run on Log(N) time
        for route_waypoint in route_description:  # in route
            if match_waypoints(world_location, route_waypoint[0]):  # trigger point and route waypoint
                
                # If above statement is true, it means our trigger point is on a certain waypoint inside our defined densly populated route
                # and the trigger yaw is the same as that particular waypoint's yaw within certain thresholds

                return match_position  # this match position is the index of the waypoint inside our route array for which our particular transform has matched with.
            match_position += 1

        return None

    @staticmethod
    def get_scenario_type(scenario, match_position, trajectory):  #scenario_subtype = RouteParser.get_scenario_type(scenario_name, match_position, trajectory)  # # "scenario_type": "Scenario1", i.e., all scenarios  # this match position is the index of the waypoint inside our route array  # traj is our densly populated route
                                                                         
        """
        Some scenarios have different types depending on the route.
        :param scenario: the scenario name
        :param match_position: the matching position for the scenarion
        :param trajectory: the route trajectory the ego is following
        :return: tag representing this subtype

        Also used to check which are not viable (Such as an scenario
        that triggers when turning but the route doesnt')
        WARNING: These tags are used at:
            - VehicleTurningRoute
            - SignalJunctionCrossingRoute
        and changes to these tags will affect them
        """

        def check_this_waypoint(tuple_wp_turn):
            """
            Decides whether or not the waypoint will define the scenario behavior
            """
            if RoadOption.LANEFOLLOW == tuple_wp_turn[1]:
                return False
            elif RoadOption.CHANGELANELEFT == tuple_wp_turn[1]:
                return False
            elif RoadOption.CHANGELANERIGHT == tuple_wp_turn[1]:
                return False
            return True

        '''


        VOID = -1
        LEFT = 1
        RIGHT = 2
        STRAIGHT = 3
        LANEFOLLOW = 4
        CHANGELANELEFT = 5
        CHANGELANERIGHT = 6


        '''

        # Unused tag for the rest of scenarios,
        # can't be None as they are still valid scenarios
        subtype = 'valid'

        # We have till "scenario_type": "Scenario4" in Town02 in the all towns 1 3 4 JSON file and other actors only in scenario 4 and I guess after scenario 4
        # and I noticed we have scenario 1, 3, 4 in all towns 1 3 4 JSON file, go figure.


        if scenario == 'Scenario4':
            for tuple_wp_turn in trajectory[match_position:]:  # the trigger point that matched with our route at the match position
                if check_this_waypoint(tuple_wp_turn):  # check if the RoadOption of that matched waypoint is a left or right, tag them as S4left, S4right, if neither then tag them as none.
                    if RoadOption.LEFT == tuple_wp_turn[1]:
                        subtype = 'S4left'
                    elif RoadOption.RIGHT == tuple_wp_turn[1]:
                        subtype = 'S4right'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        if scenario == 'Scenario7':  # for scenario 7 we have the RoadOption STRAIGHT also available. Tag it as S7opposite
            for tuple_wp_turn in trajectory[match_position:]:
                if check_this_waypoint(tuple_wp_turn):
                    if RoadOption.LEFT == tuple_wp_turn[1]:
                        subtype = 'S7left'
                    elif RoadOption.RIGHT == tuple_wp_turn[1]:
                        subtype = 'S7right'
                    elif RoadOption.STRAIGHT == tuple_wp_turn[1]:
                        subtype = 'S7opposite'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        if scenario == 'Scenario8':  # for S8 we only have left
            for tuple_wp_turn in trajectory[match_position:]:
                if check_this_waypoint(tuple_wp_turn):
                    if RoadOption.LEFT == tuple_wp_turn[1]:
                        subtype = 'S8left'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        if scenario == 'Scenario9':  # for S9 we only have right
            for tuple_wp_turn in trajectory[match_position:]:
                if check_this_waypoint(tuple_wp_turn):
                    if RoadOption.RIGHT == tuple_wp_turn[1]:
                        subtype = 'S9right'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        return subtype

    @staticmethod
    def scan_route_for_scenarios(route_name, trajectory, world_annotations):  # potential_scenarios_definitions, _ = RouteParser.scan_route_for_scenarios(config.town, route, world_annotations)  # Town02  # route as defined in the method above  # world_annotations is the dictionary of the JSON file with town names as keys for each scenario and each key returns the annotations of which positions where the scenarios are going to happen.
        """
        Just returns a plain list of possible scenarios that can happen in this route by matching
        the locations from the scenario into the route description

        :return:  A list of scenario definitions with their correspondent parameters
        """

        # the triggers dictionaries:
        existent_triggers = {}
        # We have a table of IDs and trigger positions associated  # ID meaning -> "scenario_type": "Scenario1"??
        possible_scenarios = {}

        # Keep track of the trigger ids being added
        latest_trigger_id = 0

        for town_name in world_annotations.keys():
            if town_name != route_name:
                continue
                # for the selected town

            scenarios = world_annotations[town_name]  # many possible scenarios mentioned like "scenario_type": "Scenario1"??
            
            #print(scenarios)  # all scenarios with all their transforms and everything was printed. The Dynamic Language. 
            #sys.exit("heff")



            for scenario in scenarios:  # For each existent scenario
                if "scenario_type" not in scenario:  # if a block of code does not have "scenario_type": "Scenario1 or X"  
                # if the JSON file does not have a scenario_type object key I guess
                    break  # heff this loop then
                scenario_name = scenario["scenario_type"]  # "scenario_type": "Scenario1"
                
                #print(scenario_name)  # Scenario1
                #sys.exit("heff")


                for event in scenario["available_event_configurations"]:  # for each scenario there is "available_event_configurations": [ in which there are numerous transforms and other vehicles and front and side and back transforms etc
                    waypoint = event['transform']  # trigger point of this scenario  # just noticed in some JSON available_event_configurations we have other vehicles with left or right sub fields which have front left &/or right locs and a transform underneath them as a trigger point I presume 
                    RouteParser.convert_waypoint_float(waypoint)
                    # We match trigger point to the  route, now we need to check if the route affects
                    match_position = RouteParser.match_world_location_to_route(
                        waypoint, trajectory)  # waypoint is the trigger transform and traj is our route
                    
                    # return match_position  # this match position is the index of the waypoint inside our route array for which our particular transform has matched with.  




                    if match_position is not None:
                        # We match a location for this scenario, create a scenario object so this scenario
                        # can be instantiated later

                        if 'other_actors' in event:
                            other_vehicles = event['other_actors']  # all the other_actors with front, left, right locations in this
                        else:
                            other_vehicles = None
                        scenario_subtype = RouteParser.get_scenario_type(scenario_name, match_position,  # # "scenario_type": "Scenario1"  # this match position is the index of the waypoint inside our route array  # traj is our densly populated route
                                                                         trajectory)
                        
                        # return subtype i.e., S4left, S4right, S8left, S9right, S7opposite (RoadOption = straight) etc

                        if scenario_subtype is None:
                            continue  # heff it
                        scenario_description = {
                            'name': scenario_name,  # all the scenarios we are parsing
                            'other_actors': other_vehicles,  # all the transforms inside the other vehicles i.e., left right front
                            'trigger_position': waypoint,  # waypoint is the trigger transform  that has matched with a waypoint inside our route traj
                            'scenario_type': scenario_subtype,  # scenario_name = scenario["scenario_type"]  # "scenario_type": "Scenario1"  # some scenarios have route dependent configs, S4left, etc
                        }

                        trigger_id = RouteParser.check_trigger_position(waypoint, existent_triggers)  # waypoint is the trigger transform  that has matched with a waypoint inside our route traj  # existent_triggers is empty for our first iteration
                        
                        #return trigger_id  # if the triger is already in the existing_triggers, it will return the trigger_id else it will return none i.e., implying that there are none matches xP


                        if trigger_id is None:
                            # This trigger does not exist create a new reference on existent triggers
                            existent_triggers.update({latest_trigger_id: waypoint})  # latest_trigger_id = 0 initially
                            # Update a reference for this trigger on the possible scenarios

                            possible_scenarios.update({latest_trigger_id: []})  # latest_trigger_id = 0 initially
                            
                            trigger_id = latest_trigger_id  # so for first trigger that was not already in the dictionary existent_triggers, is initially give an id of 0 for the initial very first one
                            
                            # Increment the latest trigger
                            latest_trigger_id += 1  # until we catch the next trigger id cx it is not included in our existent_triggers dictionary yet

                        possible_scenarios[trigger_id].append(scenario_description)  # so we have a whole scenario for that particular trigger location as defined above and we do these for all the trigger points defined to create all these scenarios

                        # so now for the trigger id on which we caught the trigger not being matched, we append the possible scenarios dictionary on that particular id wiht the whole scenario description dictionary

                        '''
                        scenario_description = {
                            'name': scenario_name,  # all the scenarios we are parsing
                            'other_actors': other_vehicles,  # all the transforms inside the other vehicles i.e., left right front
                            'trigger_position': waypoint,  # waypoint is the trigger transform  that has matched with a waypoint inside our route traj
                            'scenario_type': scenario_subtype,  # scenario_name = scenario["scenario_type"]  # "scenario_type": "Scenario1"  # some scenarios have route dependent configs, S4left, etc
                        }
                        '''

        return possible_scenarios, existent_triggers  # existent_triggers have all the trigger points inside the, and possible scenarios have the complete description of those scenarios from all towns 1 3 4 JSOn file compared to our route
