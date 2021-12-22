#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides Challenge routes as standalone scenarios
"""

from __future__ import print_function

import math
import traceback
import xml.etree.ElementTree as ET
import numpy.random as random
import sys

import py_trees

import carla

from agents.navigation.local_planner import RoadOption

# pylint: disable=line-too-long
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData
# pylint: enable=line-too-long
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, ScenarioTriggerer
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.route_parser import RouteParser, TRIGGER_THRESHOLD, TRIGGER_ANGLE_THRESHOLD
from srunner.tools.route_manipulation import interpolate_trajectory
from srunner.tools.py_trees_port import oneshot_behavior

from srunner.scenarios.control_loss import ControlLoss
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
from srunner.scenarios.object_crash_vehicle import DynamicObjectCrossing
from srunner.scenarios.object_crash_intersection import VehicleTurningRoute
from srunner.scenarios.other_leading_vehicle import OtherLeadingVehicle
from srunner.scenarios.maneuver_opposite_direction import ManeuverOppositeDirection
from srunner.scenarios.junction_crossing_route import SignalJunctionCrossingRoute, NoSignalJunctionCrossingRoute

from srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
                                                                     InRouteTest,
                                                                     RouteCompletionTest,
                                                                     OutsideRouteLanesTest,
                                                                     RunningRedLightTest,
                                                                     RunningStopTest,
                                                                     ActorSpeedAboveThresholdTest)

SECONDS_GIVEN_PER_METERS = 0.4

NUMBER_CLASS_TRANSLATION = {
    "Scenario1": ControlLoss,
    "Scenario2": FollowLeadingVehicle,
    "Scenario3": DynamicObjectCrossing,
    "Scenario4": VehicleTurningRoute,
    "Scenario5": OtherLeadingVehicle,
    "Scenario6": ManeuverOppositeDirection,
    "Scenario7": SignalJunctionCrossingRoute,
    "Scenario8": SignalJunctionCrossingRoute,
    "Scenario9": SignalJunctionCrossingRoute,
    "Scenario10": NoSignalJunctionCrossingRoute  # wut :3
}


def convert_json_to_transform(actor_dict):
    """
    Convert a JSON string to a CARLA transform
    """
    return carla.Transform(location=carla.Location(x=float(actor_dict['x']), y=float(actor_dict['y']),
                                                   z=float(actor_dict['z'])),
                           rotation=carla.Rotation(roll=0.0, pitch=0.0, yaw=float(actor_dict['yaw'])))


def convert_json_to_actor(actor_dict):
    """
    Convert a JSON string to an ActorConfigurationData dictionary
    """
    node = ET.Element('waypoint')
    node.set('x', actor_dict['x'])
    node.set('y', actor_dict['y'])
    node.set('z', actor_dict['z'])
    node.set('yaw', actor_dict['yaw'])

    return ActorConfigurationData.parse_from_node(node, 'simulation')  # return object of ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color) ActorConfigurationData


def convert_transform_to_location(transform_vec):
    """
    Convert a vector of transforms to a vector of locations
    """
    location_vec = []
    for transform_tuple in transform_vec:
        location_vec.append((transform_tuple[0].location, transform_tuple[1]))

    return location_vec


def compare_scenarios(scenario_choice, existent_scenario):  # if compare_scenarios(scenario_choice, existent_scenario)
    """
    Compare function for scenarios based on distance of the scenario start position
    """
    def transform_to_pos_vec(scenario):
        """
        Convert left/right/front to a meaningful CARLA position
        """
        position_vec = [scenario['trigger_position']]
        if scenario['other_actors'] is not None:
            if 'left' in scenario['other_actors']:
                position_vec += scenario['other_actors']['left']
            if 'front' in scenario['other_actors']:
                position_vec += scenario['other_actors']['front']
            if 'right' in scenario['other_actors']:
                position_vec += scenario['other_actors']['right']

            #print(position_vec)
            #sys.exit("offfff")
            '''
            Preparing scenario: RouteScenario_0
            [{'pitch': '0', 'x': 41.37, 'y': 207.4, 'yaw': 90.0, 'z': 1.21}, {'pitch': '0.0',
             'x': '75.58', 'y': '236.78', 'yaw': '180.000015', 'z': '1.21'}, {'pitch': '0.0', 
             'x': '45.82', 'y': '270.45', 'yaw': '270.000031', 'z': '1.21'}]
            '''

        return position_vec

    # put the positions of the scenario choice into a vec of positions to be able to compare

    choice_vec = transform_to_pos_vec(scenario_choice)
    existent_vec = transform_to_pos_vec(existent_scenario)
    for pos_choice in choice_vec:
        for pos_existent in existent_vec:

            dx = float(pos_choice['x']) - float(pos_existent['x'])
            dy = float(pos_choice['y']) - float(pos_existent['y'])
            dz = float(pos_choice['z']) - float(pos_existent['z'])
            dist_position = math.sqrt(dx * dx + dy * dy + dz * dz)
            dyaw = float(pos_choice['yaw']) - float(pos_choice['yaw'])
            dist_angle = math.sqrt(dyaw * dyaw)
            if dist_position < TRIGGER_THRESHOLD and dist_angle < TRIGGER_ANGLE_THRESHOLD:
                return True

    return False


class RouteScenario(BasicScenario):

    """
    Implementation of a RouteScenario, i.e. a scenario that consists of driving along a pre-defined route,
    along which several smaller scenarios are triggered
    """

    def __init__(self, world, config, debug_mode=False, criteria_enable=True, timeout=300):
        """
        Setup all relevant parameters and create scenarios along route
        """

        self.config = config  # weather, json file and waypoints in the route. No ego, no other vehicle. config.agent = NpcAgent though.
        self.route = None
        self.sampled_scenarios_definitions = None

        self._update_route(world, config, debug_mode)
        # we got densly sampled route from this and the sampled scenarios as 
        # self.sampled_scenarios_definitions from all towns 1 3 4 JSON file, 
        # i.e., the trigger points and othervehicles location transforms and 
        # locs directions as left right etc and turning info S4left etc
        # and set_ego_vehicle_route in CarlaDataProvider._ego_vehicle_route = route
        # and self.time_out as time roughly needed to complete scenario as 0.4 sec per meter
        

        ego_vehicle = self._update_ego_vehicle()
        #  return ego_vehicle  #  vehicle type mentioned here and role Hero also given and spawned at route[0][0]

        self.list_scenarios = self._build_scenario_instances(world,
                                                             ego_vehicle,
                                                             self.sampled_scenarios_definitions,
                                                             scenarios_per_tick=5,
                                                             timeout=self.timeout,
                                                             debug_mode=debug_mode)

        # return scenario_instance_vec

        super(RouteScenario, self).__init__(name=config.name,
                                            ego_vehicles=[ego_vehicle],
                                            config=config,
                                            world=world,
                                            debug_mode=False,
                                            terminate_on_failure=False,
                                            criteria_enable=criteria_enable)

    def _update_route(self, world, config, debug_mode):
        """
        Update the input route, i.e. refine waypoint list, and extract possible scenario locations

        Parameters:
        - world: CARLA world
        - config: Scenario configuration (RouteConfiguration)
        """

        # Transform the scenario file into a dictionary
        world_annotations = RouteParser.parse_annotations_file(config.scenario_file)  # scenario file is the JSON file # return final_dict  # the file has a current maps name that is an one element vec
        """
        Return the annotations of which positions where the scenarios are going to happen.
        :param annotation_filename: the filename for the anotations file
        :return:
        """

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
        
        #return final_dict  # the file has a current maps name that is an one element vec







        # prepare route's trajectory (interpolate and add the GPS route) is the route in the routes_debug.xml an example of a GPS route? Maybe. 
        # Given some raw keypoints interpolate a full dense trajectory to be used by the user.  # meaning the route we have in the routes_drbug.xml file has many sparsly populated waypoints throughout the map and we will densly populate them for our AV to drive on with hop_res of 1. :3 
        gps_route, route = interpolate_trajectory(world, config.trajectory)  # in config.trajectory we have  # carla location waypoints here from the routes_debug.xml routes> id = 0, town= Town02 xml!

        # following are returned from interpolate_trajectory:
        # print(connection)  # RoadOption.LANEFOLLOW
        # print(gps_point)  # {'lat': -0.0016101365263807565, 'lon': -3.104682434048109e-05, 'z': 0.0} 
        # and the route with densly populated waypoints and RoadOptions

        # For this Route scenario, I see we get the densly populated route plus the RoadOptions with each densly populated waypoint. Don't know if we get that in other scenarios. Don't think so. Rather we depend on carla server
        # information so that our ego vehicle controllers can drive the vehicle to target location using saparsely populated waypoints and no RoadOptions given with those waypoints, rather infro from carla server is used by the ego veh controller.


        potential_scenarios_definitions, _ = RouteParser.scan_route_for_scenarios(config.town, route, world_annotations)  # Town02  # route as defined in the method above  # world_annotations is the dictionary of the JSON file with town names as keys for each scenario and each key returns the annotations of which positions where the scenarios are going to happen.

        # We have till "scenario_type": "Scenario4" in Town02 in the all towns 1 3 4 JSON file and other actors only in scenario 4 and I guess after scenario 4
        # and I noticed we have scenario 1, 3, 4 in all towns 1 3 4 JSON file, go figure.


        '''
                        scenario_description = {
                            'name': scenario_name,  # all the scenarios we are parsing
                            'other_actors': other_vehicles,  # all the transforms inside the other vehicles i.e., left right front
                            'trigger_position': waypoint,  # waypoint is the trigger transform  that has matched with a waypoint inside our route traj
                            'scenario_type': scenario_subtype, #  it actually is S4left etc I guess # scenario_name = scenario["scenario_type"]  # "scenario_type": "Scenario1"  # some scenarios have route dependent configs, S4left, etc
                        }
        '''

        #return possible_scenarios, existent_triggers  # existent_triggers have all the trigger points inside the, and possible scenarios have the complete description of those scenarios from all towns 1 3 4 JSOn file compared to our route

        # we dont even want the existent_triggers as we are equating them to _ as that info is also in the possible_scenarios


        """
        Just returns a plain list of possible scenarios that can happen in this route by matching
        the locations from the scenario into the route description

        :return:  A list of scenario definitions with their correspondent parameters
        """




        self.route = route  # interpolated dense traj/route with RoadOptions 

        CarlaDataProvider.set_ego_vehicle_route(convert_transform_to_location(self.route))  # convert_transform_to_location(self.route) returns carla location + the RoadOption in an array of the route, instead of the waypoint object and RoadOption
        # def of this function is:
        #   CarlaDataProvider._ego_vehicle_route = route


        config.agent.set_global_plan(gps_route, self.route)  # just resampling them (by some factor) to the min distance we already wanted (of 1)

        # Sample the scenarios to be used for this route instance.
        self.sampled_scenarios_definitions = self._scenario_sampling(potential_scenarios_definitions)  # potential_scenarios_definitions is scenario_description with scenario name scenario type other vehicles trigger locations defined in the All Towns 1 3 4 JSON file 
        # return sampled_scenarios



        # Timeout of scenario in seconds
        self.timeout = self._estimate_route_timeout()
        # return int(SECONDS_GIVEN_PER_METERS * route_length)

        # Print route in debug mode  # gotta try this!!!
        if debug_mode:  # drawing waypoints on the new DENSELY populated route  # wow that was fun :3
            self._draw_waypoints(world, self.route, vertical_shift=1.0, persistency=50000.0)

    def _update_ego_vehicle(self):
        """
        Set/Update the start position of the ego_vehicle
        """
        # move ego to correct position
        elevate_transform = self.route[0][0]


        #print(elevate_transform)  # Transform(Location(x=-3.456117, y=179.239578, z=0.000000), Rotation(pitch=0.000000, yaw=-90.012932, roll=0.000000))
        #sys.exit("yeahh")


        elevate_transform.location.z += 0.5

        ego_vehicle = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz2017',  #  vehicle type mentioned here and role Hero also given
                                                          elevate_transform,
                                                          rolename='hero')

        return ego_vehicle  #  vehicle type mentioned here and role Hero also given

    def _estimate_route_timeout(self):
        """
        Estimate the duration of the route
        """
        route_length = 0.0  # in meters

        prev_point = self.route[0][0]
        for current_point, _ in self.route[1:]:
            dist = current_point.location.distance(prev_point.location)
            route_length += dist
            prev_point = current_point

        return int(SECONDS_GIVEN_PER_METERS * route_length)

    # pylint: disable=no-self-use
    def _draw_waypoints(self, world, waypoints, vertical_shift, persistency=-1):  # self._draw_waypoints(world, self.route, vertical_shift=1.0, persistency=50000.0)
        """
        Draw a list of waypoints at a certain height given in vertical_shift.
        """
        for w in waypoints:
            wp = w[0].location + carla.Location(z=vertical_shift)

            size = 0.2
            if w[1] == RoadOption.LEFT:  # Yellow
                color = carla.Color(255, 255, 0)
            elif w[1] == RoadOption.RIGHT:  # Cyan
                color = carla.Color(0, 255, 255)
            elif w[1] == RoadOption.CHANGELANELEFT:  # Orange
                color = carla.Color(255, 64, 0)
            elif w[1] == RoadOption.CHANGELANERIGHT:  # Dark Cyan
                color = carla.Color(0, 64, 255)
            elif w[1] == RoadOption.STRAIGHT:  # Gray
                color = carla.Color(128, 128, 128)
            else:  # LANEFOLLOW
                color = carla.Color(0, 255, 0)  # Green
                size = 0.1

            world.debug.draw_point(wp, size=size, color=color, life_time=persistency)

        world.debug.draw_point(waypoints[0][0].location + carla.Location(z=vertical_shift), size=0.2,
                               color=carla.Color(0, 0, 255), life_time=persistency)
        world.debug.draw_point(waypoints[-1][0].location + carla.Location(z=vertical_shift), size=0.2,
                               color=carla.Color(255, 0, 0), life_time=persistency)

    def _scenario_sampling(self, potential_scenarios_definitions, random_seed=0):  # self._scenario_sampling(potential_scenarios_definitions)  # potential_scenarios_definitions is scenario_description with scenario name scenario type other vehicles trigger locations defined in the All Towns 1 3 4 JSON file
        """
        The function used to sample the scenarios that are going to happen for this route.
        """

        # fix the random seed for reproducibility
        rng = random.RandomState(random_seed)  #random Class object initialized by the RandomState class using random seed

        def position_sampled(scenario_choice, sampled_scenarios):  # position_sampled(scenario_choice, sampled_scenarios):
            """
            Check if a position was already sampled, i.e. used for another scenario
            """
            for existent_scenario in sampled_scenarios:
                # If the scenarios have equal positions then it is true.
                if compare_scenarios(scenario_choice, existent_scenario):
                    return True

            return False

        #zman_count = 0

        # The idea is to randomly sample a scenario per trigger position.
        sampled_scenarios = []
        for trigger in potential_scenarios_definitions.keys():
            possible_scenarios = potential_scenarios_definitions[trigger]

            #print(f"Possible scenario number {zman_count}")
            #print(possible_scenarios)
            #zman_count += 1
            #if zman_count == 5:
            #    sys.exit("off")

            '''
            Possible scenario number 0
            [{'name': 'Scenario1', 'other_actors': None, 'trigger_position': {'pitch': '0', 'x': 7.63, 'y': 109.91, 'yaw': 359.0, 'z': 1.22}, 'scenario_type': 'valid'}, 
            {'name': 'Scenario3', 'other_actors': None, 'trigger_position': {'pitch': '0', 'x': 7.63, 'y': 109.91, 
            'yaw': 359.0, 'z': 1.22}, 'scenario_type': 'valid'}]
            Possible scenario number 1
            [{'name': 'Scenario1', 'other_actors': None, 'trigger_position': {'pitch': '0', 'x': 41.37, 'y': 
            207.4, 'yaw': 90.0, 'z': 1.21}, 'scenario_type': 'valid'}, {'name': 'Scenario4', 'other_actors': {
            'front': [{'pitch': '0.0', 'x': '45.82', 'y': '270.45', 'yaw': '270.000031', 'z': '1.21'}], 'left'
            : [{'pitch': '0.0', 'x': '75.58', 'y': '236.78', 'yaw': '180.000015', 'z': '1.21'}]}, 'trigger_position
            ': {'pitch': '0', 'x': 41.37, 'y': 207.4, 'yaw': 90.0, 'z': 1.21}, 'scenario_type': 'S4left'}]
            '''

            #print(possible_scenarios)
            #sys.exit("off")    

            '''
            [{'name': 'Scenario1', 'other_actors': None, 'trigger_position': {'pitch': '0', 'x': 7.63,
             'y': 109.91, 'yaw': 359.0, 'z': 1.22}, 'scenario_type': 'valid'}, {'name': 'Scenario3',
              'other_actors': None, 'trigger_position': {'pitch': '0', 'x': 7.63, 'y': 109.91, 'yaw':
               359.0, 'z': 1.22}, 'scenario_type': 'valid'}]
            '''

            # so a single possible_scenarios has the same trigger point for the multiple scenarios inside it

            scenario_choice = rng.choice(possible_scenarios)

            #print(scenario_choice)
            #sys.exit("off")  

            '''
            Preparing scenario: RouteScenario_0
            {'name': 'Scenario1', 'other_actors': None, 'trigger_position': {'pitch': '0',
             'x': 7.63, 'y': 109.91, 'yaw': 359.0, 'z': 1.22}, 'scenario_type': 'valid'}
            '''

            del possible_scenarios[possible_scenarios.index(scenario_choice)]
            # We keep sampling and testing if this position is present on any of the scenarios.
            while position_sampled(scenario_choice, sampled_scenarios):  # returns true if scenario already sampled if I am right
                if possible_scenarios is None or not possible_scenarios:
                    scenario_choice = None
                    break
                scenario_choice = rng.choice(possible_scenarios)
                del possible_scenarios[possible_scenarios.index(scenario_choice)]
                #while loop keeps on going till we find a scenario that hasn't been sampled already

            if scenario_choice is not None:
                sampled_scenarios.append(scenario_choice)  # add to sampled scenarios! 

            '''
            Example of one scenario is this
            Preparing scenario: RouteScenario_0
            {'name': 'Scenario1', 'other_actors': None, 'trigger_position': {'pitch': '0',
             'x': 7.63, 'y': 109.91, 'yaw': 359.0, 'z': 1.22}, 'scenario_type': 'valid'} 
             scenario_type can be S4left S4right S4opposite, S5, S9, etc.
            '''

        return sampled_scenarios

    def _build_scenario_instances(self, world, ego_vehicle, scenario_definitions,
                                  scenarios_per_tick=5, timeout=300, debug_mode=False):
        
        '''
        self.list_scenarios = self._build_scenario_instances(world,
                                                             ego_vehicle,
                                                             self.sampled_scenarios_definitions,
                                                             scenarios_per_tick=5,
                                                             timeout=self.timeout,
                                                             debug_mode=debug_mode)

        '''


        """
        Based on the parsed route and possible scenarios, build all the scenario classes.
        """

        scenario_instance_vec = []

        if debug_mode:  # draw the trigger positions in and mentio the scenario number scenario definitions. We had selected only the Town02 hence only saw those triggers and scenario numbers
            for scenario in scenario_definitions:
                loc = carla.Location(scenario['trigger_position']['x'],
                                     scenario['trigger_position']['y'],
                                     scenario['trigger_position']['z']) + carla.Location(z=2.0)
                world.debug.draw_point(loc, size=0.3, color=carla.Color(255, 0, 0), life_time=100000)
                world.debug.draw_string(loc, str(scenario['name']), draw_shadow=False,
                                        color=carla.Color(0, 0, 255), life_time=100000, persistent_lines=True)

        for scenario_number, definition in enumerate(scenario_definitions):
            # Get the class possibilities for this scenario number
            scenario_class = NUMBER_CLASS_TRANSLATION[definition['name']]


            '''
            NUMBER_CLASS_TRANSLATION = {
                "Scenario1": ControlLoss,
                "Scenario2": FollowLeadingVehicle,
                "Scenario3": DynamicObjectCrossing,
                "Scenario4": VehicleTurningRoute,
                "Scenario5": OtherLeadingVehicle,
                "Scenario6": ManeuverOppositeDirection,
                "Scenario7": SignalJunctionCrossingRoute,
                "Scenario8": SignalJunctionCrossingRoute,
                "Scenario9": SignalJunctionCrossingRoute,
                "Scenario10": NoSignalJunctionCrossingRoute  # wut :3 
            }

            # SO we define a route, select a town and select the scenario type and that we want,
            # e.g., Scenario10. Then we have our ego run on that route? Or advarserial vehicle as well?
            # I remember that the adversarial had a route as well. Leme look it up. NOPE
            # Only the starting locations of the ego and adversarial vehicles are mentioned.
            # Rest we define the locations of the TriggersRegion objects inside the NoSigJuncCross class
            # and add those objects to the py-tree inside the _create_behavior method.
            # so lets see what is happening here. 
            '''




            # Create the other actors that are going to appear
            if definition['other_actors'] is not None:  # FOFF. We got hellova other actors in in Town02
                
                list_of_actor_conf_instances = self._get_actors_instances(definition['other_actors'])

                #_get_actors_instances -> return list_of_actors  
                    # list of actor config objects of the other vehicles! 
                    # # return object of ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color) ActorConfigurationData
                
                #print("OFF MAN OFF")  #OFF MAN OFF
                #print(definition['other_actors'])

                '''
                {'front': [{'pitch': '0.0', 'x': '45.82', 'y': '270.45', 'yaw': '270.000031', 'z': '1.21'}], 
                'left': [{'pitch': '0.0', 'x': '75.58', 'y': '236.78', 'yaw': '180.000015', 'z': '1.21'}]}
                each other_actors has a transform as well
                '''

                #sys.exit("off")


            else:
                list_of_actor_conf_instances = []
            # Create an actor configuration for the ego-vehicle trigger position

            egoactor_trigger_position = convert_json_to_transform(definition['trigger_position'])
            # trigger positions meant for the ego actor I presume

            #print(definition['trigger_position'])
            # {'pitch': '0', 'x': 7.63, 'y': 109.91, 'yaw': 359.0, 'z': 1.22}
            #sys.exit("ell")


            scenario_configuration = ScenarioConfiguration()

            """
            ScenarioConfiguration()

            This class provides a basic scenario configuration incl.:
            - configurations for all actors
            - town, where the scenario should be executed
            - name of the scenario (e.g. ControlLoss_1)
            - type is the class of scenario (e.g. ControlLoss)

            """

            # for each trigger point we make a new scenario which has other vehicles and 
            # uses the trigger location to make different scenarios
            scenario_configuration.other_actors = list_of_actor_conf_instances
            scenario_configuration.trigger_points = [egoactor_trigger_position]
            scenario_configuration.subtype = definition['scenario_type']  #S4left etc? usually is valid. Yep.
            scenario_configuration.ego_vehicles = [ActorConfigurationData('vehicle.lincoln.mkz2017',
                                                                          ego_vehicle.get_transform(),
                                                                          'hero')]  # making ego ActorConfiguration object here rolenaming it Hero
            route_var_name = "ScenarioRouteNumber{}".format(scenario_number)  # scenario1 or scenario4, do this for all the scenarios
            
            #print(route_var_name)
            '''
            
            ScenarioRouteNumber0
            ScenarioRouteNumber1
            ScenarioRouteNumber2
            ScenarioRouteNumber3

            '''


            scenario_configuration.route_var_name = route_var_name

            try:  # all the scenario classes are avaialble for this
                scenario_instance = scenario_class(world, [ego_vehicle], scenario_configuration,
                                                   criteria_enable=False, timeout=timeout)

                # for each trigger point we make a new scenario which has other vehicles and 
                # uses the trigger location to make different scenarios



                # Do a tick every once in a while to avoid spawning everything at the same time
                if scenario_number % scenarios_per_tick == 0:
                    if CarlaDataProvider.is_sync_mode():
                        world.tick()
                    else:
                        world.wait_for_tick()

                scenario_number += 1
            except Exception as e:      # pylint: disable=broad-except
                if debug_mode:
                    traceback.print_exc()
                print("Skipping scenario '{}' due to setup error: {}".format(definition['name'], e))
                continue

            scenario_instance_vec.append(scenario_instance)

        return scenario_instance_vec

    def _get_actors_instances(self, list_of_antagonist_actors):
        """
        Get the full list of actor instances.
        """

        def get_actors_from_list(list_of_actor_def):
            """
                Receives a list of actor definitions and creates an actual list of ActorConfigurationObjects
            """
            sublist_of_actors = []
            for actor_def in list_of_actor_def:
                sublist_of_actors.append(convert_json_to_actor(actor_def))  # return object of ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color) ActorConfigurationData

            return sublist_of_actors

        list_of_actors = []
        # Parse vehicles to the left
        if 'front' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['front'])

        if 'left' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['left'])

        if 'right' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['right'])

        return list_of_actors  
        # list of actor config objects of the other vehicles! 
        # # return object of ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color) ActorConfigurationData

    # pylint: enable=no-self-use

    def _initialize_actors(self, config):
        """
        Set other_actors to the superset of all scenario actors
        """

        # Create the background activity of the route
        town_amount = {
            'Town01': 120,
            'Town02': 100,
            'Town03': 120,
            'Town04': 200,
            'Town05': 120,
            'Town06': 150,
            'Town07': 110,
            'Town08': 180,
            'Town09': 300,
            'Town10': 120,
        }

        amount = town_amount[config.town] if config.town in town_amount else 0

        new_actors = CarlaDataProvider.request_new_batch_actors('vehicle.*',
                                                                amount,
                                                                carla.Transform(),
                                                                autopilot=True,
                                                                random_location=True,
                                                                rolename='background')

        if new_actors is None:
            raise Exception("Error: Unable to add the background activity, all spawn points were occupied")

        for _actor in new_actors:
            self.other_actors.append(_actor)

        # Add all the actors of the specific scenarios to self.other_actors
        for scenario in self.list_scenarios:
            self.other_actors.extend(scenario.other_actors)

    def _create_behavior(self):
        """
        Basic behavior do nothing, i.e. Idle
        """
        scenario_trigger_distance = 1.5  # Max trigger distance between route and scenario

        behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        subbehavior = py_trees.composites.Parallel(name="Behavior",
                                                   policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        scenario_behaviors = []
        blackboard_list = []

        for i, scenario in enumerate(self.list_scenarios):
            if scenario.scenario.behavior is not None:
                route_var_name = scenario.config.route_var_name
                if route_var_name is not None:
                    scenario_behaviors.append(scenario.scenario.behavior)
                    blackboard_list.append([scenario.config.route_var_name,
                                            scenario.config.trigger_points[0].location])
                else:
                    name = "{} - {}".format(i, scenario.scenario.behavior.name)
                    oneshot_idiom = oneshot_behavior(name,
                                                     behaviour=scenario.scenario.behavior,
                                                     name=name)
                    scenario_behaviors.append(oneshot_idiom)

        # Add behavior that manages the scenarios trigger conditions
        scenario_triggerer = ScenarioTriggerer(
            self.ego_vehicles[0],
            self.route,
            blackboard_list,
            scenario_trigger_distance,
            repeat_scenarios=False
        )

        subbehavior.add_child(scenario_triggerer)  # make ScenarioTriggerer the first thing to be checked
        subbehavior.add_children(scenario_behaviors)
        subbehavior.add_child(Idle())  # The behaviours cannot make the route scenario stop
        behavior.add_child(subbehavior)

        return behavior

    def _create_test_criteria(self):
        """
        """

        criteria = []

        route = convert_transform_to_location(self.route)

        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=False)

        route_criterion = InRouteTest(self.ego_vehicles[0],
                                      route=route,
                                      offroad_max=30,
                                      terminate_on_failure=True)

        completion_criterion = RouteCompletionTest(self.ego_vehicles[0], route=route)

        outsidelane_criterion = OutsideRouteLanesTest(self.ego_vehicles[0], route=route)

        red_light_criterion = RunningRedLightTest(self.ego_vehicles[0])

        stop_criterion = RunningStopTest(self.ego_vehicles[0])

        blocked_criterion = ActorSpeedAboveThresholdTest(self.ego_vehicles[0],
                                                         speed_threshold=0.1,
                                                         below_threshold_max_time=90.0,
                                                         terminate_on_failure=True)

        criteria.append(completion_criterion)
        criteria.append(collision_criterion)
        criteria.append(route_criterion)
        criteria.append(outsidelane_criterion)
        criteria.append(red_light_criterion)
        criteria.append(stop_criterion)
        criteria.append(blocked_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
