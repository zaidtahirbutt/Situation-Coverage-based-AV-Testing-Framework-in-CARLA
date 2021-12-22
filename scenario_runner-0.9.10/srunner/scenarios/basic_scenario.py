#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provide BasicScenario, the basic class of all the scenarios.
"""

from __future__ import print_function

import operator
import py_trees
import sys

import carla

import srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions as conditions
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.weather_sim import WeatherBehavior
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import UpdateAllActorControls


class BasicScenario(object):    # directly inherit from object  

    """
    Base class for user-defined scenario


    So BaseScenario is inheriting NoSignalJunctionCrossing
    super(NoSignalJunctionCrossing, self).__init__("NoSignalJunctionCrossing",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)


    """

    #name="NoSignalJunctionCrossing" for our case right now

    def __init__(self, name, ego_vehicles, config, world,
                 debug_mode=False, terminate_on_failure=False, criteria_enable=False):  # we are using collision criteria
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self.other_actors = []
        if not self.timeout:     # pylint: disable=access-member-before-definition
            self.timeout = 60    # If no timeout was provided, set it to 60 seconds

        self.criteria_list = []  # List of evaluation criteria
        self.scenario = None

        self.ego_vehicles = ego_vehicles  #list of carla.vehicles xD
        self.name = name  # = "NoSignalJunctionCrossing"
        self.config = config  # config object l
        self.terminate_on_failure = terminate_on_failure

        self._initialize_environment(world)  # weather and frictions 

        '''
            world.set_weather(self.config.weather)
            # Set the appropriate road friction
            if self.config.friction is not None:
                friction_bp = world.get_blueprint_library().find('static.trigger.friction')
                extent = carla.Location(1000000.0, 1000000.0, 1000000.0)
                friction_bp.set_attribute('friction', str(self.config.friction))
                friction_bp.set_attribute('extent_x', str(extent.x))
                friction_bp.set_attribute('extent_y', str(extent.y))
                friction_bp.set_attribute('extent_z', str(extent.z))
        
                # Spawn Trigger Friction
                transform = carla.Transform()
                transform.location = carla.Location(-10000.0, -10000.0, 0.0)
                world.spawn_actor(friction_bp, transform)
        '''

        # Initializing adversarial actors
        self._initialize_actors(config)  #  overwritten in NoSigJunc child class # spawn other advarsarial vehicles at the given locations and setting autopilots

        if CarlaDataProvider.is_sync_mode():
            world.tick()
        else:
            world.wait_for_tick()

        # Setup scenario
        if debug_mode:
            py_trees.logging.level = py_trees.logging.Level.DEBUG

        #Use _create_behavior() from the scenario class that you are using and overload this method in this basic scenario class
        behavior = self._create_behavior()  # so self here is the object coming from NoSignalJunc class and when we call this method 
        #the method from the object is being called and the parent class method (basic scenario), is being overridden here.

        #sakht behaviour created in the above function

        criteria = None
        if criteria_enable:
            criteria = self._create_test_criteria()

        # Add a trigger condition for the behavior to ensure the behavior is only activated, when it is relevant
        behavior_seq = py_trees.composites.Sequence()
        trigger_behavior = self._setup_scenario_trigger(config)  # This is the pytree seq that waits for the ego to move :3 # THIS IS WRONG ON THE RIGHT AND RETHINK AND RELOOK AT EVERY SUCH COMMENT AGAIN => # return None cx no route  # start_location = config.trigger_points[0].location     # start location of the scenario (ego veh spawn loc)
        
        #print(trigger_behavior)  # <srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions.InTimeToArrivalToLocation object at 0x000001B1BC158388>S
        #sys.exit("fIT")

        if trigger_behavior:  # WRONG COMMENT On THE RIGHT AGAIN. READ ABOVE!  # None
            behavior_seq.add_child(trigger_behavior)  # so TRIGGER BEAHVIOUS COMES FIRST!!!!!!!

        if behavior is not None:
            behavior_seq.add_child(behavior)  # add pytree seq to another pytree seq
            behavior_seq.name = behavior.name

            # print("fffffaaa")
            # print(behavior_seq.name)  # Sequence
            # sys.exit("off")

        end_behavior = self._setup_scenario_end(config)
        if end_behavior:
            behavior_seq.add_child(end_behavior)  # none cx no route

        # FIRST behaviour in the pytree behaviour_seq is checking if ego vehicle has gained a very small velocity at it's spawn location!!!
        self.scenario = Scenario(behavior_seq, criteria, self.name, self.timeout, self.terminate_on_failure)  # 60 s timeout, criteria enabled as collision criteria, terminate on failure is false i think


        """
        Basic scenario class. This class holds the behavior_tree describing the
        scenario and the test criteria.

        The user must not modify this class.

        Important parameters:
        - behavior: User defined scenario with py_tree
        - criteria_list: List of user defined test criteria with py_tree
        - timeout (default = 60s): Timeout of the scenario in seconds
        - terminate_on_failure: Terminate scenario on first failure
        """

    def _initialize_environment(self, world):
        """
        Default initialization of weather and road friction.
        Override this method in child class to provide custom initialization.
        """

        # Set the appropriate weather conditions
        world.set_weather(self.config.weather)

        # Set the appropriate road friction
        if self.config.friction is not None:
            friction_bp = world.get_blueprint_library().find('static.trigger.friction')
            extent = carla.Location(1000000.0, 1000000.0, 1000000.0)
            friction_bp.set_attribute('friction', str(self.config.friction))
            friction_bp.set_attribute('extent_x', str(extent.x))
            friction_bp.set_attribute('extent_y', str(extent.y))
            friction_bp.set_attribute('extent_z', str(extent.z))

            # Spawn Trigger Friction
            transform = carla.Transform()
            transform.location = carla.Location(-10000.0, -10000.0, 0.0)
            world.spawn_actor(friction_bp, transform)

    def _initialize_actors(self, config):  # spawn other advarsarial vehicles at the given locations and setting autopilots
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        if config.other_actors:
            new_actors = CarlaDataProvider.request_new_actors(config.other_actors)
            if not new_actors:
                raise Exception("Error: Unable to add actors")

            for new_actor in new_actors:
                self.other_actors.append(new_actor)

    def _setup_scenario_trigger(self, config):
        """
        This function creates a trigger maneuver, that has to be finished before the real scenario starts.
        This implementation focuses on the first available ego vehicle.

        The function can be overloaded by a user implementation inside the user-defined scenario class.
        """
        start_location = None
        if config.trigger_points and config.trigger_points[0]:
            start_location = config.trigger_points[0].location     # start location of the scenario (ego veh spawn loc)

        ego_vehicle_route = CarlaDataProvider.get_ego_vehicle_route()  # need this later

        '''

        for route based

        from Carla_data_provider

            @staticmethod
            def set_ego_vehicle_route(route):
                """
                Set the route of the ego vehicle

                @todo extend ego_vehicle_route concept to support multi ego_vehicle scenarios
                """
                CarlaDataProvider._ego_vehicle_route = route

            @staticmethod
            def get_ego_vehicle_route():
                """
                returns the currently set route of the ego vehicle
                Note: Can be None
                """
                return CarlaDataProvider._ego_vehicle_route

        '''

        if start_location:
            if ego_vehicle_route:
                if config.route_var_name is None:  # pylint: disable=no-else-return
                    return conditions.InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                            ego_vehicle_route,
                                                                            start_location,
                                                                            5)
                else:
                    check_name = "WaitForBlackboardVariable: {}".format(config.route_var_name)
                    return conditions.WaitForBlackboardVariable(name=check_name,
                                                                variable_name=config.route_var_name,
                                                                variable_value=True,
                                                                var_init_value=False)

            return conditions.InTimeToArrivalToLocation(self.ego_vehicles[0],  # ef __init__(self, actor, time, location...
                                                        2.0,
                                                        start_location)  # start location is the spawn location in the xml file of ego

        return None

    def _setup_scenario_end(self, config):
        """
        This function adds and additional behavior to the scenario, which is triggered
        after it has ended.

        The function can be overloaded by a user implementation inside the user-defined scenario class.
        """
        ego_vehicle_route = CarlaDataProvider.get_ego_vehicle_route()

        if ego_vehicle_route:
            if config.route_var_name is not None:
                set_name = "Reset Blackboard Variable: {} ".format(config.route_var_name)
                return py_trees.blackboard.SetBlackboardVariable(name=set_name,
                                                                 variable_name=config.route_var_name,
                                                                 variable_value=False)
        return None

    def _create_behavior(self):  #this never runs! 
        #https://stackoverflow.com/questions/25062114/calling-child-class-method-from-parent-class-file-in-python
        """
        Pure virtual function to setup user-defined scenario behavior
        """

        # print("fffffaaa")
        # print("whatup")  # Sequence
        # sys.exit("off")  # so thisss didn't run!!!!!! Lets see if the original class one ran!

        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def _create_test_criteria(self):
        """
        Pure virtual function to setup user-defined evaluation criteria for the
        scenario
        """
        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def change_control(self, control):  # pylint: disable=no-self-use
        """
        This is a function that changes the control based on the scenario determination
        :param control: a carla vehicle control
        :return: a control to be changed by the scenario.

        Note: This method should be overriden by the user-defined scenario behavior
        """
        return control

    def remove_all_actors(self):
        """
        Remove all actors
        """
        for i, _ in enumerate(self.other_actors):
            if self.other_actors[i] is not None:
                if CarlaDataProvider.actor_id_exists(self.other_actors[i].id):
                    CarlaDataProvider.remove_actor_by_id(self.other_actors[i].id)
                self.other_actors[i] = None
        self.other_actors = []


class Scenario(object):

    """
    Basic scenario class. This class holds the behavior_tree describing the
    scenario and the test criteria.

    The user must not modify this class.

    Important parameters:
    - behavior: User defined scenario with py_tree
    - criteria_list: List of user defined test criteria with py_tree
    - timeout (default = 60s): Timeout of the scenario in seconds
    - terminate_on_failure: Terminate scenario on first failure
    """

    def __init__(self, behavior, criteria, name, timeout=60, terminate_on_failure=False):
        self.behavior = behavior  # py tree
        self.test_criteria = criteria  # collision criteria
        self.timeout = timeout
        self.name = name  # NoSignalJunctionCrossing

        if self.test_criteria is not None and not isinstance(self.test_criteria, py_trees.composites.Parallel):  # isinstance -> Return true if the object argument is an instance of the classinfo argument, or of a (direct, indirect or virtual) subclass thereof
            # list of nodes
            for criterion in self.test_criteria:
                criterion.terminate_on_failure = terminate_on_failure

            # Create py_tree for test criteria
            self.criteria_tree = py_trees.composites.Parallel(
                name="Test Criteria",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
            )
            self.criteria_tree.add_children(self.test_criteria)
            self.criteria_tree.setup(timeout=1)
        else:
            self.criteria_tree = criteria

        # Create node for timeout
        self.timeout_node = TimeOut(self.timeout, name="TimeOut")

        # Create overall py_tree  # so this overall tree is a parallel tree with following children that are updated together with one tick, cool.
        self.scenario_tree = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)   # name = "NoSignalJunctionCrossing"
        if behavior is not None:
            self.scenario_tree.add_child(self.behavior)  # main pytree from NoSignalJunction class along with none added first in BasicScenario class cx of no route
        self.scenario_tree.add_child(self.timeout_node)
        self.scenario_tree.add_child(WeatherBehavior())  # All these children tick at the same time, lets go deeper into behavior
        self.scenario_tree.add_child(UpdateAllActorControls())

        if criteria is not None:
            self.scenario_tree.add_child(self.criteria_tree)
        self.scenario_tree.setup(timeout=1)

    def _extract_nodes_from_tree(self, tree):  # pylint: disable=no-self-use
        """
        Returns the list of all nodes from the given tree
        """
        node_list = [tree]
        more_nodes_exist = True
        while more_nodes_exist:
            more_nodes_exist = False
            for node in node_list:
                if node.children:
                    node_list.remove(node)
                    more_nodes_exist = True
                    for child in node.children:
                        node_list.append(child)

        if len(node_list) == 1 and isinstance(node_list[0], py_trees.composites.Parallel):
            return []

        return node_list

    def get_criteria(self):
        """
        Return the list of test criteria (all leave nodes)
        """
        criteria_list = self._extract_nodes_from_tree(self.criteria_tree)
        return criteria_list

    def terminate(self):
        """
        This function sets the status of all leaves in the scenario tree to INVALID
        """
        # Get list of all nodes in the tree
        node_list = self._extract_nodes_from_tree(self.scenario_tree)

        # Set status to INVALID
        for node in node_list:
            node.terminate(py_trees.common.Status.INVALID)

        # Cleanup all instantiated controllers
        actor_dict = {}
        try:
            check_actors = operator.attrgetter("ActorsWithController")
            actor_dict = check_actors(py_trees.blackboard.Blackboard())
        except AttributeError:
            pass
        for actor_id in actor_dict:
            actor_dict[actor_id].reset()
        py_trees.blackboard.Blackboard().set("ActorsWithController", {}, overwrite=True)

'''
Script for understanding inheritance in classes and method/function overloading

# Online Python compiler (interpreter) to run Python online.
# Write Python 3 code in this online editor and run it.
print("Hello world")
class A(object):  # parent
    #def __init__(self):
    def __init__(self, z):
        
        self.x = self.methodB()
        z = self.methodB()  # one print "am in methodB" is just from this!
        
    def methodA(self):
        print("in methodA")
    
    
    def methodB(self):
        raise NotImplementedError("Must override methodB")

class B(A):  # child
    y = []
    def __init__(self):
        #super(B, self).__init__()
        super(B, self).__init__(self.y)
        
    def methodB(self):
        print("am in methodB")
        
getB = B()
getB.methodB()
getB.x
#getB.y # dealt with this --  # AttributeError: 'B' object has no attribute 'y'
#a = getB.y nothing happens here
#q = a


This is the output
    Hello world
am in methodB
am in methodB
am in methodB


'''