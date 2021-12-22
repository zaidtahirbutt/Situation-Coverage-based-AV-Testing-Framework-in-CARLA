#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Non-signalized junctions: crossing negotiation:

The hero vehicle is passing through a junction without traffic lights
And encounters another vehicle passing across the junction.
"""

import py_trees
import carla
import sys


from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      SyncArrival,
                                                                      KeepVelocity,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerRegion
from srunner.scenarios.basic_scenario import BasicScenario


class NoSignalJunctionCrossing(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20  # for what
    _ego_vehicle_driven_distance = 105  # for what

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15  # for what

    '''
        From scenario_runnerZ.py file
                            #print(member[1])  # <class 'no_signal_junction_crossing.NoSignalJunctionCrossing'> was returned in scenario class
                #i.e., NoSignalJunctionCrossing class (address maybe, as an imported library/module) was returned to this variable scenario_class
                #because we are initializing NoSignalJunctionCrossing class by initializing scenario_class() below

                # Config has all parameters like ego and other veh, their details
                # like locations vheicle transforms and types, weather, towns, trigger points (ego veh spawn loc), routes if any etc
                scenario = scenario_class(self.world,
                                          self.ego_vehicles,
                                          config,
                                          self._args.randomize,
                                          self._args.debug)


    '''






    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        #BasicScenario class
                
                #if self.config.friction is not None: we can introduce friction triggers in conig file!
                
                # Initializing adversarial actors
                # self._initialize_actors(config)
                #Default initialization of other actors.
                #Override this method in child class to provide custom initialization.

                #self.ego_vehicles = ego_vehicles
                
                #self.name = name
                
                #self.config = config  # config object
                
        super(NoSignalJunctionCrossing, self).__init__("NoSignalJunctionCrossing",   #passing the class in the super function
                                                       ego_vehicles,
                                                       config,
                                                       world,
                                                       debug_mode,
                                                       criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_actor_transform = config.other_actors[0].transform  # only one actor here right?
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)

        # print(config.other_actors[0].transform.location.z)  # 0.5
        # print(config.other_actors[0].transform.location.z - 500)  #  -499.5 tf
        # sys.exit("yello")

        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)  # Return: carla.Actor
        first_vehicle.set_simulate_physics(enabled=False)  # Enables or disables the simulation of physics on this actor.
        self.other_actors.append(first_vehicle)

    def _create_behavior(self): #overloading original method in the parent class
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant (advarsarial vehicle) to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """


        # print("fffffaaa")
        # print("whatup")  # Sequence
        # sys.exit("off")  # so thisss is running!

        #<ego_vehicle x="-74.32" y="-50" z="0.5" yaw="270" model="vehicle.lincoln.mkz2017" />
        #<other_actor x="-105" y="-136" z="0.5" yaw="0" model="vehicle.tesla.model3" /> 

        # Creating leaf nodes
        start_other_trigger = InTriggerRegion(
            self.ego_vehicles[0],  # ego passed
            -80, -70,  # min max x, ego vehicle moving along y axis, as x axis values are the same as inital condition spawn locs for ego.
            -75, -60)  # min max y

        sync_arrival = SyncArrival(  # this trigger and the pass_through below running in parallel
            self.other_actors[0], self.ego_vehicles[0],  # other and ego passed
            carla.Location(x=-74.63, y=-136.34))  # where the actoes should meet tf

        pass_through_trigger = InTriggerRegion(  # Trigger box  # Lets draw a box here in the map. ! 
            self.ego_vehicles[0],
            -90, -70,  # this trigger box is for ego vehicle and it has same values of x axis as for the initial spawn x axis loc of ego, only y axis values were changed.
            -124, -119)

        keep_velocity_other = KeepVelocity(
            self.other_actors[0],
            self._other_actor_target_velocity)  # 15 ms-1

        stop_other_trigger = InTriggerRegion(
            self.other_actors[0],  # this trigger box is for other vehicles and it's loc suggests other vehicle is moving along x axis cx y-axis trigger box values are in the same range as it's initial spawning location
            -45, -35,  # apply the damn brakes in this region as seen in the parallel pytree node below xD
            -140, -130)  # <other_actor x="-105" y="-136", means other vehicle is moving along x axis cx y-axis trigger box values are in the same range as it's initial spawning location

        stop_other = StopVehicle(
            self.other_actors[0],  # other vehicle
            self._other_actor_max_brake)  # 1.0

        end_condition = InTriggerRegion(
            self.ego_vehicles[0],  # for ego vehicle
            -90, -70,  # same x axis range as it is travelling across y axis
            -170, -156
        )


        #ZZZZZZZZSo we got a bunch of trigger boxes and behaviour functions set up.

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()  # see def of sequence below
        scenario_sequence = py_trees.composites.Sequence()
        sync_arrival_parallel = py_trees.composites.Parallel(  #  see def of Parallel below
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree

        #A sequence will progressively tick over each of its children so long as
        #each child returns :data:`~py_trees.common.Status.FAILURE` or :data:`~py_trees.common.Status.RUNNING` the sequence will halt and the parent will adopt
        #the result of this child. If it reaches the last child, it returns with
        #that result regardless.
        root.add_child(scenario_sequence)  # PROGRESSIVELY ticks as stated above meaning addition sequence of these children is the main thing for a scenario
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(sync_arrival_parallel)  # parallel node
        scenario_sequence.add_child(keep_velocity_other_parallel)  # parallel node
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))

        #parallel nodes. Ticks every child every time the parallel is run (a poor man's form of parallelism). Parallels enable a kind of concurrency
        sync_arrival_parallel.add_child(sync_arrival)  # these two (sync_arrival, pass_through_trigger) added in one parallel node 
        sync_arrival_parallel.add_child(pass_through_trigger)  #     The condition terminates with SUCCESS, when the actor reached the target region
        # so when pass_through_trigger returns SUCCESS, the tree gets out of both these parallel nodes (sync arrival and pass through) and moves on to next, else the other vehicle keeps synching with other vehicle


        keep_velocity_other_parallel.add_child(keep_velocity_other)  # does ticking every child mean mean every method of class is run? I don't think so. Let's see.
        keep_velocity_other_parallel.add_child(stop_other_trigger)  # these two (keep_velocity_other, stop_other_trigger) added in one parallel node

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
