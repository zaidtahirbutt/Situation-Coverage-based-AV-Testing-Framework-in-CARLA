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
from srunner.scenariomanager.scenarioatomics.atomic_behaviors5 import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      SyncArrival,
                                                                      KeepVelocity,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerRegion
from srunner.scenarios.basic_scenario import BasicScenario
#from automatic_control_agent_z5_other_veh import *

from util.lane_explorerZ3 import draw_junction  # def draw_junction(debug, junction, l_time=10, wp_str_flag): debu is world.debug instance variable object
cyan = carla.Color(0, 255, 255)

class IntersectionScenarioZ_11(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """
    #wp_str_flag = None  # enable or disable printing of waypoint locations as strings

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20  # for what
    _ego_vehicle_driven_distance = 105  # for what

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15  # for what
    junction_waypoint_loc = carla.Location(-74.63, -136.34, 0)  # which is actually the sync arrival location

    # other vehicle AgentZ

    #other_veh_agentZ = None

    def __init__(self, world, ego_vehicles, config, intersection_situations, other_veh_agentZ, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60, show_int_wps=False, show_int_wp_locs=False):
        """
        Setup all relevant parameters and create scenario
        """


        self.debug = world.debug

        # ****** Warning: This method world.get_map() does call the simulation. It is expensive, and should only be called once. *******************************
        self.map = world.get_map()  # getting world map
        # maybe in the future versions of situation coverage, we will call this just once.

        self.show_int_wps = show_int_wps

        self.wp_str_flag = show_int_wp_locs  # enable or disable printing of waypoint locations as strings

        self._other_actor_transform = None

        # Timeout of scenario in seconds
        self.timeout = timeout


        # othervehicle AgentZ
        self.other_veh_agentZ = other_veh_agentZ

        # Create IntersectionSituations object here so that it is accessible throughout the class
        #self.intersection_situations = intersection_situations  # don't need multiple copies if I can use it's values here

        # Definning different trigger variables here so that it doesn't get too messy later
        self.start_other_trigger_min_x = intersection_situations.ego_veh.ego_startothertrigger_dict["key_ego_startothertrigger_min_x"]
        self.start_other_trigger_max_x = intersection_situations.ego_veh.ego_startothertrigger_dict["key_ego_startothertrigger_max_x"]
        self.start_other_trigger_min_y = intersection_situations.ego_veh.ego_startothertrigger_dict["key_ego_startothertrigger_min_y"]
        self.start_other_trigger_max_y = intersection_situations.ego_veh.ego_startothertrigger_dict["key_ego_startothertrigger_max_y"]
        self.start_other_trigger_midpoint_distance_x = abs(self.start_other_trigger_max_x - self.start_other_trigger_min_x)/2
        self.start_other_trigger_midpoint_distance_y = abs(self.start_other_trigger_max_y - self.start_other_trigger_min_y)/2


        self.sync_arrival_x = intersection_situations.conflictpoint_syncarrival_loc_x
        self.sync_arrival_y = intersection_situations.conflictpoint_syncarrival_loc_y


        self.pass_through_trigger_min_x = intersection_situations.ego_veh.ego_passthroughtrigger_dict["key_ego_passthroughtrigger_min_x"]
        self.pass_through_trigger_max_x = intersection_situations.ego_veh.ego_passthroughtrigger_dict["key_ego_passthroughtrigger_max_x"]
        self.pass_through_trigger_min_y = intersection_situations.ego_veh.ego_passthroughtrigger_dict["key_ego_passthroughtrigger_min_y"]
        self.pass_through_trigger_max_y = intersection_situations.ego_veh.ego_passthroughtrigger_dict["key_ego_passthroughtrigger_max_y"]
        self.pass_through_trigger_midpoint_distance_x = abs(self.pass_through_trigger_max_x - self.pass_through_trigger_min_x)/2
        self.pass_through_trigger_midpoint_distance_y = abs(self.pass_through_trigger_max_y - self.pass_through_trigger_min_y)/2


        self.stop_other_trigger_min_x = intersection_situations.other_veh.other_vehicle_stopothertrigger_dict["key_other_vehicle_stopothertrigger_min_x"]
        self.stop_other_trigger_max_x = intersection_situations.other_veh.other_vehicle_stopothertrigger_dict["key_other_vehicle_stopothertrigger_max_x"]
        self.stop_other_trigger_min_y = intersection_situations.other_veh.other_vehicle_stopothertrigger_dict["key_other_vehicle_stopothertrigger_min_y"]
        self.stop_other_trigger_max_y = intersection_situations.other_veh.other_vehicle_stopothertrigger_dict["key_other_vehicle_stopothertrigger_max_y"]
        self.stop_other_trigger_midpoint_distance_x = abs(self.stop_other_trigger_max_x - self.stop_other_trigger_min_x)/2
        self.stop_other_trigger_midpoint_distance_y = abs(self.stop_other_trigger_max_y - self.stop_other_trigger_min_y)/2
        

        self.other_vehicle_goal_loc_x = intersection_situations.other_veh.other_vehicle_stopothertrigger_dict["key_other_vehicle_destination_x"]
        self.other_vehicle_goal_loc_y = intersection_situations.other_veh.other_vehicle_stopothertrigger_dict["key_other_vehicle_destination_y"]

        self.other_vehicle_goal_carla_Location = carla.Location(self.other_vehicle_goal_loc_x, self.other_vehicle_goal_loc_y, 0)        

        self.end_condition_min_x = intersection_situations.ego_veh.ego_endconditiontrigger_dict["key_ego_endconditiontrigger_min_x"]
        self.end_condition_max_x = intersection_situations.ego_veh.ego_endconditiontrigger_dict["key_ego_endconditiontrigger_max_x"]
        self.end_condition_min_y = intersection_situations.ego_veh.ego_endconditiontrigger_dict["key_ego_endconditiontrigger_min_y"]
        self.end_condition_max_y = intersection_situations.ego_veh.ego_endconditiontrigger_dict["key_ego_endconditiontrigger_max_y"]
        self.end_condition_midpoint_distance_x = abs(self.end_condition_max_x - self.end_condition_min_x)/2
        self.end_condition_midpoint_distance_y = abs(self.end_condition_max_y - self.end_condition_min_y)/2



        super(IntersectionScenarioZ_11, self).__init__("IntersectionScenarioZ_11",   #passing the class in the super function
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

        #print(first_vehicle_transform.location)  # why z - 500? 
        # Location(x=-105.000000, y=-136.000000, z=-499.500000)  # why inside the earth at -499.5? Look at spawning function
        # <other_actor x="-105" y="-136" z="0.5" yaw="0" model="vehicle.tesla.model3" />
        '''
        Hi,
        the z-value is set to -500 to enable route-based scenarios. Imagine all traffic participants 
        would be spawned directly at surface level. This would quickly result in a traffic jam. Hence,
        the actors are under the surface, until they become relevant. As soon as this is the case, they
        are moved to surface level (usually z=0). If you want to run a single scenario only, you can remove
        all this parts, or better directly go with OpenSCENARIO.
        So they are set_transform-ed to z = 0 location and it doesn't return an error when actor is spawned under
        the earth!
        '''
        if self.show_int_wps:
            # ********************** draw starting waypoint of the other vehicle ZZz ***********************
            self.debug.draw_point(location=self._other_actor_transform.location, size=0.1, life_time=0)  # size=0.1 works only
            
            self.debug.draw_string(self._other_actor_transform.location, "Other_vehicle spawn location", color=carla.Color(255,255,255,0), life_time=50000)  # due to some reason life_time=0 doesn't work for draw string

            # ********************** draw spawn location of the ego vehicle ZZz ***********************
            self.debug.draw_point(location=self.config.ego_vehicles[0].transform.location, size=0.1, life_time=0)  # size=0.1 works only
            #self.debug.draw_point(location=self.ego_vehicles[0].get_location(), size=0.1, life_time=0)  # size=0.1 works only
            # we can get these ego spawn loc details from config as well


            self.debug.draw_string(self.config.ego_vehicles[0].transform.location, "Ego vehicle spawn location", color=carla.Color(255,255,255,0), life_time=50000)  # due to some reason life_time=0 doesn't work for draw string
            #self.debug.draw_string(self.ego_vehicles[0].get_location(), "Ego vehicle spawn location", color=carla.Color(255,255,255,0), life_time=50000)  # due to some reason life_time=0 doesn't work for draw string
            
            # ****************** draw bounding box of start_other_trigger trigger location ***************

            # draw_box(self, box, rotation, thickness=0.1, color=(255,0,0), life_time=-1.0)
            # carla.BoundingBox__init__(self, location, extent), extent -> (carla.Vector3D – meters) – Vector containing half the size of the box for every axis.
            
            # intersection leg1 BOX
            self.debug.draw_box(carla.BoundingBox(carla.Location(self.start_other_trigger_min_x + self.start_other_trigger_midpoint_distance_x, self.start_other_trigger_min_y + self.start_other_trigger_midpoint_distance_y, 0), carla.Vector3D(self.start_other_trigger_midpoint_distance_x, self.start_other_trigger_midpoint_distance_y, 2)), carla.Rotation(), life_time=0)  
            
            # intersection leg2 BOX
            #self.debug.draw_box(carla.BoundingBox(carla.Location(-143, -106.5, 0), carla.Vector3D(5, 7.5, 2)), carla.Rotation(), life_time=0)  
            
            # intersection leg1 LOC STRING
            #self.debug.draw_string(carla.Location(-75, -67, 0), "start_other_trigger location", color=carla.Color(255,255,255,0), life_time=50_000)  # due to some reason life_time=0 doesn't work for draw string
            self.debug.draw_string(carla.Location(self.start_other_trigger_min_x + self.start_other_trigger_midpoint_distance_x, self.start_other_trigger_min_y + self.start_other_trigger_midpoint_distance_y, 0), f"start_other_trigger location x={self.start_other_trigger_min_x + self.start_other_trigger_midpoint_distance_x}, y={self.start_other_trigger_min_y + self.start_other_trigger_midpoint_distance_y}", color=cyan, life_time=50_000)  # due to some reason life_time=0 doesn't work for draw string
             
            # intersection leg2 LOC STRING
            #self.debug.draw_string(carla.Location(-143, -106.5, 0), f"start_other_trigger location x={-143}, y={-106.5}", color=cyan, life_time=50_000)  # due to some reason life_time=0 doesn't work for draw string

            # had to input the carla.Location with z = 0, otherwise it was giving error.
            # like carla.Location(x, y, z=0) instead of carla.Location(x, y)

            # so after ego passes through this trigger, the other vehcile starts to sync with it according to the gain values we have set in the sync execution node

            # ****************** draw bounding box of sync_arrival location ***************
            self.debug.draw_box(carla.BoundingBox(carla.Location(self.sync_arrival_x, self.sync_arrival_y, 0), carla.Vector3D(0.5, 0.5, 2)), carla.Rotation(), life_time=0)

            #self.debug.draw_string(carla.Location(-74.63, -136.34, 0), "sync_arrival other with ego target_location", color=carla.Color(255,255,255,0), life_time=50000)  # due to some reason life_time=0 doesn't work for draw string
            self.debug.draw_string(carla.Location(self.sync_arrival_x, self.sync_arrival_y, 0), f"sync_arrival other with ego target_location x={self.sync_arrival_x}, y={self.sync_arrival_y}", color=cyan, life_time=50000)  # due to some reason life_time=0 doesn't work for draw string



            # ****************** draw bounding box of pass_through_trigger trigger location (parallel node with sync arrival) ***************
            self.debug.draw_box(carla.BoundingBox(carla.Location(self.pass_through_trigger_min_x + self.pass_through_trigger_midpoint_distance_x, self.pass_through_trigger_min_y + self.pass_through_trigger_midpoint_distance_y, 0), carla.Vector3D(self.pass_through_trigger_midpoint_distance_x, self.pass_through_trigger_midpoint_distance_y, 2)), carla.Rotation(), life_time=0)
            
            # ****************** draw bounding box of two other pass_through_trigger trigger location (parallel node with sync arrival) ***************
            # left
            #self.debug.draw_box(carla.BoundingBox(carla.Location(-104.14, -138.5, 0), carla.Vector3D(2.5, 10, 2)), carla.Rotation(), life_time=0)
            # right
            #self.debug.draw_box(carla.BoundingBox(carla.Location(-60.65, -137.5, 0), carla.Vector3D(2.5, 10, 2)), carla.Rotation(), life_time=0)
            
            # write their names after drawing as well.
            # draw_string(self, location, text, draw_shadow=False, color=(255,0,0), life_time=-1.0)
            #self.debug.draw_string(carla.Location(-80, -121.5, 0), "pass_through_trigger ego", color=carla.Color(255,255,255,0), life_time=50000)  # due to some reason life_time=0 doesn't work for draw string
            self.debug.draw_string(carla.Location(self.pass_through_trigger_min_x + self.pass_through_trigger_midpoint_distance_x, self.pass_through_trigger_min_y + self.pass_through_trigger_midpoint_distance_y, 0), f"pass_through_trigger ego x={self.pass_through_trigger_min_x + self.pass_through_trigger_midpoint_distance_x}, y={self.pass_through_trigger_min_y + self.pass_through_trigger_midpoint_distance_y}", color=cyan, life_time=50000)  # due to some reason life_time=0 doesn't work for draw string


            # ****************** draw bounding box of stop_other_trigger trigger location ***************
            self.debug.draw_box(carla.BoundingBox(carla.Location(self.stop_other_trigger_min_x + self.stop_other_trigger_midpoint_distance_x, self.stop_other_trigger_min_y + self.stop_other_trigger_midpoint_distance_y, 0), carla.Vector3D(self.stop_other_trigger_midpoint_distance_x, self.stop_other_trigger_midpoint_distance_y, 2)), carla.Rotation(), life_time=0)

            #self.debug.draw_string(carla.Location(-40, -135, 0), "stop_other_trigger", color=carla.Color(255,255,255,0), life_time=50000)  # due to some reason life_time=0 doesn't work for draw string
            self.debug.draw_string(carla.Location(self.stop_other_trigger_min_x + self.stop_other_trigger_midpoint_distance_x + 3, self.stop_other_trigger_min_y + self.stop_other_trigger_midpoint_distance_y + 3, 0), f"stop_other_trigger x={self.stop_other_trigger_min_x + self.stop_other_trigger_midpoint_distance_x}, y={self.stop_other_trigger_min_y + self.stop_other_trigger_midpoint_distance_y}", color=cyan, life_time=50000)  # due to some reason life_time=0 doesn't work for draw string


            # ****************** draw bounding box of end_condition ego trigger location (parallel node with sync arrival) ***************
            self.debug.draw_box(carla.BoundingBox(carla.Location(self.end_condition_min_x + self.end_condition_midpoint_distance_x, self.end_condition_min_y + self.end_condition_midpoint_distance_y, 0), carla.Vector3D(self.end_condition_midpoint_distance_x, self.end_condition_midpoint_distance_y, 2)), carla.Rotation(), life_time=0)

            #self.debug.draw_string(carla.Location(-80, -163, 0), "end_condition ego trigger", color=carla.Color(255,255,255,0), life_time=50000)  # due to some reason life_time=0 doesn't work for draw string
            self.debug.draw_string(carla.Location(self.end_condition_min_x + self.end_condition_midpoint_distance_x, self.end_condition_min_y + self.end_condition_midpoint_distance_y, 0), f"end_condition ego trigger x={self.end_condition_min_x + self.end_condition_midpoint_distance_x}, y={self.end_condition_min_y + self.end_condition_midpoint_distance_y}", color=cyan, life_time=50000)  # due to some reason life_time=0 doesn't work for draw string

            # print(config.other_actors[0].transform.location.z)  # 0.5
            # print(config.other_actors[0].transform.location.z - 500)  #  -499.5 tf
            # sys.exit("yello")

        # ********************************* Draw intersection waypoints and corners ************************************

        #junction_waypoint_loc = carla.Location(-74.63, -136.34, 0)  # which is actually the sync arrival location

        #sys.exit("Sys.exit. IntersectionSituations executed successfully")

        try:  # try because it can return none
            current_w = self.map.get_waypoint(self.junction_waypoint_loc)  # get_waypoint(self, location, project_to_road=True, lane_type=carla.LaneType.Driving)
            if current_w.is_junction:  # yes it is a junction!!!!!!
                junction = current_w.get_junction()
                #draw_junction(self.debug, junction, l_time=50_000, self.wp_str_flag)  # SyntaxError: positional argument follows keyword argument
                if self.show_int_wps:
                    draw_junction(self.debug, junction, l_time=50_000, wp_str_flag=self.wp_str_flag)  # Corrected -> SyntaxError: positional argument follows keyword argument
            #   print("Yeeeeeah it is a junction")
            else:
                print("ain't junction")
        except Exception as e:
            print(str(e))


        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)  # Return: carla.Actor
        first_vehicle.set_simulate_physics(enabled=False)  # Enables or disables the simulation of physics on this actor.
        self.other_actors.append(first_vehicle)

        # Set up automatic control for other vehicle here 

        #self.other_veh_agentZ = OtherVehControlAgent()
        self.other_visualize = False


        # load the scenario settings in the ego driving agent 
        #goal = carla.Location(x=-148.97, y=-76.77, z=0.0)  #ego lef leg loc
        
        self.other_veh_agentZ.game_loop_init(self.other_vehicle_goal_carla_Location, self.other_visualize)
        #self.other_veh_agentZ.game_loop_init(goal, self.other_visualize)

        #while True:
            


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
            self.start_other_trigger_min_x,  self.start_other_trigger_max_x,  # min max x, ego vehicle moving along y axis, as x axis values are the same as inital condition spawn locs for ego.
             self.start_other_trigger_min_y,  self.start_other_trigger_max_y)  # min max y

        sync_arrival = SyncArrival(  ######## Parallel-2 ####################  # this trigger and the pass_through below running in parallel
            self.other_actors[0], self.ego_vehicles[0],  # other and ego passed
            carla.Location(x=self.sync_arrival_x, y=self.sync_arrival_y))  # where the actors should meet tf

        pass_through_trigger = InTriggerRegion( ######## Parallel-2 #################### # Trigger box  # Lets draw a box here in the map. ! 
            self.ego_vehicles[0],
            self.pass_through_trigger_min_x, self.pass_through_trigger_max_x,  # this trigger box is for ego vehicle and it has same values (naah xD) of x axis as for the initial spawn x axis loc of ego, only y axis values were changed.
            self.pass_through_trigger_min_y, self.pass_through_trigger_max_y)

        keep_velocity_other = KeepVelocity( ######## Parallel-1 ####################
            self.other_actors[0],
            self._other_actor_target_velocity, 
            self.other_veh_agentZ, self.other_vehicle_goal_carla_Location)  # 15 ms-1 defined above :3  # Object inserted here

        
        stop_other_trigger = InTriggerRegion( ######## Parallel-1 ####################
            self.other_actors[0],  # this trigger box is for other vehicles and it's loc suggests other vehicle is moving along x axis cx y-axis trigger box values are in the same range as it's initial spawning location
            self.stop_other_trigger_min_x, self.stop_other_trigger_max_x,  # apply the damn brakes in this region as seen in the parallel pytree node below xD
            self.stop_other_trigger_min_y, self.stop_other_trigger_max_y)  # <other_actor x="-105" y="-136", means other vehicle is moving along x axis cx y-axis trigger box values are in the same range as it's initial spawning location

        stop_other = StopVehicle(
            self.other_actors[0],  # other vehicle
            self._other_actor_max_brake)  # 1.0

        end_condition = InTriggerRegion(  ######## Parallel-1 #################### plus also at end again. So if it is parallel to keep velocity I can use it for end_loop of other vehicle as well
            self.ego_vehicles[0],  # for ego vehicle
            self.end_condition_min_x, self.end_condition_max_x,  # same x axis range as it is travelling across y axis
            self.end_condition_min_y, self.end_condition_max_y
        )

        ######## Parallel-1 ####################

        #ZZZZZZZZSo we got a bunch of trigger boxes and behaviour functions set up.

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()  # see def of sequence below
        scenario_sequence = py_trees.composites.Sequence()  # adding this to root below # so we have our behaviour seq pytree here 
        sync_arrival_parallel = py_trees.composites.Parallel(  #  see def of Parallel below
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree

        #A sequence will progressively tick over each of its children so long as (PROGRESSIVELY TICK OVER EACH OF ITS CHILDREN)
        #each child returns :data:`~py_trees.common.Status.FAILURE` or :data:`~py_trees.common.Status.RUNNING` the sequence will halt and the parent will adopt
        #the result of this child. If it reaches the last child, it returns with
        #that result regardless.
        root.add_child(scenario_sequence)  # PROGRESSIVELY ticks as stated above meaning addition sequence of these children is the main thing for a scenario
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))  # here the z axis value is 0.5 now, previously spawned at z = -500 by carlaDataProvider   
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(sync_arrival_parallel)  # parallel node
        scenario_sequence.add_child(keep_velocity_other_parallel)  # parallel node
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        #scenario_sequence.add_child(ActorDestroy(self.other_actors[0], self.other_veh_agentZ)) # Object inserted here
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0])) # Object inserted here

        #parallel nodes. Ticks every child every time the parallel is run (a poor man's form of parallelism). Parallels enable a kind of concurrency
        sync_arrival_parallel.add_child(sync_arrival)  # these two (sync_arrival, pass_through_trigger) added in one parallel node 
        sync_arrival_parallel.add_child(pass_through_trigger)  #     The condition terminates with SUCCESS, when the actor reached the target region
        # so when pass_through_trigger returns SUCCESS, the tree gets out of both these parallel nodes (sync arrival and pass through) and moves on to next, else the other vehicle keeps synching with other vehicle


        keep_velocity_other_parallel.add_child(keep_velocity_other)  # does ticking every child mean mean every method of class is run? I don't think so. Let's see.
        keep_velocity_other_parallel.add_child(stop_other_trigger)  # these two (keep_velocity_other, stop_other_trigger) added in one parallel node
        keep_velocity_other_parallel.add_child(end_condition) # adding this 18March21 (notes regarding this in OneNote)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        #collison_criteria = CollisionTest(self.ego_vehicles[0])
        collison_criteria = CollisionTest(self.ego_vehicles[0], self.other_actors[0],  terminate_on_failure=True)  # Lets see what happens on terminate on failure
        #collison_criteria = CollisionTest(self.ego_vehicles[0], self.other_actors[0]) 
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

    def _setup_scenario_end(self, config):  # Taken from Basic Scenario 
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

