# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles.
The agent also responds to traffic lights. """


import carla
from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
import sys

class BasicAgent(Agent):
    """
    BasicAgent implements a basic agent that navigates scenes to reach a given
    target destination. This agent respects traffic lights and other vehicles.
    """

    def __init__(self, vehicle, target_speed=20):  

    # from automatic_control.py agent = BasicAgent(world.player)
    # also, blueprint.set_attribute('role_name', 'hero') in that script.
    

        """

        :param vehicle: actor to apply to local planner logic onto
        """
        super(BasicAgent, self).__init__(vehicle)

        self._proximity_tlight_threshold = 5.0  # meters
        self._proximity_vehicle_threshold = 10.0  # meters
        self._state = AgentState.NAVIGATING
        args_lateral_dict = {
            'K_P': 1,
            'K_D': 0.4,
            'K_I': 0,
            'dt': 1.0/20.0}
        self._local_planner = LocalPlanner(
            self._vehicle, opt_dict={'target_speed' : target_speed,
            'lateral_control_dict':args_lateral_dict})
        self._hop_resolution = 2.0
        self._path_seperation_hop = 2
        self._path_seperation_threshold = 0.5
        self._target_speed = target_speed
        self._grp = None

    def set_destination(self, location):

        # agent.set_destination((spawn_point.location.x,
        #                           spawn_point.location.y,
        #                           spawn_point.location.z))

        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router
        """

        start_waypoint = self._map.get_waypoint(self._vehicle.get_location())  # random gen of vehicle Z
        end_waypoint = self._map.get_waypoint(                                  
            carla.Location(location[0], location[1], location[2]))
        
        #print(f"{start_waypoint} is start waypoint")
        #print(f"{end_waypoint} is end waypoint")

        # Waypoint(Transform(Location(x=13.992948, y=134.392670, z=0.000000), Rotation(pitch=360.000000, yaw=-0.817200, roll=0.000000))) is start waypoint
        # Waypoint(Transform(Location(x=-5.532081, y=-79.032463, z=0.000000), Rotation(pitch=360.000000, yaw=91.413544, roll=0.000000))) is end waypoint

        route_trace = self._trace_route(start_waypoint, end_waypoint)  # so we have start and end way point
        # now we gonna trace a whole route from just these start and end waypoints using GlobalRoutePlannerDAO
        # and GlobalRoutePlanner classes
        # and we recieve an optimal route using A* on the graph that we make there and respecting the route
        # resolution that is 2.0 in this case i.e., the distance between each successive waypoint.
        # route_trace = route, let's print it :3

        #print(route_trace[0])  #(<carla.libcarla.Waypoint object at 0x000002A2D71C4C90>, <RoadOption.LANEFOLLOW: 4>)
        #sys.exit("off")

        #print(route_trace[0][0])  # Waypoint(Transform(Location(x=1.507047, y=55.319771, z=0.000000), Rotation(pitch=360.000000, yaw=269.637451, roll=0.000000)))
        #sys.exit("off")  # so the Wapoint object was of course as I thought, accessible mA

        # So we have the whole route with the RoadOptions in it

        self._local_planner.set_global_plan(route_trace)

        """
        Resets the waypoint queue and buffer to match the new plan. Also
        sets the global_plan flag to avoid creating more waypoints

        :param current_plan: list of (carla.Waypoint, RoadOption)
        :return:
        """

        """
        LocalPlanner implements the basic behavior of following a trajectory of waypoints that is generated on-the-fly.
        The low-level motion of the vehicle is computed by using two PID controllers, one is used for the lateral control
        and the other for the longitudinal control (cruise speed).

        When multiple paths are available (intersections) this local planner makes a random choice. THIS RANDOM CHOICE PART is imp
        """

    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the optimal route
        from start_waypoint to end_waypoint
        """

        # Setting up global router
        if self._grp is None:
            dao = GlobalRoutePlannerDAO(self._vehicle.get_world().get_map(), self._hop_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp

        # Obtain route plan
        route = self._grp.trace_route(
            start_waypoint.transform.location,
            end_waypoint.transform.location)

        return route

    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """

        # is there an obstacle in front of us?
        hazard_detected = False

        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")

        # check possible obstacles
        vehicle_state, vehicle = self._is_vehicle_hazard(vehicle_list)
        if vehicle_state:
            if debug:
                print('!!! VEHICLE BLOCKING AHEAD [{}])'.format(vehicle.id))

            self._state = AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True

        # check for the state of the traffic lights
        light_state, traffic_light = self._is_light_red(lights_list)
        if light_state:
            if debug:
                print('=== RED LIGHT AHEAD [{}])'.format(traffic_light.id))

            self._state = AgentState.BLOCKED_RED_LIGHT
            hazard_detected = True

        if hazard_detected:
            control = self.emergency_stop()  # from dady class Agent YYAMERRROOOOOO!
        else:
            self._state = AgentState.NAVIGATING
            # standard local planner behavior
            control = self._local_planner.run_step(debug=debug)  # returned carla control

            '''_local_planner.run_step
            Execute one step of local planning which involves
            running the longitudinal and lateral PID controllers to
            follow the waypoints trajectory.

                :param target_speed: desired speed
                :param debug: boolean flag to activate waypoints debugging
                :return: control
            '''

        return control

    def done(self):
        """
        Check whether the agent has reached its destination.
        :return bool
        """
        return self._local_planner.done()
