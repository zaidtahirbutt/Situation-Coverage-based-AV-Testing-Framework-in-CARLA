#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import sys
import time

import py_trees

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer2 import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from automatic_control_agent_z8_ego import *  # EgoControlAgent() imported from here
#from automatic_control_agent_z5_other_veh import *  # OtherVehControlAgent() imported from here



class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze_scenario()
    5. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, debug_mode=False, sync_mode=False, timeout=2.0):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.scenario_class = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = timeout
        self._watchdog = Watchdog(float(self._timeout))

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.collision_counts = None
        self.collision_test_result = None

    def _reset(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def cleanup(self):
        """
        This function triggers a proper termination of a scenario
        """

        if self.scenario is not None:
            self.scenario.terminate()

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, ego_goal_location, other_veh_agentZ_og, ego_visualize=False, agent=None):
        """
        Load a new scenario
        """
        self._reset()

        # Initializing our custom ego
        self.ego_agentZ = EgoControlAgent()

        # load the scenario settings in the ego driving agent 
        self.ego_agentZ.game_loop_init(ego_goal_location, ego_visualize)
        #sys.exit("reached here load_scenario")

        # default code of agent not used
        self._agent = AgentWrapper(agent) if agent else None  # No agent right now  # one agent returned
        if self._agent is not None:
            self._sync_mode = True
 
        ################################ Initialize Other Vehicle Agent here!!!!!!!!!!!!! ##############################
        self.other_veh_agentZ_og = other_veh_agentZ_og


        self.scenario_class = scenario  # NoSignalJunctionCrossing Class
        self.scenario = scenario.scenario  # Scenario(behavior_seq, criteria, self.name, self.timeout, self.terminate_on_failure) class in Basic Scenario Class

        '''
        Scenario Class
        def __init__(self, behavior, criteria, name, timeout=60, terminate_on_failure=False):
            self.behavior = behavior
            self.test_criteria = criteria
            self.timeout = timeout
            self.name = name  # name = "NoSignalJunctionCrossing"
        '''
        self.scenario_tree = self.scenario.scenario_tree  # from scenario class in basic_scenario py file --> Tha main scenario tree that has everything :3 self.scenario_tree = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)   # name = "NoSignalJunctionCrossing"
        
        # Create overall py_tree --> from scenario class in basic_scenario py file
        #self.scenario_tree = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        


        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors

        # To print the scenario tree uncomment the next line
        #py_trees.display.render_dot_tree(self.scenario_tree)

        #sys.exit("OFF")

        if self._agent is not None:
            self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog.start()
        self._running = True

        while self._running:  # is equal to false when the scenario tree is finished running as seen below in self._tick_scenario(timestamp)
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()  # snapshot of what's happening in the world?
                '''
                This snapshot comprises all the information for every actor on scene at a certain moment of time. It creates and gives acces to a data structure containing a series of carla.ActorSnapshot. The client recieves a new snapshot on every tick that cannot be stored.

                '''


                if snapshot:
                    timestamp = snapshot.timestamp
                    #print(timestamp)
            if timestamp:  # meaning carla worl is running i.e., the scenario in that world

                # So the scenario is ticked in a while loop which is pretty fast :3 
                # So do I need to declare the veh driving agent here or independently? Let's see.
                # Maybe I should call the ego and other veh agents here :3 
                # Run the run_step method here of the automatic_control_agent_z here :3

                self.ego_agentZ.game_loop_step()  # making new plan again for each step, just an experiment.
                
                self._tick_scenario(timestamp)  # Run next tick of scenario and the agent.

        self.ego_agentZ.game_loop_end()
        self.other_veh_agentZ_og.game_loop_end()  # ENDING IT HERE AL!

        self._watchdog.stop()

        self.cleanup()

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - \
            self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()

            if self._debug_mode:
                print("\n--------- Tick ---------\n")  # cool
                #sys.exit("fhawk")
            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()  # update all actors velocity, location, transform

            if self._agent is not None:
                ego_action = self._agent()  # doesn't enter this condition for NoSignalJunctionCrossing so let's ignore it for now
            if self._agent is not None:
                self.ego_vehicles[0].apply_control(ego_action)  # will see what is this ego_action

            # Tick scenario
            self.scenario_tree.tick_once()  # look at this, moving through the sequences in a tree I suppose!!!!!~~~!! vip!
            #sys.exit("fhawk")

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False  # meaning scenario finished  # When last child/node of the pytree i.e., the root = pytree sequence() is executed and in a non running state

        if self._sync_mode and self._running and self._watchdog.get_status():
            CarlaDataProvider.get_world().tick()

    def get_running_status(self):
        """
        returns:
           bool:  False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function is used by the overall signal handler to terminate the scenario execution
        """
        self._running = False

    def analyze_scenario(self, stdout, filename, junit):
        """
        This function is intended to be called from outside and provide
        the final statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        if self.scenario.test_criteria is None:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in self.scenario.get_criteria():
            if (not criterion.optional and
                    criterion.test_status != "SUCCESS" and
                    criterion.test_status != "ACCEPTABLE"):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit)  # self is the scenario manager class object
        output.write()

        self.collision_counts = output.collision_counts
        self.collision_test_result = output.collision_test_result
        


        return failure or timeout
