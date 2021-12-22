#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

try: 
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_q

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

class KeyboardControl(object):
    def __init__(self, world):
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        #return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
        return (key == K_ESCAPE)# or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
        # ADDED

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import collections
#import datetime
import glob
import math
import random
import re
import sys
import weakref


import carla

from examples.manual_control import (World, HUD, CameraManager, CollisionSensor, LaneInvasionSensor, GnssSensor, IMUSensor)  
# keyboardcontrol class removed
                
from carla import ColorConverter as cc


# sys append path if compiler doesn't find these imports! (already included in env variables)
from agents.navigation.behavior_agent_z4 import BehaviorAgentZ4  # pylint: disable=import-error
from agents.navigation.roaming_agent_z import RoamingAgentZ  # pylint: disable=import-error
from agents.navigation.basic_agent_z import BasicAgentZ  # pylint: disable=import-error

import os
import argparse
import logging
import time
import pygame
import traceback



# *********** START from Srunner Manual_control.py ********************************* 

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class WorldSR(World):  # World class inherited here, so the self.args to WORLDSR object are used by the init method of parent class World cx there is no init method here I presume. Yeah I was right Alhmd.

    restarted = False

    def restart(self):  # overwritten parent restart() method

        if self.restarted:
            return
        self.restarted = True

        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713

        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get the ego vehicle
        while self.player is None:
            print("Waiting for the ego vehicle...")  # yep this is entered.
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')  # all of em.
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == "scenario":  # set_attribute to scenario in carla_data_provider.py, so that role_name attribute has been changed on the server side even! FOR THE OTHER VEHICLE :3
                    print("Ego vehicle found")
                    self.player = vehicle
                    break
        
        self.player_name = self.player.type_id  # The identifier of the blueprint this actor was based on, e.g. vehicle.ford.mustang.

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)  # 'Gamma correction of the camera (default: 2.2)'
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def tick(self, clock):
        if len(self.world.get_actors().filter(self.player_name)) < 1:
            return False

        self.hud.tick(self, clock)
        return True

# *********** END from Srunner Manual_control.py ********************************* 

# Ego Control Agent class

class OtherVehControlAgent:

    agent = None  # will assign the control agent to this
    visualize = False

    def __init__ (self): 

        self.args = ArgsOverwrite()  # parse the arguments that were previously in the main method using overwrite class
        pygame.init()
        pygame.font.init()
        self.world = None  
        

  
    def game_loop_init(self, goal_carla_location, visualize=False):  # Ideally I would want the carla client and carla world to be passed through the arguments here

        self.visualize = visualize

        print("goal location of OTHER vehicle is", goal_carla_location)

        if self.visualize == True:

            try:
                client = carla.Client(self.args.host, self.args.port)  
                client.set_timeout(4.0)

                self.display = pygame.display.set_mode(
                    (self.args.width, self.args.height),
                    pygame.HWSURFACE | pygame.DOUBLEBUF)

                hud = HUD(self.args.width, self.args.height)  # HUD object created here
                self.world = WorldSR(client.get_world(), hud, self.args)  # world is freaking set in this with the pygame display camz and sensors etc

                self.controller = KeyboardControl(self.world)  # using KeyboardControl in this file

                #sys.exit("reached here")

                if self.args.agent == "Behavior":
                    self.agent = BehaviorAgentZ4(self.world.player, behavior=self.args.behavior)  # world.player is the ego vehicle on the server side who's rolename attribute we overwrote as 'hero' and the WorldSR class grabs that vehicle as world.player

                    # end_condition ego trigger location
                    destination = goal_carla_location  # carla.Location(-80, -163, 0)

                    #self.agent.set_destination(self.agent.vehicle.get_location(), destination, clean=True)

                
            except Exception as e:
                traceback.print_exc()
                print("Could not setup EgoControlAgent due to {}".format(e))

        elif self.visualize == False:  # we won't run the world.render commands in the game_loop_step method to decrease the computation load. Will edit the World class to not initialize the camera manager or anything later for further optimization


            try:
                client = carla.Client(self.args.host, self.args.port)
                client.set_timeout(4.0)

                # No display as there is no visualization

                # display = pygame.display.set_mode(
                #     (self.args.width, self.args.height),
                #     pygame.HWSURFACE | pygame.DOUBLEBUF)

                hud = HUD(self.args.width, self.args.height)  # HUD object created here
                self.world = WorldSR(client.get_world(), hud, self.args)  # world is freaking set in this with the pygame display camz and sensors etc

                self.controller = KeyboardControl(self.world)  # using KeyboardControl in this file


                #sys.exit("reached here")

                if self.args.agent == "Behavior":
                    self.agent = BehaviorAgentZ4(self.world.player, behavior=self.args.behavior)  # world.player is the ego vehicle on the server side who's rolename attribute we overwrote as 'hero' and the WorldSR class grabs that vehicle as world.player

                    # end_condition ego trigger location
                    destination = goal_carla_location  # carla.Location(-80, -163, 0)

                    #self.goal = destination

                    #self.agent.set_destination(self.agent.vehicle.get_location(), destination, clean=True)

                   
            except Exception as e:
                traceback.print_exc()
                print("Could not setup EgoControlAgent due to {}".format(e))

        else:

            #sys.exit("reached here")
            raise ValueError("Select proper visualization option")

    def game_loop_step(self, target_velocity):  # will run just once so no while loop

        #sys.exit("reached here")

        try:

            while True:

                clock = pygame.time.Clock()  # same as the manual control . py    
                clock.tick_busy_loop(60)  # Check out behavior after commenting this out



                if self.controller.parse_events():  # if quit key is pressed 
                  return  

                # As soon as the server is ready continue!
                if not self.world.world.wait_for_tick(10.0):
                    continue

                if not self.world.tick(clock):  # doesn't enter this if our ego has been spawned  # if len(self.world.get_actors().filter(self.player_name)) < 1:
                    return

                if self.args.agent == "Behavior":
                    self.agent.update_information(self.world, target_velocity)  # get target speed limit from sign posts, traffic light info and incoming waypoint info. Overwrote the speed restrictions mostly
                    #sys.exit("here")  # yes enters this

                    if self.visualize:
                        self.world.render(self.display)
                        pygame.display.flip()


                    if len(self.agent.get_local_planner().waypoints_queue) == 0:
                        print("Target reached, mission accomplished...")
                        #break

                    #speed_limit = world.player.get_speed_limit()  # see def below, though we have max speed variable for our behaviour agent behaviour object as well           
                    # overwrite this with hundred and also in behaviorZ script :3 (don't go deeper into addition with that like local planner or else make a new local plannerZ etc)
                    #speed_limit = 100  # Added

                    # modify this from atomic behavior :3 
                    speed_limit = target_velocity  # Added 

                    '''
                    get_speed_limit(self)
                    The client returns the speed limit affecting this vehicle according to last tick (it does not call the simulator). The speed limit is updated when passing by a speed limit signal, so a vehicle might have none right after spawning.
                    Return: float – m/s
                    '''

                    self.agent.get_local_planner().set_speed(speed_limit)  # Request new target speed. According to the speed limit signal max speed.
                    # Maybe change this speed limit method as well to terminate these kinds of limits

                    control = self.agent.run_step()
                    self.world.player.apply_control(control)

                    break

        except Exception as e: 
            traceback.print_exc()
            print(e)

    def game_loop_end(self):

        try:
            self.world.destroy()  # World object returned by WorldSR class
            print("***************########## destroying other vehicle and it's sensors #######*****************")
            pygame.quit()

        except Exception as e: 
            traceback.print_exc()
            print(e)

    
class ArgsOverwrite:

    rolename = 'hero'
    gamma = 2.2 
    width = 1280
    height = 720
    host = '127.0.0.1'
    port = 2000
    behavior = "cautious"
    agent = "Behavior"
    seed = None
    verbose = False
    autopilot = False

    def __init__ (self):
    #filter turns blue if I write it as it is without self
        self.filter = "vehicle.*"   # Needed for CARLA version
