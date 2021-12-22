#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function


# ==============================================================================
## Google API Import
# ==============================================================================
import os
import pathlib


if "models" in pathlib.Path.cwd().parts:
  while "models" in pathlib.Path.cwd().parts:
    os.chdir('..')
elif not pathlib.Path('models').exists():
  print("git clone the model")
  #!git clone --depth 1 https://github.com/tensorflow/models


#import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
#from IPython.display import display
#from grabscreen import grab_screen
#import cv2

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util



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
import datetime
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
from agents.navigation.behavior_agent_z2 import BehaviorAgentZ2  # pylint: disable=import-error
from agents.navigation.roaming_agent_z import RoamingAgentZ  # pylint: disable=import-error
from agents.navigation.basic_agent_z import BasicAgentZ  # pylint: disable=import-error

import os
import argparse
import logging
import time
import pygame
import traceback
import numpy as np
import cv2



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

        # Image width and height
        self.im_width = 800
        self.im_height = 450
        self.SHOW_CAM = False #True

        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get the ego vehicle
        while self.player is None:
            print("Waiting for the ego vehicle...")  # yep this is entered.
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')  # all of em.
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == "hero":  # set_attribute to hero in carla_data_provider.py, so that role_name attribute has been changed on the server side even!
                    print("Ego vehicle found")
                    self.player = vehicle  # here we got our ego. Spawn RGB sensor/camera here as well
                    break
        
        self.player_name = self.player.type_id  # The identifier of the blueprint this actor was based on, e.g. vehicle.ford.mustang.

        # one camera sensor here
        self.blueprint_library = self.world.get_blueprint_library()
        self.rgb_cam = self.blueprint_library.find("sensor.camera.rgb")
        self.rgb_cam.set_attribute("image_size_x", f"{self.im_width}")
        self.rgb_cam.set_attribute("image_size_y", f"{self.im_height}")
        self.rgb_cam.set_attribute("fov", f"100")
        #self.rgb_cam.set_attribute("fov", f"130")
        #self.rgb_cam.set_attribute("fov", f"90")

        #transform = carla.Transform(carla.Location(x=2.5, z=0.7 ))
        #transform = carla.Transform(carla.Location(x=2.5, z=1.5 ))  # This seemed good
        transform = carla.Transform(carla.Location(x=2.5, z=1.0 ))
        self.sensor_cam = self.world.spawn_actor(self.rgb_cam, transform, attach_to=self.player)
        self.sensor_cam.listen(lambda data: self.process_img(data))


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

    def process_img(self, image):
        i = np.array(image.raw_data)
        #print(dir(image))
        #print(i.shape)
        i2 = i.reshape(self.im_height, self.im_width, 4)  # (1440000,)
        i3 = i2[:, :, :3]
        if self.SHOW_CAM:
            cv2.imshow("", i3)
            cv2.waitKey(1)
        self.front_camera = i3  #800x450 image

    def tick(self, clock):
        if len(self.world.get_actors().filter(self.player_name)) < 1:
            return False

        self.hud.tick(self, clock)
        return True

# *********** END from Srunner Manual_control.py ********************************* 

# Ego Control Agent class

class EgoControlAgent:

    agent = None  # will assign the control agent to this
    visualize = False
    collision_warning = False

    def __init__ (self): 

        self.args = ArgsOverwrite()  # parse the arguments that were previously in the main method using overwrite class
        pygame.init()
        pygame.font.init()
        self.world = None  
        

  
    def game_loop_init(self, goal_carla_location, visualize=False):  # Ideally I would want the carla client and carla world to be passed through the arguments here

        self.visualize = visualize

        # Google object detection API initializations
        physical_devices = tf.config.experimental.list_physical_devices('GPU')
        assert len(physical_devices) > 0, "Not enough GPU hardware devices available"
        tf.config.experimental.set_memory_growth(physical_devices[0], True)

        PATH_TO_LABELS = 'models/research/object_detection/data/mscoco_label_map.pbtxt'
        self.category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

        model_name = 'ssd_mobilenet_v1_coco_2017_11_17'
        self.model = self.load_model3(model_name)  # This can be a hazard for CPU usage going to 100%


        print("goal location of EGO vehilce is", goal_carla_location)

        self.goal = goal_carla_location

        try:
            client = carla.Client(self.args.host, self.args.port)
            client.set_timeout(4.0)

            # No display as there is no visualization
            if self.visualize == True:
                display = pygame.display.set_mode(
                    (self.args.width, self.args.height),
                    pygame.HWSURFACE | pygame.DOUBLEBUF)

            hud = HUD(self.args.width, self.args.height)  # HUD object created here
            # World and ego vehicle filtered here
            self.world = WorldSR(client.get_world(), hud, self.args)  # world is freaking set in this with the pygame display camz and sensors etc

            self.controller = KeyboardControl(self.world)  # using KeyboardControl in this file


            #sys.exit("reached here")

            if self.args.agent == "Behavior":
                self.agent = BehaviorAgentZ2(self.world.player, behavior=self.args.behavior)  # world.player is the ego vehicle on the server side who's rolename attribute we overwrote as 'hero' and the WorldSR class grabs that vehicle as world.player

                # end_condition ego trigger location
                destination = goal_carla_location  # carla.Location(-80, -163, 0)

                self.agent.set_destination(self.agent.vehicle.get_location(), destination, clean=True)

                   
        except Exception as e:
            traceback.print_exc()
            print("Could not setup EgoControlAgent due to {}".format(e))

    def game_loop_step(self):  # will run just once so no while loop

        #sys.exit("reached here")

        try:
            while True:
                clock = pygame.time.Clock()  # same as the manual control . py    
                clock.tick_busy_loop(60)  # Check out behavior after commenting this out

                #self.agent.set_destination(self.agent.vehicle.get_location(), self.goal, clean=True)

                if self.controller.parse_events():  # if quit key is pressed 
                  return  

                # As soon as the server is ready continue!
                if not self.world.world.wait_for_tick(10.0):
                    continue

                if not self.world.tick(clock):  # doesn't enter this if our ego has been spawned  # if len(self.world.get_actors().filter(self.player_name)) < 1:
                    return

                if self.args.agent == "Behavior":
                    self.agent.update_information(self.world)  # get target speed limit from sign posts, traffic light info and incoming waypoint info. Overwrote the speed restrictions mostly
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
                    speed_limit = 40  # Added 

                    '''
                    get_speed_limit(self)
                    The client returns the speed limit affecting this vehicle according to last tick (it does not call the simulator). The speed limit is updated when passing by a speed limit signal, so a vehicle might have none right after spawning.
                    Return: float – m/s
                    '''

                    self.agent.get_local_planner().set_speed(speed_limit)  # Request new target speed. According to the speed limit signal max speed.
                    # Maybe change this speed limit method as well to terminate these kinds of limits

                    # Doing object detection here
                    
                    #output_dict = self.run_inference_for_single_image(self.model, self.world.front_camera)
                    image_np = cv2.resize(self.world.front_camera, (800,450))
                    output_dict = self.run_inference_for_single_image(self.model, image_np)
                    #image_np = self.world.front_camera
                    #image_np = cv2.cvtColor(self.world.front_camera, cv2.COLOR_BGR2RGB)


                    # Actual detection.
                    #image = self.world.front_camera


                    # Visualization of the results of a detection.
                    
                    vis_util.visualize_boxes_and_labels_on_image_array(
                      image_np,
                      output_dict['detection_boxes'],
                      output_dict['detection_classes'],
                      output_dict['detection_scores'],
                      self.category_index,
                      instance_masks=output_dict.get('detection_masks_reframed', None),
                      use_normalized_coordinates=True,
                      line_thickness=8)
                        
                    boxes = output_dict['detection_boxes']
                    classes = output_dict['detection_classes']
                    scores = output_dict['detection_scores']

                    for i,b in enumerate(boxes):
                            #   car                    bus                  truck
                        if classes[i] == 3: #or classes[i] == 6 or classes[i] == 8:
                            # Default
                            if scores[i] >= 0.5:  # I can change this as well ;3
                            
                            # Fault 1
                            #if scores[i] >= 0.95:  # I can change this as well ;3
                            #if scores[i] >= 0.8:  # I can change this as well ;3
                            #if scores[i] >= 0.6:  # I can change this as well ;3
                                mid_x = (boxes[i][1]+boxes[i][3])/2
                                mid_y = (boxes[i][0]+boxes[i][2])/2
                                #apx_distance = round(((1 - (boxes[i][3] - boxes[i][1]))**4),1)
                                apx_distance = round(((1 - (boxes[i][3] - boxes[i][1]))**4),1)

                                #cv2.putText(image_np, '{}'.format(apx_distance), (int(mid_x*image_np.shape[1]),int(mid_y*image_np.shape[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                                #cv2.putText(image_np, '{}'.format(apx_distance), (int(mid_x*800),int(mid_y*450)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                                #cv2.putText(image_np, '{}'.format(scores[i]), (int(mid_x*800),int(mid_y*450)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                                #wewe
                                
                                # Fault 3
                                #if apx_distance <=0.35:  # This seems to have filtered out the miss calls
                                #if apx_distance <=0.5:  # Almost 1 meter
                                
                                # Default
                                if apx_distance <=0.6: # Almost 2.5 meters  
                                

                                #if apx_distance <=0.75:
                                                                    
                                    # cv2.putText(image_np, 'COLLISION-HAZARD!!!', (int(mid_x*800) - 50, int(mid_y*450) - 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
                                    # self.collision_warning = True

                                    #print("Hazard", apx_distance)
                                    #if mid_x > 0.3 and mid_x < 0.7:  # if it is in the middle!
                                    #if mid_x > 0.0 and mid_x < 1.0:  # all


                                    #if mid_x > 0.2 and mid_x < 0.8:  # not so middle
                                    #if mid_x > 0.1 and mid_x < 0.9:  # not so middle
                                    
                                    # Default
                                    if mid_x > 0.05 and mid_x < 0.98:  # not so middle
                                    
                                    # Fault 2
                                    #if mid_x > 0.3 and mid_x < 0.6:  # not so middle
                                    
                                        cv2.putText(image_np, 'COLLISION-HAZARD!!!', (int(mid_x*800) - 50, int(mid_y*450) - 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
                                        self.collision_warning = True
                                    


                    #cv2.imshow('window',cv2.resize(image_np,(800,450)))
                    #if cv2.waitKey(25) & 0xFF == ord('q'):
                      #cv2.destroyAllWindows()
                      #break
                        
                    #display(Image.fromarray(image_np))
                    #cv2.imshow('object Detected', image_np)
                    cv2.imshow('object detection', cv2.resize(image_np, (800,600)))
                    #cv2.imshow('object detection', cv2.resize(image_np, (400,300)))
                    cv2.waitKey(1)

                    if self.collision_warning:
                        control = self.emergency_stop()
                    else:
                        control = self.agent.run_step()
                    
                    self.world.player.apply_control(control)

                    break

        except Exception as e: 
            traceback.print_exc()
            print(e)

    @staticmethod
    def emergency_stop():
        """
        Send an emergency stop command to the vehicle

            :return: control for braking
        """
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False

        return control


    def game_loop_end(self):

        try:
            self.world.sensor_cam.destroy()  # Destroy the spawned RGB camera
            self.world.destroy()  # World object returned by WorldSR class
            pygame.quit()

        except Exception as e: 
            traceback.print_exc()
            print(e)

    def load_model3(self, model_name):
      base_url = 'http://download.tensorflow.org/models/object_detection/'
      model_file = model_name + '.tar.gz'
      dir_loc = 'D://scenario_runner-0.9.10//models//research//object_detection//models//'

      model_dir = tf.keras.utils.get_file(
        fname= dir_loc + model_name, 
        origin=base_url + model_file,
        untar=True,
        cache_dir=dir_loc)
        

       
      #print(model_dir)

      model_dir = pathlib.Path(model_dir)/"saved_model"
      #model_dir = str(pathlib.Path(model_dir))
     
      #model_dir = model_dir + "//" + "saved_model"
     
      #print(model_dir)

      model = tf.saved_model.load(str(model_dir))

      return model

    def run_inference_for_single_image(self, model, image):
      image = np.asarray(image)
      # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
      input_tensor = tf.convert_to_tensor(image)
      # The model expects a batch of images, so add an axis with `tf.newaxis`.
      input_tensor = input_tensor[tf.newaxis,...]

      # Run inference
      with tf.device('/cpu:0'):
          model_fn = model.signatures['serving_default']
          output_dict = model_fn(input_tensor)

      # All outputs are batches tensors.
      # Convert to numpy arrays, and take index [0] to remove the batch dimension.
      # We're only interested in the first num_detections.
      num_detections = int(output_dict.pop('num_detections'))
      output_dict = {key:value[0, :num_detections].numpy() 
                     for key,value in output_dict.items()}
      output_dict['num_detections'] = num_detections

      # detection_classes should be ints.
      output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
       
      # Handle models with masks:
      if 'detection_masks' in output_dict:
        # Reframe the the bbox mask to the image size.
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                  output_dict['detection_masks'], output_dict['detection_boxes'],
                   image.shape[0], image.shape[1])      
        detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                           tf.uint8)
        output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()
        
      return output_dict

    
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
