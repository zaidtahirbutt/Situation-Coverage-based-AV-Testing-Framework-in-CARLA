#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import random
import time

red = carla.Color(255, 0, 0)
green = carla.Color(0, 255, 0)
blue = carla.Color(47, 210, 231)
medium_blue = carla.Color(50, 255, 255)
cyan = carla.Color(0, 255, 255)
yellow = carla.Color(255, 255, 0)
orange = carla.Color(255, 162, 0)
white = carla.Color(255, 255, 255)

trail_life_time = 10
waypoint_separation = 4


def draw_transform(debug, trans, col=carla.Color(255, 0, 0), lt=-1):
    debug.draw_arrow(
        trans.location, trans.location + trans.get_forward_vector(),
        #thickness=0.05, arrow_size=0.1, color=col, life_time=lt)
        thickness=0.4, arrow_size=1.2, color=col, life_time=lt)

# draw_arrow(self, begin, end, thickness=0.1, arrow_size=0.1, color=(255,0,0), life_time=-1.0)

def draw_waypoint_union(debug, w0, w1, color=carla.Color(255, 0, 0), lt=5):
    debug.draw_line(
        w0.transform.location + carla.Location(z=0.25),
        w1.transform.location + carla.Location(z=0.25),
        thickness=0.1, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(w1.transform.location + carla.Location(z=0.25), 0.1, color, lt, False)


def draw_waypoint_info(debug, w, lt=5):
    w_loc = w.transform.location
    debug.draw_string(w_loc + carla.Location(z=0.5), "lane: " + str(w.lane_id), False, yellow, lt)
    debug.draw_string(w_loc + carla.Location(z=1.0), "road: " + str(w.road_id), False, blue, lt)
    debug.draw_string(w_loc + carla.Location(z=-.5), str(w.lane_change), False, red, lt)

def draw_junction(debug, junction, l_time=10, wp_str_flag=True):
    """Draws a junction bounding box and the initial and final waypoint of every lane."""
    # draw bounding box
    box = junction.bounding_box



    # four corners of the intersection
    point1 = box.location + carla.Location(x=box.extent.x, y=box.extent.y, z=2)
    point2 = box.location + carla.Location(x=-box.extent.x, y=box.extent.y, z=2)
    point3 = box.location + carla.Location(x=-box.extent.x, y=-box.extent.y, z=2)
    point4 = box.location + carla.Location(x=box.extent.x, y=-box.extent.y, z=2)
    debug.draw_line(
        point1, point2,
        thickness=0.1, color=orange, life_time=l_time, persistent_lines=False)
    debug.draw_line(
        point2, point3,
        thickness=0.1, color=orange, life_time=l_time, persistent_lines=False)
    debug.draw_line(
        point3, point4,
        thickness=0.1, color=orange, life_time=l_time, persistent_lines=False)
    debug.draw_line(
        point4, point1,
        thickness=0.1, color=orange, life_time=l_time, persistent_lines=False)
    # draw junction pairs (begin-end) of every lane
    #junction_w = junction.get_waypoints(carla.LaneType.Any)
    
    junction_w = junction.get_waypoints(carla.LaneType.Driving)

    for pair_w in junction_w:
        #draw_transform(debug, pair_w[0].transform, orange, l_time)

        draw_interpolate_waypoints(debug, pair_w, l_time, wp_str_flag)
        #break

        draw_transform(debug, pair_w[0].transform, green, l_time)

        #w_x, w_y, w_z = round(pair_w[0].transform.location.x, 2), round(pair_w[0].transform.location.y, 2), round(pair_w[0].transform.location.z, 2)
        w_x, w_y, w_z = pair_w[0].transform.location.x, pair_w[0].transform.location.y, pair_w[0].transform.location.z

        #debug.draw_string(pair_w[0].transform.location, f"x={w_x}, y={w_y}, z={w_z}", color=yellow, life_time=50_000)  # due to some reason life_time=0 doesn't work for draw string
        if wp_str_flag:
            debug.draw_string(pair_w[0].transform.location, f"x={round(w_x, 2)}, y={round(w_y, 2)}, z={round(w_z, 2)}", color=cyan, life_time=50_000)  # Will print over previously printed values in the display if repetition occurs
        #debug.draw_string(carla.Location(w_x, w_y, z=3), f"x={round(w_x, 2)}, y={round(w_y, 2)}, z={round(w_z, 2)}", color=cyan, life_time=50_000)  # Will print over previously printed values in the display if repetition occurs

        debug.draw_point(
            pair_w[0].transform.location + carla.Location(z=0.75), 0.1, orange, l_time, False)
       
        #draw_transform(debug, pair_w[1].transform, orange, l_time)
        draw_transform(debug, pair_w[1].transform, red, l_time)
        
        #w_x, w_y, w_z = round(pair_w[1].transform.location.x, 2), round(pair_w[1].transform.location.y, 2), round(pair_w[1].transform.location.z, 2)
        w_x, w_y, w_z = pair_w[1].transform.location.x, pair_w[1].transform.location.y, pair_w[1].transform.location.z
        if wp_str_flag:
            debug.draw_string(pair_w[1].transform.location, f"x={round(w_x, 2)}, y={round(w_y, 2)}, z={round(w_z, 2)}", color=cyan, life_time=50_000)  # Will print over previously printed values in the display if repetition occurs
        #debug.draw_string(carla.Location(w_x, w_y, z=3), f"x={round(w_x, 2)}, y={round(w_y, 2)}, z={round(w_z, 2)}", color=cyan, life_time=50_000)  # Will print over previously printed values in the display if repetition occurs

        debug.draw_point(
            pair_w[1].transform.location + carla.Location(z=0.75), 0.1, orange, l_time, False)
        
        debug.draw_line(
            pair_w[0].transform.location + carla.Location(z=0.75),
            pair_w[1].transform.location + carla.Location(z=0.75), 0.1, white, l_time, False)

def draw_interpolate_waypoints(debug, pair_w, l_time, wp_str_flag=True):
    
    #next_until_lane_end = pair_w[0].next_until_lane_end(10)  # don't think I need this
    #next_until_lane_end2 = pair_w[1].next_until_lane_end(10)  
    next_outward_wp = pair_w[1].next(10)  
    prev_outward_wp = pair_w[1]  # just making prev_outward_wp a container for this type of object
    #next_outward_wp2 = next_outward_wp[0].next(10)

    next_inward_wp = pair_w[0].previous(10)  #pair_w[0].next(10)
    prev_inward_wp = pair_w[0]  # just making prev_inward_wp a container for this type of object


    wp_counter = 0  #None  # initialize 
    wp_counter2 = 0  #None  # initialize 

    # list indices must be integers or slices, not NoneType

    for i in range(10):

        #print(f"I am in here at count i == {i}")

        for count, wp in enumerate(next_outward_wp):  # thoruhg all the wps in next list  # awesome result
            
            #if next_outward_wp[count].road_id == pair_w[1].road_id:  # if they are on the same roads and not chaning lanes and spreading out to other roads/lanes
            if (next_outward_wp[count].road_id == prev_outward_wp.road_id) or (i == 0):  # i = 0 condition to skip the next_outward_wp[count].road_id =! pair_w[1].road_id condition as the very first wp is on the intersection that we are starting at :3
            # after the brackets, the conditions worked
                #print(f"I am in here at count i == {i}")
                wp_counter = count  # for the first iteration we are relying on the fact that pair_w[1].next(10) will give us only one waypoint and count wont matter as it would be equal to count = 0
                
                w_x, w_y, w_z = wp.transform.location.x, wp.transform.location.y, wp.transform.location.z
                if wp_str_flag:
                    debug.draw_string(wp.transform.location, 
                        f"x={round(w_x, 2)}, y={round(w_y, 2)}, z={round(w_z, 2)}", color=cyan, life_time=50_000)
                draw_transform(debug, wp.transform, red, l_time)
            
        prev_outward_wp = next_outward_wp[wp_counter]        
        next_outward_wp = next_outward_wp[wp_counter].next(10)  # [wp_counter] is here so that we don't change roads and go too far out

    for i in range(10):

        #print(f"I am in here at count i == {i}")

        for count, wp in enumerate(next_inward_wp):  # thoruhg all the wps in next list  # awesome result
            
            #if next_outward_wp[count].road_id == pair_w[1].road_id:  # if they are on the same roads and not chaning lanes and spreading out to other roads/lanes
            if (next_inward_wp[count].road_id == prev_inward_wp.road_id) or (i == 0):  # i = 0 condition to skip the next_outward_wp[count].road_id =! pair_w[1].road_id condition :3
            # after the brackets, the conditions worked
                #print(f"I am in here at count i == {i}")
                wp_counter2 = count  # for the first iteration we are relying on the fact that pair_w[1].next(10) will give us only one waypoint and count wont matter as it would be equal to count = 0
                
                w_x, w_y, w_z = wp.transform.location.x, wp.transform.location.y, wp.transform.location.z
                if wp_str_flag:
                    debug.draw_string(wp.transform.location, 
                        f"x={round(w_x, 2)}, y={round(w_y, 2)}, z={round(w_z, 2)}", color=cyan, life_time=50_000)
                draw_transform(debug, wp.transform, green, l_time)
            
        prev_inward_wp = next_inward_wp[wp_counter2]        
        next_inward_wp = next_inward_wp[wp_counter2].previous(10)  # [wp_counter] is here so that we don't change roads and go too far out


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-i', '--info',
        action='store_true',
        help='Show text information')
    argparser.add_argument(
        '-x',
        default=0.0,
        type=float,
        help='X start position (default: 0.0)')
    argparser.add_argument(
        '-y',
        default=0.0,
        type=float,
        help='Y start position (default: 0.0)')
    argparser.add_argument(
        '-z',
        default=0.0,
        type=float,
        help='Z start position (default: 0.0)')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        default=os.getpid(),
        type=int,
        help='Seed for the random path (default: program pid)')
    argparser.add_argument(
        '-t', '--tick-time',
        metavar='T',
        default=0.2,
        type=float,
        help='Tick time between updates (forward velocity) (default: 0.2)')
    args = argparser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        world = client.get_world()
        m = world.get_map()
        debug = world.debug

        random.seed(args.seed)
        print("Seed: ", args.seed)

        loc = carla.Location(args.x, args.y, args.z)
        print("Initial location: ", loc)

        current_w = m.get_waypoint(loc)  # get_waypoint(self, location, project_to_road=True, lane_type=carla.LaneType.Driving)
        # Generates a waypoint in the center of the closest driving lane as per def

        # main loop
        while True:
            # list of potential next waypoints
            potential_w = list(current_w.next(waypoint_separation))


            # Getting centers of both right and left lanes using the code below

            # check for available right driving lanes
            if current_w.lane_change & carla.LaneChange.Right:
                right_w = current_w.get_right_lane()
                if right_w and right_w.lane_type == carla.LaneType.Driving:
                    potential_w += list(right_w.next(waypoint_separation))

            # check for available left driving lanes
            if current_w.lane_change & carla.LaneChange.Left:
                left_w = current_w.get_left_lane()  # a waypoint int the CENTER of the left lane
                if left_w and left_w.lane_type == carla.LaneType.Driving:
                    potential_w += list(left_w.next(waypoint_separation))

            '''
                get_left_lane(self)
                Generates a Waypoint at the center of the left lane based on the direction of the current Waypoint, taking into account if the lane change is allowed in this location. Will return None if the lane does not exist.
            Return: carla.Waypoint
            
            '''

            # choose a random waypoint to be the next
            next_w = random.choice(potential_w)  # Random seed used here :3
            potential_w.remove(next_w)

            # Render some nice information, notice that you can't see the strings if you are using an editor camera
            if args.info:
                draw_waypoint_info(debug, current_w, trail_life_time)
            draw_waypoint_union(debug, current_w, next_w, cyan if current_w.is_junction else green, trail_life_time)  # union as in connection
            draw_transform(debug, current_w.transform, white, trail_life_time)

            # print the remaining waypoints
            for p in potential_w:  # draw all potential waypoint unions/connections with the current waypoint
                draw_waypoint_union(debug, current_w, p, red, trail_life_time)
                draw_transform(debug, p.transform, white, trail_life_time)

            # draw all junction waypoints and bounding box
            if next_w.is_junction:
                junction = next_w.get_junction()
                draw_junction(debug, junction, trail_life_time)





            # update the current waypoint and sleep for some time
            current_w = next_w
            time.sleep(args.tick_time)

    finally:
        pass


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nExit by user.')
    finally:
        print('\nExit.')
