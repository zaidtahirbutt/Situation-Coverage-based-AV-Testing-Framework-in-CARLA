# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides implementation for GlobalRoutePlannerDAO
"""

import numpy as np


class GlobalRoutePlannerDAO(object):  # DAO, data access object?
    """
    This class is the data access layer for fetching data
    from the carla server instance for GlobalRoutePlanner
    """

    def __init__(self, wmap, sampling_resolution):  # from interpolate in route_manipulation GlobalRoutePlannerDAO(world.get_map(), hop_resolution)  # hop_resolution=1.0
        """
        Constructor method.

            :param wmap: carla.world object
            :param sampling_resolution: sampling distance between waypoints
        """
        self._sampling_resolution = sampling_resolution
        self._wmap = wmap

    def get_topology(self):
        """
        Accessor for topology.
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects.

            :return topology: list of dictionary objects with the following attributes
                entry   -   waypoint of entry point of road segment
                entryxyz-   (x,y,z) of entry point of road segment
                exit    -   waypoint of exit point of road segment
                exitxyz -   (x,y,z) of exit point of road segment
                path    -   list of waypoints separated by 1m from entry
                            to exit
        """
        topology = []
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():  #of the whole map

            '''
            Carla world getter method
            get_topology(self)
            Returns a list of tuples describing a minimal graph of the topology of the OpenDRIVE file. 
            The tuples contain pairs of waypoints located either at the point a road begins or ends. 
            The first one is the origin and the second one represents another road end that can be reached. 
            This graph can be loaded into NetworkX to work with. 
            Output could look like this: [(w0, w1), (w0, w2), (w1, w3), (w2, w3), (w0, w4)].
            Return: list(tuple(carla.Waypoint, carla.Waypoint))
            '''



            wp1, wp2 = segment[0], segment[1]
            l1, l2 = wp1.transform.location, wp2.transform.location
            # Rounding off to avoid floating point imprecision
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg_dict = dict()
            seg_dict['entry'], seg_dict['exit'] = wp1, wp2
            seg_dict['entryxyz'], seg_dict['exitxyz'] = (x1, y1, z1), (x2, y2, z2)
            seg_dict['path'] = []
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                w = wp1.next(self._sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self._sampling_resolution:  # while loop :3
                    seg_dict['path'].append(w)
                    w = w.next(self._sampling_resolution)[0]
            else:
                seg_dict['path'].append(wp1.next(self._sampling_resolution)[0])
            topology.append(seg_dict)
        

        #for segs in topology:
        #    for key in segs.keys():
        #        print(key)

        '''
        These were the keys printed
        entry
        exit
        entryxyz
        exitxyz
        path
        '''

        #for segs in topology:
        #    for key, value in segs.items():
        #        print(f"{key} has these values {value}")

        '''
       The first dictionay segs in topology had the following keys and their corresponding values 
        entry has these values Waypoint(Transform(Location(x=52.700909, y=187.576401, z=0.000000), Rotation(pitch=360.000000, yaw=-179.973694, roll=0.000000)))
        exit has these values Waypoint(Transform(Location(x=34.700912, y=187.568146, z=0.000000), Rotation(pitch=360.000000, yaw=180.026260, roll=0.000000)))
        entryxyz has these values (53.0, 188.0, 0.0)
        exitxyz has these values (35.0, 188.0, 0.0)
        path has these values [<carla.libcarla.Waypoint object at 0x000002032A20AB10>, <carla.libcarla.Waypoint object at 0x000002032A20ABD0>,
        <carla.libcarla.Waypoint object at 0x000002032A20AC30>, <carla.libcarla.Waypoint object at 0x000002032A20AC90>, 
        <carla.libcarla.Waypoint object at 0x000002032A20ACF0>, <carla.libcarla.Waypoint object at 0x000002032A20AD50>, 
        <carla.libcarla.Waypoint object at 0x000002032A20ADB0>, <carla.libcarla.Waypoint object at 0x000002032A20AE10>, 
        <carla.libcarla.Waypoint object at 0x000002032A20AE70>, <carla.libcarla.Waypoint object at 0x000002032A20AED0>,
         <carla.libcarla.Waypoint object at 0x000002032A20AF30>, <carla.libcarla.Waypoint object at 0x000002032A20AF90>,
          <carla.libcarla.Waypoint object at 0x000002032A20B030>, <carla.libcarla.Waypoint object at 0x000002032A20B090>,
           <carla.libcarla.Waypoint object at 0x000002032A20B0F0>, <carla.libcarla.Waypoint object at 0x000002032A20B150>,
            <carla.libcarla.Waypoint object at 0x000002032A20B1B0>]
        '''



        return topology  # a lot of seg_dic in topology. These seg_dic have entries, exits and paths for those particular entry-exit
        #returns a densely populated list of dictionaries that have entry exit and a waypoint path between them

    def get_waypoint(self, location):
        """
        The method returns waypoint at given location

            :param location: vehicle location
            :return waypoint: generated waypoint close to location
        """
        waypoint = self._wmap.get_waypoint(location)
        return waypoint

    def get_resolution(self):
        """ Accessor for self._sampling_resolution """
        return self._sampling_resolution
