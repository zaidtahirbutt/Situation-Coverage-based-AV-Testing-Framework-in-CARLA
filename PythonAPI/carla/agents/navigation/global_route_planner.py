# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


"""
This module provides GlobalRoutePlanner implementation.
"""

import math
import numpy as np
import networkx as nx
import sys

import carla
from agents.navigation.local_planner import RoadOption
from agents.tools.misc import vector


class GlobalRoutePlanner(object):
    """
    This class provides a very high level route plan.
    Instantiate the class by passing a reference to
    A GlobalRoutePlannerDAO object.
    """

    def __init__(self, dao):  # from route_manipulation interpolate functon, grp = GlobalRoutePlanner(dao), dao has world and hop_resolution=1
        """
        Constructor
        """
        self._dao = dao  #object
        self._topology = None
        self._graph = None
        self._id_map = None
        self._road_id_to_edge = None
        self._intersection_end_node = -1
        self._previous_decision = RoadOption.VOID

    def setup(self):
        """
        Performs initial server data lookup for detailed topology
        and builds graph representation of the world map.
        """
        self._topology = self._dao.get_topology()  # returns a densely populated list of dictionaries that have entry exit and a waypoint path between them for a given map.
        self._graph, self._id_map, self._road_id_to_edge = self._build_graph()  # return graph, id_map, road_id_to_edge
        self._find_loose_ends()
        self._lane_change_link()

        #Now we have a full graph made with nodes and edges. Nodes are the waypoint entry exit xyz points and the node ids are their values from the keys of entry exit xyz point and edges are connections between all the entry exits of the roads and all the attributes along with the possible lane changes

    def _build_graph(self):
        """
        This function builds a networkx graph representation of topology.
        The topology is read from self._topology.
        graph node properties:
            vertex   -   (x,y,z) position in world map
        graph edge properties:
            entry_vector    -   unit vector along tangent at entry point
            exit_vector     -   unit vector along tangent at exit point
            net_vector      -   unit vector of the chord from entry to exit
            intersection    -   boolean indicating if the edge belongs to an
                                intersection
        return      :   graph -> networkx graph representing the world map,  id_map and road_id_to_edge can be used to access this graph
                        id_map-> mapping from (x,y,z) to node id. Simply as node id is the unique value of the unique key x, y, z 
                        road_id_to_edge-> map from road id to edge in the graph. each road id corresponds to a unique edge in the graph between two unique nodes (n1, n2) as the lsat values of a road id key in the dictionary with further road segment dictionary with lane key inside it bullyea
        """
        graph = nx.DiGraph()
        id_map = dict()  # Map with structure {(x,y,z): id, ... }
        road_id_to_edge = dict()  # Map with structure {road_id: {lane_id: edge, ... }, ... }

        for segment in self._topology:

            entry_xyz, exit_xyz = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            intersection = entry_wp.is_junction
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id

            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map

                #print(f"{vertex} is vertex in {entry_xyz}-entry_xyz and {exit_xyz}-exit_xyz")  # (53.0, 188.0, 0.0) is vertex in (53.0, 188.0, 0.0)-entry_xyz and (35.0, 188.0, 0.0)-exit_xyz
                #sys.exit("OFF123")

                if vertex not in id_map:
                    new_id = len(id_map)
                    id_map[vertex] = new_id
                    
                    #print(new_id)  # PRINTED 0  # So we giving dic length as node Ids i.e., the dictionary values to the correspondoing keys and the entry and exity xyz of a segment as dictionary keys (two seperate)
                    #sys.exit("OFF123")

                    graph.add_node(new_id, vertex=vertex)  # adding nodes and their values here
            n1 = id_map[entry_xyz]  # for they xyz keys, we have following ID values (length of the id map dictionary)
            n2 = id_map[exit_xyz]
            if road_id not in road_id_to_edge:  # entry_wp.road_id
                road_id_to_edge[road_id] = dict()  # new dictionary for a particular (each) road id
            if section_id not in road_id_to_edge[road_id]:  # entry_wp.section_id, entry_wp.lane_id
                road_id_to_edge[road_id][section_id] = dict()  # new dictionary (or a particular (each) section id) inside a road id dictionary, inside it's section id key.
            road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)  # new dictionary inside a road id dictionary, inside it's section id key dictionary, inside it's lane id key, is the id_map dictionary values tuple that are inside entry exite xyz keys and all IDs are unique as well as they correspond to the growing length of their dictionary .

            entry_carla_vector = entry_wp.transform.rotation.get_forward_vector()
            exit_carla_vector = exit_wp.transform.rotation.get_forward_vector()

            # Adding edge with all these FLUFFIN attributes
            graph.add_edge(
                n1, n2,  # these are two nodes, hence an edge is between two nodes. They have vectors between entry and exit xyz, not just way points and the full densly populated path between them
                length=len(path) + 1, path=path,  # path is the path of a particular segment
                entry_waypoint=entry_wp, exit_waypoint=exit_wp,
                entry_vector=np.array(
                    [entry_carla_vector.x, entry_carla_vector.y, entry_carla_vector.z]),
                exit_vector=np.array(
                    [exit_carla_vector.x, exit_carla_vector.y, exit_carla_vector.z]),
                net_vector=vector(entry_wp.transform.location, exit_wp.transform.location),
                intersection=intersection, type=RoadOption.LANEFOLLOW)

        return graph, id_map, road_id_to_edge

    def _find_loose_ends(self):
        """
        This method finds road segments that have an unconnected end, and
        adds them to the internal graph representation
        """
        count_loose_ends = 0
        hop_resolution = self._dao.get_resolution()
        for segment in self._topology:
            end_wp = segment['exit']
            exit_xyz = segment['exitxyz']
            road_id, section_id, lane_id = end_wp.road_id, end_wp.section_id, end_wp.lane_id
            if road_id in self._road_id_to_edge and section_id in self._road_id_to_edge[road_id] and lane_id in self._road_id_to_edge[road_id][section_id]:
                pass
            else:
                count_loose_ends += 1
                if road_id not in self._road_id_to_edge:
                    self._road_id_to_edge[road_id] = dict()
                if section_id not in self._road_id_to_edge[road_id]:  # if this key section_id is not in this dic
                    self._road_id_to_edge[road_id][section_id] = dict()
                n1 = self._id_map[exit_xyz]  # id_map[vertex] = new_id??
                n2 = -1*count_loose_ends
                self._road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)
                next_wp = end_wp.next(hop_resolution)
                path = []
                while next_wp is not None and next_wp and next_wp[0].road_id == road_id and next_wp[0].section_id == section_id and next_wp[0].lane_id == lane_id:
                    path.append(next_wp[0])
                    next_wp = next_wp[0].next(hop_resolution)
                if path:
                    n2_xyz = (path[-1].transform.location.x,
                              path[-1].transform.location.y,
                              path[-1].transform.location.z)
                    self._graph.add_node(n2, vertex=n2_xyz)
                    self._graph.add_edge(
                        n1, n2,
                        length=len(path) + 1, path=path,  # path of a particular segment
                        entry_waypoint=end_wp, exit_waypoint=path[-1],
                        entry_vector=None, exit_vector=None, net_vector=None,
                        intersection=end_wp.is_junction, type=RoadOption.LANEFOLLOW)

    def _localize(self, location):
        """
        This function finds the road segment closest to given location
        location        :   carla.Location to be localized in the graph
        return          :   pair node ids representing an edge in the graph
        """
        waypoint = self._dao.get_waypoint(location)
        edge = None
        try:
            edge = self._road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        except KeyError:
            print(
                "Failed to localize! : ",
                "Road id : ", waypoint.road_id,
                "Section id : ", waypoint.section_id,
                "Lane id : ", waypoint.lane_id,
                "Location : ", waypoint.transform.location.x,
                waypoint.transform.location.y)
        return edge

    def _lane_change_link(self):
        """
        This method places zero cost links in the topology graph - Path is empty [] and Length is 0, type=next_road_option 1 for left and 2 for right
        representing availability of lane changes.
        """

        for segment in self._topology:
            left_found, right_found = False, False

            for waypoint in segment['path']:
                if not segment['entry'].is_junction:
                    next_waypoint, next_road_option, next_segment = None, None, None

                    if waypoint.right_lane_marking.lane_change & carla.LaneChange.Right and not right_found:
                        next_waypoint = waypoint.get_right_lane()
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANERIGHT  # = 2
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint, intersection=False, exit_vector=None,
                                    path=[], length=0, type=next_road_option, change_waypoint=next_waypoint)
                                right_found = True
                    if waypoint.left_lane_marking.lane_change & carla.LaneChange.Left and not left_found:
                        next_waypoint = waypoint.get_left_lane()
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANELEFT
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint, intersection=False, exit_vector=None,
                                    path=[], length=0, type=next_road_option, change_waypoint=next_waypoint)
                                left_found = True
                if left_found and right_found:
                    break

    def _distance_heuristic(self, n1, n2):
        """
        Distance heuristic calculator for path searching
        in self._graph
        """
        l1 = np.array(self._graph.nodes[n1]['vertex'])
        l2 = np.array(self._graph.nodes[n2]['vertex'])
        return np.linalg.norm(l1-l2)

    def _path_search(self, origin, destination):
        """
        This function finds the shortest path connecting origin and destination
        using A* search with distance heuristic.
        origin      :   carla.Location object of start position
        destination :   carla.Location object of of end position
        return      :   path as list of node ids (as int) of the graph self._graph
        connecting origin and destination
        """

        start, end = self._localize(origin), self._localize(destination)

        #print(f"{start[0]} is the start and {end[0]} is the end of first iteration")  # 10 is the start and 10 is the end of first iteration
        #sys.exit("hawk")


        route = nx.astar_path(
            self._graph, source=start[0], target=end[0],  #start[0] and end[0] are the first ids of the nodes i.e the length of the node dictionary
            heuristic=self._distance_heuristic, weight='length') 
        route.append(end[1])


        #print(route) #[10, 11] guess these are the graph node IDs cx localization returned edges so end[1] was the ending node
        #sys.exit("awk")

        return route

    def _successive_last_intersection_edge(self, index, route):
        """
        This method returns the last successive intersection edge
        from a starting index on the route.
        This helps moving past tiny intersection edges to calculate
        proper turn decisions.
        """

        last_intersection_edge = None
        last_node = None
        for node1, node2 in [(route[i], route[i+1]) for i in range(index, len(route)-1)]:
            candidate_edge = self._graph.edges[node1, node2]
            if node1 == route[index]:
                last_intersection_edge = candidate_edge
            if candidate_edge['type'] == RoadOption.LANEFOLLOW and candidate_edge['intersection']:
                last_intersection_edge = candidate_edge
                last_node = node2
            else:
                break

        return last_node, last_intersection_edge

    def _turn_decision(self, index, route, threshold=math.radians(35)):
        """
        This method returns the turn decision (RoadOption) for pair of edges
        around current index of route list
        """

        decision = None
        previous_node = route[index-1]
        current_node = route[index]
        next_node = route[index+1]
        next_edge = self._graph.edges[current_node, next_node]
        if index > 0:
            if self._previous_decision != RoadOption.VOID and self._intersection_end_node > 0 and self._intersection_end_node != previous_node and next_edge['type'] == RoadOption.LANEFOLLOW and next_edge['intersection']:
                decision = self._previous_decision
            else:
                self._intersection_end_node = -1
                current_edge = self._graph.edges[previous_node, current_node]
                calculate_turn = current_edge['type'] == RoadOption.LANEFOLLOW and not current_edge[
                    'intersection'] and next_edge['type'] == RoadOption.LANEFOLLOW and next_edge['intersection']
                if calculate_turn:
                    last_node, tail_edge = self._successive_last_intersection_edge(index, route)
                    self._intersection_end_node = last_node
                    if tail_edge is not None:
                        next_edge = tail_edge
                    cv, nv = current_edge['exit_vector'], next_edge['exit_vector']
                    if cv is None or nv is None:
                        return next_edge['type']
                    cross_list = []
                    for neighbor in self._graph.successors(current_node):
                        select_edge = self._graph.edges[current_node, neighbor]
                        if select_edge['type'] == RoadOption.LANEFOLLOW:
                            if neighbor != route[index+1]:
                                sv = select_edge['net_vector']
                                cross_list.append(np.cross(cv, sv)[2])
                    next_cross = np.cross(cv, nv)[2]
                    deviation = math.acos(np.clip(
                        np.dot(cv, nv)/(np.linalg.norm(cv)*np.linalg.norm(nv)), -1.0, 1.0))
                    if not cross_list:
                        cross_list.append(0)
                    if deviation < threshold:
                        decision = RoadOption.STRAIGHT
                    elif cross_list and next_cross < min(cross_list):
                        decision = RoadOption.LEFT
                    elif cross_list and next_cross > max(cross_list):
                        decision = RoadOption.RIGHT
                    elif next_cross < 0:
                        decision = RoadOption.LEFT
                    elif next_cross > 0:
                        decision = RoadOption.RIGHT
                else:
                    decision = next_edge['type']

        else:
            decision = next_edge['type']

        self._previous_decision = decision
        return decision

    def abstract_route_plan(self, origin, destination):
        """
        The following function generates the route plan based on
        origin      : carla.Location object of the route's start position
        destination : carla.Location object of the route's end position
        return      : list of turn by turn navigation decisions as
        agents.navigation.local_planner.RoadOption elements
        Possible values are STRAIGHT, LEFT, RIGHT, LANEFOLLOW, VOID
        CHANGELANELEFT, CHANGELANERIGHT
        """

        route = self._path_search(origin, destination)  # in node format
        plan = []

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            plan.append(road_option)

        return plan

    def _find_closest_in_list(self, current_waypoint, waypoint_list):
        min_distance = float('inf')
        closest_index = -1
        for i, waypoint in enumerate(waypoint_list):
            distance = waypoint.transform.location.distance(
                current_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def trace_route(self, origin, destination):
        """
        This method returns list of (carla.Waypoint, RoadOption)
        from origin to destination
        """

        route_trace = []
        route = self._path_search(origin, destination)  #A* search
        #print(route) #[10, 11] guess these are the graph node IDs cx localization returned edges so end[1] was the ending node
        #sys.exit("awk")



        current_waypoint = self._dao.get_waypoint(origin)
        destination_waypoint = self._dao.get_waypoint(destination)
        resolution = self._dao.get_resolution()

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)  # print(route) #[10, 11] guess these are the graph node IDs cx localization returned edges so end[1] was the ending node
            # return decision in road_option above (RoadOption.LEFT,RIGHT etc) for the route we have given, explore the graph and let us know the turn decisions



            edge = self._graph.edges[route[i], route[i+1]]
            path = []

            if edge['type'] != RoadOption.LANEFOLLOW and edge['type'] != RoadOption.VOID:
                route_trace.append((current_waypoint, road_option))
                exit_wp = edge['exit_waypoint']
                n1, n2 = self._road_id_to_edge[exit_wp.road_id][exit_wp.section_id][exit_wp.lane_id]  # next edge to the route[i+1]
                next_edge = self._graph.edges[n1, n2]
                if next_edge['path']:
                    closest_index = self._find_closest_in_list(current_waypoint, next_edge['path'])
                    closest_index = min(len(next_edge['path'])-1, closest_index+5)
                    current_waypoint = next_edge['path'][closest_index]
                else:
                    current_waypoint = next_edge['exit_waypoint']
                route_trace.append((current_waypoint, road_option))

            else:
                path = path + [edge['entry_waypoint']] + edge['path'] + [edge['exit_waypoint']]
                closest_index = self._find_closest_in_list(current_waypoint, path)
                for waypoint in path[closest_index:]:
                    current_waypoint = waypoint
                    route_trace.append((current_waypoint, road_option))
                    if len(route)-i <= 2 and waypoint.transform.location.distance(destination) < 2*resolution:
                        break
                    elif len(route)-i <= 2 and current_waypoint.road_id == destination_waypoint.road_id and current_waypoint.section_id == destination_waypoint.section_id and current_waypoint.lane_id == destination_waypoint.lane_id:
                        destination_index = self._find_closest_in_list(destination_waypoint, path)
                        if closest_index > destination_index:
                            break

        return route_trace
