#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import networkx as nx
import matplotlib.pyplot as plt

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    map = world.get_map()

    topology = map.get_topology()
    # G = nx.petersen_graph()
    # nodes = set()
    # for a, b in topology:
    #     nodes.add(a)
    #     nodes.add(b)
    # G.add_nodes_from(nodes)
    # G.add_edges_from(topology)

    # nx.draw(G, pos=lambda x: x.transform.location)
    # plt.savefig("graph.png")

    # waypoints = map.generate_waypoints(2.0)
    # for w in waypoints:
    #     if w.transform.location.x < 1.0:
    #         print(w.transform)
    #     world.debug.draw_point(w.transform.location, life_time=20)

    for first, second in topology:
        # if first.transform.location.x < 1.0:
        #     print(first.transform)
        # if second.transform.location.x < 1.0:
        #     print(second.transform)
        world.debug.draw_line(
            first.transform.location + carla.Location(z=6),
            second.transform.location + carla.Location(z=6),
            life_time=60)


if __name__ == '__main__':

    main()
