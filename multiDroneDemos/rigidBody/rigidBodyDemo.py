# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

import numpy as np

# URI to the Crazyflie to connect to
# We're gonna make a pyramid thing
uri0 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
uri3 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
uris = [uri0, uri1, uri2, uri3] #Defining the array for all 4 vertices

z_offset = 0.2 # We add in this z offset to account for the drone init height

#Defining the rotation matrices for some borderline CG stuffs
R_x = lambda alpha: np.array(
    [1, 0, 0],
    [0, np.cos(alpha), -np.sin(alpha)],
    [0, np.sin(alpha), np.cos(alpha)]
)

R_y = lambda alpha: np.array(
    [np.cos(alpha), 0, np.sin(alpha)],
    [0, 1, 0],
    [-np.sin(alpha), 0, np.cos(alpha)]
)

R_z = lambda alpha: np.array(
    [np.cos(alpha), -np.sin(alpha), 0],
    [np.sin(alpha), np.cos(alpha), 0],
    [0, 0, 1]
)

vertexes = [] # Vertexes defines the initial positions of for the drones

for i in range(0,3):
    v = np.array(np.cos(i * 2 * np.pi / 3), np.cos(i * 2 * np.pi / 3), z_offset - np.sqrt(2)/2)
    vertexes.append(v)
vertexes.append(np.array(0,0,np.sqrt(2)/2))

for vertex in vertexes:
    vertex += np.array(0,0,z_offset)

# Initialize the vertex positions
def initVertexPositions():
    for uriIndex in range(0, uris.len()):
        with SyncCrazyflie(uris[uriIndex], cf=Crazyflie(rw_cache='./cache')) as scf:
                # Arm the Crazyflie
                scf.cf.supervisor.send_arming_request(True)
                time.sleep(1.0)
        
                with PositionHlCommander(
                        scf,
                        x=0.0, y=0.0, z=0.0,
                        default_velocity=0.3,
                        default_height=0.5,
                        controller=PositionHlCommander.CONTROLLER_PID) as pc:
                            x, y, z = vertexes[uriIndex]
                            pc.go_to(x, y, z, 0.5)
                            pass

# def land_on_elevated_surface():
#     with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
#         # Arm the Crazyflie
#         scf.cf.supervisor.send_arming_request(True)
#         time.sleep(1.0)

#         with PositionHlCommander(scf,
#                                  default_height=0.5,
#                                  default_velocity=0.2,
#                                  default_landing_height=0.35,
#                                  controller=PositionHlCommander.CONTROLLER_PID) as pc:
#             # fly onto a landing platform at non-zero height (ex: from floor to desk, etc)
#             pc.forward(1.0)
#             pc.left(1.0)
#             # land() will be called on context exit, gradually lowering to default_landing_height, then stopping motors

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    initVertexPositions()

    time.sleep(3)



    # slightly_more_complex_usage()
    # land_on_elevated_surface()