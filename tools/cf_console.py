#
# Copyright (C) 2023 ETH Zurich
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the GPL-3.0 license.  See the LICENSE file for details.
#
# Author: Carl Friess
#


import os.path
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie

if len(sys.argv) != 2:
    print(f"Usage: {os.path.basename(sys.argv[0])} uri")
    exit(1)

cflib.crtp.init_drivers(enable_debug_driver=False)

cf = Crazyflie(rw_cache='./cf_cache')
is_connected = False


def _connected(link_uri):
    global is_connected
    print("[CF]", f"Connected to {link_uri}")
    is_connected = True


def _connection_failed(link_uri, msg):
    global is_connected
    print("[CF]", f"Connection to {link_uri} failed: {msg}")
    is_connected = False


def _connection_lost(link_uri, msg):
    global is_connected
    print("[CF]", f"Connection to {link_uri} lost: {msg}")
    is_connected = False


def _disconnected(link_uri):
    global is_connected
    print("[CF]", f"Disconnected from {link_uri}")
    is_connected = False


cf.connected.add_callback(_connected)
cf.disconnected.add_callback(_disconnected)
cf.connection_failed.add_callback(_connection_failed)
cf.connection_lost.add_callback(_connection_lost)
cf.console.receivedChar.add_callback(lambda chars: sys.stdout.write(chars))

cf.open_link(sys.argv[1] + "?rate_limit=10")
