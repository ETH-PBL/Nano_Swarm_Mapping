#
# Copyright (C) 2023 ETH Zurich
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the GPL-3.0 license.  See the LICENSE file for details.
#
# Author: Carl Friess
#


import os
import sys
import struct
from datetime import datetime
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
from termcolor import colored

vicon_enabled = True

if vicon_enabled:
    from vicon_commander import ViconWrapper

if len(sys.argv) != 2:
    print(f"Usage: {os.path.basename(sys.argv[0])} path")
    exit(1)

path = sys.argv[1]
os.makedirs(path, exist_ok=True)
pose_log = open(os.path.join(path, "poses.csv"), "w")
scan_log = open(os.path.join(path, "scans.csv"), "w")
if vicon_enabled:
    vicon_log = open(os.path.join(path, "vicon_poses.csv"), "w")

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


def _packet_received(packet):
    if packet.channel == 0:
        _packet_received.data = packet.datal
    elif packet.channel == 1:
        _packet_received.data += packet.datal
        handle_packet(_packet_received.data)
        _packet_received.data = []


_packet_received.data = []


def _msg_packet_received(packet):
    if packet.channel == 0:
        [src, tag, length] = struct.unpack("<BBL", bytes(packet.datal[:6]))
        _msg_packet_received.src = src
        _msg_packet_received.tag = tag
        _msg_packet_received.len = length
        _msg_packet_received.data = []
    elif packet.channel == 1:
        _msg_packet_received.data += packet.datal
        if len(_msg_packet_received.data) == _msg_packet_received.len:
            print(datetime.now().time(),
                  f"{colored(str(_msg_packet_received.src), 'magenta')} => MSG " +
                  f"TAG={_msg_packet_received.tag} LEN={_msg_packet_received.len}")
            print("".join("{:02x}".format(x) for x in _msg_packet_received.data))
            decode_msg(_msg_packet_received.tag, bytes(_msg_packet_received.data))


cf.connected.add_callback(_connected)
cf.disconnected.add_callback(_disconnected)
cf.connection_failed.add_callback(_connection_failed)
cf.connection_lost.add_callback(_connection_lost)
cf.add_port_callback(1, _packet_received)
cf.add_port_callback(10, _msg_packet_received)

cf.open_link("usb://0")

seq_nums = {}
messages = {}
drop_filter = []


def handle_packet(data):
    src = data[0] & 0x0F
    dst = (data[0] >> 4) & 0x0F
    ack = bool(data[1] & 0x01)
    end = bool(data[1] & 0x02)
    seq = (data[1] >> 4) & 0x0F
    tag = (data[1] >> 2) & 0x03
    print(datetime.now().time(),
          f"{colored(str(src), 'blue')} -> {colored('B' if dst == 0xF else str(dst), 'blue')} SEQ={seq} " +
          (colored('[ACK] ', 'green') if ack else f"TAG={tag} ") + colored('[END] ' if end else '', 'yellow') +
          "".join("{:02x}".format(x) for x in data[2:]))

    # # Assemble messages
    # if not ack:
    #     # Check for new packets
    #     if src in drop_filter:
    #         if end:
    #             drop_filter.remove(src)
    #     elif src not in seq_nums or (seq_nums[src] + 1) % 16 == seq:
    #         seq_nums[src] = seq
    #         if src not in messages:
    #             messages[src] = []
    #         messages[src] += data[2:]
    #         if end:
    #             print(datetime.now().time(),
    #                   f"{colored(str(src), 'magenta')} => {colored('B' if dst == 0xF else str(dst), 'magenta')} " +
    #                   f"(MSG) TAG={tag} LEN={len(messages[src])}")
    #             print("".join("{:02x}".format(x) for x in messages[src]))
    #             decode_msg(tag, bytes(messages[src]))
    #             messages[src] = []
    #     elif seq_nums[src] != seq:
    #         print(colored(f">> Packet drop detected from {src}!", "red"))
    #         drop_filter.append(src)
    #         del seq_nums[src]
    #         messages[src] = []


def decode_pose_id(uint16):
    return (uint16 >> 12) & 0xF, uint16 & 0xFFF


def decode_msg(tag, msg):
    # Pose broadcast message
    if tag == 0:
        [x, y, yaw, pose_id, num_edges] = struct.unpack("<fffHBx", msg[:16])
        pose_drone, pose_node = decode_pose_id(pose_id)
        print(colored(f"Pose {pose_drone}/{pose_node}:", "cyan"))
        print(colored(f"X:{x} Y:{y} YAW:{yaw}", "cyan"))
        print(colored(f"Num edges: {num_edges}", "cyan"))
        for edge in [msg[16 + i:16 + i + 16] for i in range(0, num_edges * 16, 16)]:
            [t_x, t_y, t_yaw, to] = struct.unpack("<fffHxx", edge)
            to_drone, to_node = decode_pose_id(to)
            print(colored(f" -> {to_drone}/{to_node}: X:{t_x} Y:{t_y} YAW:{t_yaw}", "cyan"))
        pose_log.write(f"{pose_drone}/{pose_node},{x},{y},{yaw}\n")
        pose_log.flush()
    elif tag == 1:
        [pose_id] = struct.unpack("<H", msg)
        pose_drone, pose_node = decode_pose_id(pose_id)
        print(colored(f"Scan request: {pose_drone}/{pose_node}", "cyan"))
    elif tag == 2:
        [pose_id] = struct.unpack("<H", msg[:2])
        pose_drone, pose_node = decode_pose_id(pose_id)
        print(colored(f"Scan response: {pose_drone}/{pose_node}", "cyan"))
        scan_log.write(f"{pose_drone}/{pose_node},{msg.hex()}\n")
        scan_log.flush()
    elif tag == 3:
        len(msg)
        [type, pose_id] = struct.unpack("<BxH", msg)
        if type == 0:
            print(colored("Control message: START", "cyan"))
        elif type == 1:
            print(colored("Control message: DONE", "cyan"))
        elif type == 2:
            pose_drone, pose_node = decode_pose_id(pose_id)
            print(colored(f"Control message: POSE {pose_drone}/{pose_node}", "cyan"))
            if vicon_enabled:
                pose_update(pose_drone, pose_node)


def send_start(addr):
    packet = CRTPPacket()
    packet.port = 11
    packet.channel = 0
    packet.data = struct.pack("<BiHxx", addr, 0, 0)
    cf.send_packet(packet)


# Vicon logging
if vicon_enabled:
    to_log_vicon = ["DRONE_0", "DRONE_1", "DRONE_2", "DRONE_3"]
    vicon = ViconWrapper(ip="192.168.10.1", period=20, subjects=to_log_vicon, time0=datetime.now(), path=path)


def pose_update(drone, node):
    key = f"DRONE_{drone}"
    if key in vicon.position:
        pos = vicon.position[key]
        quat = vicon.quaternions[key]
        vicon_log.write(f"{drone}/{node},{pos[0]},{pos[1]},{pos[2]},{quat[0]},{quat[1]},{quat[2]},{quat[3]}\n")
        vicon_log.flush()
        print(f"{drone}/{node}:", pos)


for i in range(4):
    input(f"Press Enter to start drone {i}...\n")
    send_start(i)
