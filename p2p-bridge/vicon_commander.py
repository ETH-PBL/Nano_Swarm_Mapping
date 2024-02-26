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
import time
from datetime import datetime
from threading import Thread
from typing import List

from pyvicon.pyvicon import PyVicon, StreamMode, Direction, Result


class ViconWrapper(Thread):
    def __init__(self, ip: str, period: float, subjects: List[str], time0, path: str, markers: bool = False):
        Thread.__init__(self)
        self.ip = ip
        self.period = period
        self.subjects = subjects
        self.t0 = time0
        self.track_markers = markers

        self.vicon = None
        self.file = open(os.path.join(path, "vicon.csv"), "w")
        self.logging_en = False
        self.position = dict([])
        self.quaternions = dict([])
        self.should_disconnect = False

        self.start()

    def run(self):
        self.connect()
        self.loop()

    def connect(self):
        self.vicon = PyVicon()
        print("[Vicon]", "SDK version : {}".format(self.vicon.__version__))
        print("[Vicon]", self.vicon.connect(self.ip))
        print("[Vicon]", "Connection status : {}".format(self.vicon.is_connected()))
        self.vicon.set_stream_mode(StreamMode.ServerPush)
        self.vicon.enable_segment_data()
        self.vicon.enable_marker_data()
        self.vicon.enable_unlabeled_marker_data()
        self.vicon.enable_device_data()
        self.vicon.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)

    def loop(self):
        while 1:
            if self.should_disconnect:
                self.vicon.disconnect()
                break
            while self.vicon.get_frame() != Result.Success:
                print("[Vicon]", "Dropped frame!")
                time.sleep(self.period / 1000.0)
            timestamp = int(1000 * (datetime.now() - self.t0).total_seconds())
            subj_count = self.vicon.get_subject_count()
            marker_count = self.vicon.get_unlabeled_marker_count()
            for i in range(0, subj_count):
                name = self.vicon.get_subject_name(i)
                if name in self.subjects:
                    pos = self.vicon.get_segment_global_translation(name, name)
                    quat = self.vicon.get_segment_global_quaternion(name, name)
                    if quat is not None:
                        pos = pos / 1000.0
                        self.position[name] = pos
                        self.quaternions[name] = quat
                        if self.logging_en:
                            prefix = name + "_"
                            self.log(timestamp, prefix + "posx", pos[0])
                            self.log(timestamp, prefix + "posy", pos[1])
                            self.log(timestamp, prefix + "posz", pos[2])
                            self.log(timestamp, prefix + "qw", quat[0])
                            self.log(timestamp, prefix + "qx", quat[1])
                            self.log(timestamp, prefix + "qy", quat[2])
                            self.log(timestamp, prefix + "qz", quat[3])
            if self.logging_en and self.track_markers:
                for i in range(marker_count):
                    pos = self.vicon.get_unlabeled_marker_global_translation(0)
                    prefix = "marker" + str(i) + "_"
                    self.log(timestamp, prefix + "posx", pos[0] / 1000)
                    self.log(timestamp, prefix + "posy", pos[1] / 1000)
                    self.log(timestamp, prefix + "posz", pos[2] / 1000)

            time.sleep(self.period / 1000.0)

    def log(self, timestamp: int, id_var: str, *values):
        self.file.write(f"{timestamp},{id_var},{','.join(map(str, values))}\n")

    def save_log(self):
        self.file.flush()

    def logging_enabled(self, val: bool):
        self.logging_en = val

    def disconnect(self):
        self.should_disconnect = True
