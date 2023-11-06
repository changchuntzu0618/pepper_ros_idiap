######################################################################
# Copyright (c) 2018 Idiap Research Institute <http://www.idiap.ch/>
######################################################################

from __future__ import print_function

import os
import sys
import time
import argparse

from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

import qi
import logging

from gui import QiInterfaceBase

from robot import Robot

from pprint import pprint

class QiInterfaceImagePosition(QiInterfaceBase):
    """
    """
    def __init__(self, ip, port, x=-1, y=-1, no_connect=False):
        """
        """
        super(type(self), self).__init__(ip,
                                         port,
                                         with_camera=2,
                                         no_connect=no_connect,
                                         x=x, y=y)


    def _create_turn_box(self):
        """Create buttons to rotate the robot"""
        gpe = QtWidgets.QGroupBox("Rotation", self)
        layout = QtWidgets.QVBoxLayout()

        for angle in [30.0, 10.0, -10.0, -30.0]:
            name = "Left" if angle > 0 else "Right"
            btn = QtWidgets.QPushButton("{} {:g}".format(name, abs(angle)), self)
            btn.clicked.connect(lambda s, t=angle: self.robot.move_to(0, 0, t))
            layout.addWidget(btn)

        # Finally add centering
        layout.addWidget(self._center_body_with_head_btn())

        gpe.setLayout(layout)
        return gpe


    def _create_misc_box(self):
        """Create buttons to rotate the robot"""
        gpe = QtWidgets.QGroupBox("Rotation", self)
        layout = QtWidgets.QVBoxLayout()

        layout.addWidget(self._stand_init_fast_btn())
        layout.addWidget(self._stop_altracker_btn())
        layout.addWidget(self._show_on_the_right_btn())
        layout.addWidget(self._show_on_the_left_below_btn())
        layout.addWidget(self._show_on_the_left_btn())
        layout.addWidget(self._show_behind_btn())

        gpe.setLayout(layout)
        return gpe


    def _create_gui(self):
        """
        """
        self.setWindowTitle("Robot front image")

        self.grid = QtWidgets.QGridLayout()

        # Main layout
        hbox = QtWidgets.QGridLayout(self)

        self.row_id = 0
        self.col_id = 0
        hbox.addWidget(self._create_head_control_box(), self.row_id, self.col_id)

        self.row_id += 1
        self.col_id = 0
        hbox.addWidget(self._create_turn_box(), self.row_id, self.col_id)

        self.row_id += 1
        self.col_id = 0
        hbox.addWidget(self._create_misc_box(), self.row_id, self.col_id)

        self.row_id += 1
        self.col_id = 0
        hbox.addWidget(self._create_quit_box(), self.row_id, self.col_id)

        self.setLayout(hbox)


if __name__ == "__main__":
    """
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip",
                        type=str,
                        default=os.environ.get("NAO_IP", None),
                        help="Robot IP address.")
    parser.add_argument("--port",
                        type=int,
                        default=9559,
                        help="Naoqi port number")
    parser.add_argument('--no-connect',
                        action='store_true',
                        help="Don't connect to robot")
    parser.add_argument("-x",
                        type=int,
                        default=-1,
                        help="Position of the window")
    parser.add_argument("-y",
                        type=int,
                        default=-1,
                        help="Position of the window")

    opts = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(message)s",
                        datefmt="%H:%M:%S")


    app = QtWidgets.QApplication(sys.argv)
    gui = QiInterfaceImagePosition(opts.ip, opts.port,
                                   opts.x, opts.y,
                                   no_connect=opts.no_connect)
    gui.show()
    sys.exit(app.exec_())
