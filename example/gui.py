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

from image_position_getter import ImagePositionGetter

from utils import animations

from robot import Robot

import logging

# class FunctionThread(QtCore.QThread):
#     """A Qt thread to keep the main window responsive"""

#     def __init__(self, f):
#         super(FunctionThread, self).__init__()
#         self.function = f

#     def run(self):
#         self.function()


class QiInterfaceBase(QtWidgets.QWidget):
    """The base class that holds all functions to create buttons. The
       __create_gui() has to be implemented in the derived class to
       build the GUI.

    """
    def __init__(self, ip, port,
                 with_camera=0,
                 no_connect=False,
                 columns=2,
                 x=-1,
                 y=-1):
        """
        """
        super(QiInterfaceBase, self).__init__()

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+W"), self, self.close)
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.close)

        self.ip = ip
        self.port = port
        self.session = qi.Session()
        if not no_connect:
            self._connect_to_robot()
        self.robot = Robot(self.session)

        # When equal to True, no_camera will display some buttons to
        # move the head.
        #
        # When equal to False, it displays the (front) camera image,
        # and clicking on a point of the image will make the robot
        # point at that point.
        self.with_camera = with_camera

        self.row_id = 0
        self.col_id = 0
        self.max_nb_columns = columns

        self._create_gui()

        if not (x < 0 and y < 0):
            self.move(x, y)

        # self.thread = FunctionThread(self.robot.wake_up)

    # def _start_thread(self):
    #     if not self.thread.isRunning():
    #         self.thread.start()

    # def _thread_btn(self):
    #     btn = QtGui.QPushButton("Wake up", self)
    #     btn.clicked.connect(self._start_thread)
    #     return btn



    def _increment_indices(self):
        """Increment row_id and col_id"""
        self.col_id += 1
        if self.col_id == self.max_nb_columns:
            self.col_id = 0
            self.row_id += 1


    def _wake_up_btn(self):
        """Return a button to call wake_up"""
        btn = QtWidgets.QPushButton("Wake up", self)
        btn.clicked.connect(self.robot.wake_up)
        return btn


    def _rest_btn(self):
        """Return a button to call rest"""
        btn = QtWidgets.QPushButton("Rest", self)
        btn.clicked.connect(self.robot.rest)
        return btn


    def _center_body_with_head_btn(self):
        """"""
        btn = QtWidgets.QPushButton("Center body", self)
        btn.clicked.connect(self.robot.center_body_with_head)
        return btn


    def _stop_altracker_btn(self):
        """Return a button to stop the tracker (bug from SBRE)"""
        btn = QtWidgets.QPushButton("No Tracker", self)
        btn.clicked.connect(self.robot.disable_altraker)
        return btn


    def _bow_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Bow", self)
        btn.clicked.connect(self.robot.bow)
        return btn


    # def _stand_init_btn(self):
    #     """
    #     """
    #     btn = QtGui.QPushButton("StandInit", self)
    #     btn.clicked.connect(lambda: self.robot.apply_posture("StandInit", 0.4))
    #     return btn


    def _say_btn(self, text_to_say, text_to_display=""):
        """Return a button with a text to be said"""
        if len(text_to_display) == 0:
            text_to_display = text_to_say
        btn = QtWidgets.QPushButton(text_to_display, self)
        btn.clicked.connect(lambda: self.robot.say(text_to_say))
        return btn


    def _raise_both_hands_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Raise both hands", self)
        btn.clicked.connect(self.robot.raise_both_hands)
        return btn


    def _hold_glass_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Hold glass", self)
        btn.clicked.connect(self.robot.hold_glass)
        return btn


    def _show_on_the_right_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Show on the right", self)
        # btn.clicked.connect(self.robot.show_on_the_right)
        btn.clicked.connect(lambda:
                            [self.robot.show_on_the_right_high(),
                             time.sleep(0.5),
                             self.robot.stand_init_fast()])
        return btn


    def _show_behind_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Show behind", self)
        # btn.clicked.connect(self.robot.show_on_the_left)
        btn.clicked.connect(lambda:
                            [self.robot.show_behind(),
                             time.sleep(0.5),
                             self.robot.stand_init_fast()])
        return btn

    def _show_on_the_left_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Show left", self)
        btn.clicked.connect(lambda:
                            [self.robot.show_on_the_left(),
                             time.sleep(0.5),
                             self.robot.stand_init_fast()])
        return btn

    def _photo_pose_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Photo pose", self)
        # btn.clicked.connect(self.robot.show_on_the_left)
        btn.clicked.connect(lambda:
                            [self.robot.photo_pose()])

        return btn


    def _show_on_the_left_below_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Show on the left below", self)
        btn.clicked.connect(lambda:
                            [self.robot.show_on_the_left_below(),
                             time.sleep(0.5),
                             self.robot.stand_init_fast()])
        return btn


    def _stand_init_fast_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Stand init", self)
        btn.clicked.connect(self.robot.stand_init_fast)
        return btn


    def _connect_to_robot(self):
        """
        """
        try:
            print("Connecting to robot {}:{}".format(self.ip, self.port))
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print ("Cannot connect ip {} on port {}".format(self.ip, self.port))
            # sys.exit(1)


    def _interrupt_animated_speech(self):
        """
        """
        print("Interrupting animation")
        self.robot.interrupt_animated_speech()


    def _mute_animated_speech(self):
        """
        """
        print("Starting animation")
        animated_speech("^start({explain1}) \\pau=10000\\ ^stop({explain1})". \
                        format(**animations))


    def _quit_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Quit", self)
        btn.clicked.connect(exit)
        return btn


    def _interrupt_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Interrupt", self)
        btn.clicked.connect(self._interrupt_animated_speech)
        btn.setStyleSheet("background-color: red")
        return btn


    def _animated_speech_btn(self, btn_name, file_name, lang):
        """
        """
        btn = QtWidgets.QPushButton(btn_name, self)
        btn.clicked.connect \
            (lambda: self.robot.animated_speech(formatted_file=file_name))
        return btn


    def _nod_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Nod", self)
        btn.clicked.connect(self.robot.nod)
        return btn


    def _show_image_btn(self, btn_name, image_path_on_robot):
        """
        """
        btn = QtWidgets.QPushButton(btn_name, self)
        btn.clicked.connect(lambda: self.robot.show_image(image_path_on_robot))
        return btn


    def _create_head_control_box(self):
        """Return a group with to control the head"""

        if self.with_camera == 1:
            return self._head_commands()
        elif self.with_camera == 2:
            gpe = QtWidgets.QGroupBox("Head control", self)
            layout = QtWidgets.QVBoxLayout()
            lbl = QtWidgets.QLabel("Click on the image where you want the robot to look at")
            lbl.setWordWrap(1)
            layout.addWidget(lbl)
            layout.addWidget(ImagePositionGetter(self.robot, rate=500))
            gpe.setLayout(layout)
            gpe.setMinimumHeight(350)

            return gpe


    def _create_arm_box(self):
        """
        """
        gpe = QtWidgets.QGroupBox("Arm control", self)
        layout = QtWidgets.QVBoxLayout()

        # Left arm
        btn = QtWidgets.QPushButton("Block left arm", self)
        btn.clicked.connect(lambda: self.robot.set_stiffnesses("LArm", 1.0))
        layout.addWidget(btn)

        btn = QtWidgets.QPushButton("Release left arm", self)
        btn.clicked.connect(lambda: self.robot.set_stiffnesses("LArm", 0.0))
        layout.addWidget(btn)

        # Right arm
        btn = QtWidgets.QPushButton("Block right arm", self)
        btn.clicked.connect(lambda: self.robot.set_stiffnesses("RArm", 1.0))
        layout.addWidget(btn)

        btn = QtWidgets.QPushButton("Release right arm", self)
        btn.clicked.connect(lambda: self.robot.set_stiffnesses("RArm", 0.0))
        layout.addWidget(btn)

        layout.addWidget(self._raise_both_hands_btn())
        layout.addWidget(self._hold_glass_btn())

        gpe.setLayout(layout)
        return gpe


    def _head_commands(self):
        """
        """
        head_group = QtWidgets.QGroupBox("Head control", self)
        head_angles = QtWidgets.QGridLayout()
        up_btn = QtWidgets.QPushButton("^", self)
        up_btn.clicked.connect(lambda: self.robot.move_joint(["HeadPitch"], [-5], [0.7]))
        down_btn = QtWidgets.QPushButton("V", self)
        down_btn.clicked.connect(lambda: self.robot.move_joint(["HeadPitch"], [5], [0.7]))
        right_btn = QtWidgets.QPushButton(">", self)
        right_btn.clicked.connect(lambda: self.robot.move_joint(["HeadYaw"], [-10], [0.5]))
        right2_btn = QtWidgets.QPushButton(">>", self)
        right2_btn.clicked.connect(lambda: self.robot.move_joint(["HeadYaw"], [-20], [0.5]))
        left_btn = QtWidgets.QPushButton("<", self)
        left_btn.clicked.connect(lambda: self.robot.move_joint(["HeadYaw"], [10], [0.5]))
        left2_btn = QtWidgets.QPushButton("<<", self)
        left2_btn.clicked.connect(lambda: self.robot.move_joint(["HeadYaw"], [20], [0.5]))
        nod_btn = QtWidgets.QPushButton("Nod", self)
        nod_btn.clicked.connect(self.robot.nod)
        head_angles.addWidget(up_btn, 0, 2)
        head_angles.addWidget(nod_btn, 1, 2)
        head_angles.addWidget(left2_btn, 2, 0)
        head_angles.addWidget(left_btn, 2, 1)
        head_angles.addWidget(down_btn, 2, 2)
        head_angles.addWidget(right_btn, 2, 3)
        head_angles.addWidget(right2_btn, 2, 4)
        head_group.setLayout(head_angles)

        return head_group


    def _create_tablet_box(self):
        """Return a group box with all general actions"""
        gpe = QtWidgets.QGroupBox("Tablet")
        layout = QtWidgets.QVBoxLayout()

        idiap_image = "http://198.18.0.1/apps/idiap/idiap-1600.png"
        btn = self._show_image_btn("Idiap", idiap_image)
        layout.addWidget(btn)

        idiap_image = "http://198.18.0.1/apps/idiap/idiap-qr.png"
        btn = self._show_image_btn("Idiap QR", idiap_image)
        layout.addWidget(btn)

        black_image = "http://198.18.0.1/apps/idiap/black.png"
        btn = self._show_image_btn("Black", black_image)
        btn.setStyleSheet("background-color: black; color: white;")
        layout.addWidget(btn)

        green_image = "http://198.18.0.1/apps/idiap/green.png"
        btn = self._show_image_btn("Green", green_image)
        btn.setStyleSheet("background-color: green; color: white;")
        layout.addWidget(btn)

        # Empty web page with <h1 id="content">
        empty_page = "http://198.18.0.1/apps/idiap/empty.html"
        btn = QtWidgets.QPushButton("Empty web page", self)
        btn.clicked.connect(lambda: self.robot.load_url(empty_page))
        layout.addWidget(btn)

        edit = QtWidgets.QLineEdit("Print me on the tablet")
        edit.returnPressed.connect(
            lambda:
            self.robot.update_content_url_page(str(edit.text()), "content"))
        layout.addWidget(edit)

        gpe.setLayout(layout)

        return gpe


    def _create_speech_box(self):
        """Return a group box with all general actions"""
        gpe = QtWidgets.QGroupBox("Speech")
        layout = QtWidgets.QHBoxLayout()

        # Say versus animation
        btn = QtWidgets.QCheckBox("Animation")
        btn.setChecked(False)
        layout.addWidget(btn)

        # Add combo box to select language
        combo = QtWidgets.QComboBox()
        for l in self.robot.get_available_languages():
            combo.addItem(l)
        combo.activated[str].connect(
            lambda: self.robot.set_language(str(combo.currentText())))
        self.robot.set_language(str(combo.currentText()))
        layout.addWidget(combo)

        # Text area
        edit = QtWidgets.QLineEdit("Pepper")
        edit.returnPressed.connect(
            lambda:
            [
                self.robot.animated_speech(str(edit.text()))
                if btn.isChecked()
                else self.robot.say(str(edit.text())),
                edit.setText("")
            ])
        layout.addWidget(edit)

        gpe.setLayout(layout)
        return gpe


    def _create_pointing_box(self):
        """Return a group box with all general actions"""
        gpe = QtWidgets.QGroupBox("Pointing")
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self._show_on_the_right_btn())
        layout.addWidget(self._show_on_the_left_btn())
        layout.addWidget(self._show_behind_btn())
        layout.addWidget(self._photo_pose_btn())
        gpe.setLayout(layout)
        return gpe


    def _create_action_box(self):
        """Return a group box with all general actions"""
        gpe = QtWidgets.QGroupBox("General")
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self._wake_up_btn())
        layout.addWidget(self._rest_btn())
        layout.addWidget(self._stop_altracker_btn())
        layout.addWidget(self._bow_btn())
        layout.addWidget(self._stand_init_fast_btn())
        layout.addWidget(self._nod_btn())
        layout.addWidget(self._center_body_with_head_btn())
        # layout.addWidget(self._thread_btn())
        gpe.setLayout(layout)
        return gpe


    def _create_quit_box(self):
        """Return a group of quit and interrupt speech"""
        gpe = QtWidgets.QGroupBox("Quit")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self._interrupt_btn())
        layout.addWidget(self._quit_btn())
        gpe.setLayout(layout)
        return gpe


    def _create_gui(self):
        """
        """
        self.setWindowTitle("Robot")
        self.grid = QtWidgets.QGridLayout()

        # Main layout
        hbox = QtWidgets.QGridLayout(self)

        hbox.addWidget(self._create_speech_box(),
                       self.row_id, self.col_id,
                       1, self.max_nb_columns)
        self._increment_indices()

        self.row_id += 1
        self.col_id = 0

        # hbox.addLayout(command_box, 0, 0, 1, 1)
        # if self.no_camera:
        #     hbox.addWidget(self._head_commands(), 0, 1, 1, 1)
        # else:
        #     hbox.addWidget(ImagePositionGetter(self.robot), 0, 1, 1, 1)
        hbox.addWidget(self._create_action_box(), self.row_id, self.col_id)
        self._increment_indices()

        if self.with_camera > 0:
            hbox.addWidget(self._create_head_control_box(),
                           self.row_id,
                           self.col_id)
            self._increment_indices()

        hbox.addWidget(self._create_arm_box(), self.row_id, self.col_id)
        self._increment_indices()

        hbox.addWidget(self._create_pointing_box(), self.row_id, self.col_id)
        self._increment_indices()

        hbox.addWidget(self._create_tablet_box(), self.row_id, self.col_id)
        self._increment_indices()

        self.row_id += 1
        self.col_id = 0
        hbox.addWidget(self._create_quit_box(), self.row_id, self.col_id)
        self._increment_indices()


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
    parser.add_argument('--with-camera',
                        type=int,
                        default=0,
                        help="Head control: 0: None, 1: button, 2: image")
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
    parser.add_argument("-c",
                        type=int,
                        default=4,
                        help="Number of columns of the grid layout")


    opts = parser.parse_args()


    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(message)s",
                        datefmt="%H:%M:%S")

    app = QtWidgets.QApplication(sys.argv)
    gui = QiInterfaceBase(opts.ip, opts.port,
                          opts.with_camera, opts.no_connect,
                          x = opts.x, y = opts.y, columns = opts.c)
    gui.show()
    sys.exit(app.exec_())
