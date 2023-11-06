# coding: utf-8

######################################################################
# Copyright (c) 2019 Idiap Research Institute <http://www.idiap.ch/>
######################################################################

from __future__ import print_function

import os
import sys
import time
import logging
import argparse

from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

import qi

from image_position_getter import ImagePositionGetter
from utils import animations
from robot import Robot

logger = logging.getLogger("wizard")
USB_SERVER = "http://198.18.0.1/apps/idiap"

# When switching from image view to web view, the tablet needs time
# int between before actually showing the new content. This delay
# prevent from pressing twice the button to make it work
TABLET_DELAY = 0.5 # second

class RobotRunnable(QtCore.QRunnable):
    """Interface to call all functions from the robot in a separate thread"""

    next_task_requested = QtCore.pyqtSignal(str)

    def __init__(self, robot, func, *args, **kwargs):
        super(type(self), self).__init__()
        self.robot = robot
        self.func = func
        # self.next_task = kwargs.pop("next_task", None)
        self.args = args
        self.kwargs = kwargs

    def run(self):
        logger.debug("Call {} with args {} kwargs {}".
                     format(self.func, self.args, self.kwargs))
        f = getattr(self.robot, self.func)
        try:
            f(*self.args, **self.kwargs)
        except Exception as e:
            logger.error("Cannot run '{}'".format(self.func))


class QiInterfaceBase(QtWidgets.QWidget):
    """Base interface to control a NAOqi robot

    Args:
        ip:
        port:
        with_camera:
    """
    def __init__(self, ip, port,
                 with_camera=0,
                 no_connect=False,
                 n_columns=3,
                 image_rate=200,
                 x=-1,
                 y=-1):
        """
        """
        super(QiInterfaceBase, self).__init__()

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+W"), self, self.close)
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.close)

        this_dir = os.path.dirname(os.path.realpath(__file__))
        icon_path = os.path.join(this_dir, "..", "images", "icon.png")
        self.setWindowIcon(QtGui.QIcon(icon_path))

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
        self.max_nb_columns = n_columns

        # To rotate the robot
        self.dial = QtWidgets.QDial()
        self.dial.setRange(-180, 180)
        self.dial.setNotchesVisible(True)
        self.dial.setValue(0)
        self.dial.sliderReleased.connect(self._dial_released)

        self.head_controller = ImagePositionGetter(self.robot, rate=image_rate)
        self.head_controller.position.connect(self.turn_head)

        # Some global buttons that need to be added in the GUI
        # somewhere
        self.anim_checkbox = QtWidgets.QCheckBox("Use Animated Speech")
        self.anim_checkbox.setChecked(False)

        self.lang_combobox = QtWidgets.QComboBox()
        for l in self.robot.get_available_languages():
            self.lang_combobox.addItem(l)
        self.robot.set_language(str(self.lang_combobox.currentText()))
        self.lang_combobox.activated[str].connect(
            lambda:
            self.robot.set_language(str(self.lang_combobox.currentText())))

        self.speed_spinbox = QtWidgets.QSpinBox(self)
        self.speed_spinbox.setRange(50, 200)
        self.speed_spinbox.setValue(100)
        self.speed_spinbox.valueChanged.connect(
            lambda:
            self.robot.set_speed(self.speed_spinbox.value()))
        self.robot.set_speed(self.speed_spinbox.value())

        # Indicate what type of application last accessed the tablet
        self.tablet_mode = ""

        # Text area to say some text
        self.say_lineedit = QtWidgets.QLineEdit()
        self.say_lineedit.setPlaceholderText("Your text...")
        self.say_lineedit.returnPressed.connect(lambda: [
            self._say_from_lineedit(),
            self.say_lineedit.setText(""),
        ])

        # Text area to display text on the tablet
        self.print_lineedit = QtWidgets.QLineEdit()
        self.print_lineedit.setPlaceholderText("Write on tablet...")
        self.print_lineedit.returnPressed.connect(lambda: [
            self.print_on_tablet(), self.print_lineedit.setText(""), ])


        self._create_gui()

        if not (x < 0 and y < 0):
            self.move(x, y) # Move the window, not the robot...

    def _connect_to_robot(self):
        """
        """
        try:
            print("Connecting to robot {}:{}".format(self.ip, self.port))
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            logger.error("Cannot connect ip {} on port {}".
                         format(self.ip, self.port))

    def execute(self, name, *args, **kwargs):
        """Call function robot.name()"""
        qrun = RobotRunnable(self.robot, name, *args, **kwargs)
        QtCore.QThreadPool.globalInstance().start(qrun)

    def _increment_indices(self):
        """Increment row_id and col_id"""
        self.col_id += 1
        if self.col_id == self.max_nb_columns:
            self.col_id = 0
            self.row_id += 1

    def _dial_released(self):
        """"""
        angle = self.dial.value()
        # Minus the angle to match trigo
        self.execute("move_to", x=0, y=0, theta=-angle)
        time.sleep(1.0)
        self.dial.setValue(0)

    @QtCore.pyqtSlot(tuple)
    def turn_head(self, t):
        names  = ["HeadYaw", "HeadPitch"]
        angles = list(t)
        speed = [1.0, 1.0]
        self.execute("move_joint",
                     joint_name=names,
                     angle_in_degree=angles,
                     time_in_sec=speed)

    def _quit_btn(self):
        """
        """
        btn = QtWidgets.QPushButton("Quit", self)
        btn.clicked.connect(exit)
        return btn


    def _say_from_lineedit(self):
        """Call animated_speech if box anim_checkbox is checked, say
        otherwise.

        """
        text = unicode(self.say_lineedit.text()).encode("utf-8")
        function_name = "say"
        if self.anim_checkbox.isChecked():
            function_name = "animated_speech"
        self.execute(function_name, text)

        # self.execute(init)

    def _set_tablet_mode(self, mode):
        self.tablet_mode = mode

    def print_on_tablet(self):
        """Either print text on tablet or open URL (if text staring by http or
        www.)

        """
        tablet_mode = "print"
        text = self.print_lineedit.text()
        logger.info("Print '{}'".format(text))

        if text[0:4] in ["http", "www."]:
            if text[0:4] == "www.":
                text = "https://{}".format(text)
            self.robot.load_url(text)
            time.sleep(TABLET_DELAY)
            self.robot.load_url(text)
        else:
            self.robot.load_url(os.path.join(USB_SERVER, "empty.html"))
            time.sleep(TABLET_DELAY)
            self.robot.update_content_url_page(text, "content")

        self._set_tablet_mode(tablet_mode)

    def _show_image_btn(self,
                        button_name,
                        image_name,
                        path=USB_SERVER,
                        style_sheet=""):
        """Button to show an image

        Args:
            image_name: Absolute name (url) or relative to next argument 'path'
            path: Used only if image name is not a full path

        Example:

            You can use

            image_name = http://www.mummer-project.eu/media/media_462687_en.jpg
            path = ""

            or

            image_name = image.png
            path = http://198.18.0.1/apps/YOUR_OWN_APP

            in which case image.png should be in

            /home/nao/.local/share/PackageManager/apps/YOUR_OWN_APP/html

        """
        btn = QtWidgets.QPushButton(button_name, self)
        if style_sheet:
            btn.setStyleSheet(style_sheet)
        image_path = image_name
        if (image_path[0:4] != "http") and (image_path[0:4] != "www."):
            image_path = os.path.join(path, image_name)
        btn.clicked.connect(lambda: [
            self._set_tablet_mode("image"),
            time.sleep(TABLET_DELAY),
            self.execute("show_image",
                         image_file_on_robot=image_path)])
        return btn

    def _create_robot_button(self, name, function_name):
        """Create a simple push button which calls a robot 'function_name'

        Args:
            name: name to be displayed on the button
            function_name: function from "Robot" to be called

        Example:

            To call function "wake_up":

            _create_button("Wake up", "wake_up"):

        """
        btn = QtWidgets.QPushButton(name, self)
        btn.clicked.connect(lambda: self.execute(function_name))
        return btn

    def _create_robot_button_args(self, name, function_name, *args, **kwargs):
        """Create a simple push button which calls a robot 'function_name'

        Args:
            name: name to be displayed on the button
            function_name: function from "Robot" to be called
            args: to be passed to self.execute
            kwargs: to be passed to self.execute

        Example:

            To call function "wake_up":

            _create_button("Wake up", "wake_up"):

        """
        btn = QtWidgets.QPushButton(name, self)
        btn.clicked.connect(lambda: self.execute(function_name, args, kwargs))
        return btn

    def _create_general_box(self):
        gpe = QtWidgets.QGroupBox("General", self)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self._create_robot_button("Wake up", "wake_up"))
        layout.addWidget(self._create_robot_button("Rest", "rest"))
        layout.addWidget(self._create_robot_button("Disable tracker", "disable_altraker"))
        layout.addWidget(self._create_robot_button("Enable tracker", "enable_altraker"))
        layout.addWidget(self._create_robot_button("Disable breathing", "disable_breathing"))
        layout.addWidget(self._create_robot_button("Enable breathing", "enable_breathing"))
        layout.addWidget(self._create_robot_button("Stand init", "stand_init_fast"))
        # layout.addWidget(self._enable_breathing_btn())
        # layout.addWidget(self._disable_breathing_btn())
        # layout.addWidget(self._stand_init_btn())
        # layout.addWidget(self._interruption_btn())
        layout.addWidget(self._quit_btn())
        gpe.setLayout(layout)
        return gpe

    def _create_head_box(self):
        gpe = QtWidgets.QGroupBox("Head", self)
        layout = QtWidgets.QVBoxLayout()
        lbl = "Click where you want\nthe robot to look at"
        layout.addWidget(QtWidgets.QLabel(lbl))
        layout.addWidget(self.head_controller)
        gpe.setLayout(layout)
        return gpe

    def _create_tablet_box(self):
        gpe = QtWidgets.QGroupBox("Tablet", self)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.print_lineedit)

        layout.addWidget(self._show_image_btn("Idiap", "idiap-1600.png"))
        layout.addWidget(self._show_image_btn("MuMMER", "mummer-1600.png"))

        layout.addWidget(self._show_image_btn(
            "Black", "black.png",
            style_sheet="background-color: black; color: white;"))

        layout.addWidget(self._show_image_btn(
            "Green", "green.png",
            style_sheet="background-color: green; color: white;"))

        gpe.setLayout(layout)
        return gpe

    # def _create_tablet_box(self):
    #     gpe = QtWidgets.QGroupBox("Tablet", self)
    #     layout = QtWidgets.QVBoxLayout()
    #     layout.addWidget(self.print_lineedit)

    #     idiap_image = "http://198.18.0.1/apps/idiap/idiap-1600.png"
    #     btn = self._show_image_btn("Idiap", idiap_image)
    #     layout.addWidget(btn)

    #     idiap_image = "http://198.18.0.1/apps/idiap/idiap-qr.png"
    #     btn = self._show_image_btn("Idiap QR", idiap_image)
    #     layout.addWidget(btn)

    #     black_image = "http://198.18.0.1/apps/idiap/black.png"
    #     btn = self._show_image_btn("Black", black_image)
    #     btn.setStyleSheet("background-color: black; color: white;")
    #     layout.addWidget(btn)

    #     green_image = "http://198.18.0.1/apps/idiap/green.png"
    #     btn = self._show_image_btn("Green", green_image)
    #     btn.setStyleSheet("background-color: green; color: white;")
    #     layout.addWidget(btn)

    #     # Empty web page with <h1 id="content">
    #     empty_page = "http://198.18.0.1/apps/idiap/empty.html"
    #     btn = QtWidgets.QPushButton("Empty web page", self)
    #     btn.clicked.connect(lambda: self.robot.load_url(empty_page))
    #     layout.addWidget(btn)

    #     edit = QtWidgets.QLineEdit("Print me on the tablet")
    #     edit.returnPressed.connect(
    #         lambda:
    #         self.robot.update_content_url_page(str(edit.text()), "content"))
    #     layout.addWidget(edit)

    #     gpe.setLayout(layout)

    #     return gpe

    def _create_speech_box(self):
        gpe = QtWidgets.QGroupBox("Speech", self)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.anim_checkbox)
        layout.addStretch(1)
        layout.addWidget(QtWidgets.QLabel("Speed of speech"))
        layout.addWidget(self.speed_spinbox)
        layout.addStretch(1)
        layout.addWidget(QtWidgets.QLabel("Language"))
        layout.addWidget(self.lang_combobox)
        layout.addStretch(1)
        layout.addWidget(self.say_lineedit)
        gpe.setLayout(layout)
        return gpe

    def _create_mvt_box(self):
        gpe = QtWidgets.QGroupBox("Movement", self)
        layout = QtWidgets.QVBoxLayout()
        lbl = "Click to turn the robot"
        layout.addWidget(QtWidgets.QLabel(lbl))
        layout.addWidget(self.dial)
        layout.addWidget(self._create_robot_button("Center body", "center_body_with_head"))
        gpe.setLayout(layout)
        return gpe

    def _create_gui(self):
        """Add all the widgets"""
        layout = QtWidgets.QGridLayout(self)

        layout.addWidget(self._create_general_box(), self.row_id, self.col_id)
        self._increment_indices()
        layout.addWidget(self._create_mvt_box(), self.row_id, self.col_id)
        self._increment_indices()
        layout.addWidget(self._create_head_box(), self.row_id, self.col_id)
        self._increment_indices()
        layout.addWidget(self._create_speech_box(), self.row_id, self.col_id)
        self._increment_indices()
        layout.addWidget(self._create_tablet_box(), self.row_id, self.col_id)
        self._increment_indices()

        self.setLayout(layout)

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
    parser.add_argument("-n", "--nb-columns",
                        type=int,
                        default=6,
                        help="Number of columns for the interface")
    parser.add_argument("--say",
                        type=str,
                        default="",
                        help="File .ini of buttons")
    parser.add_argument("--quiz",
                        type=str,
                        default="",
                        help="File .ini for quiz")
    parser.add_argument("--verbose",
                        type=int,
                        default=20,
                        help="Verbosity level (10, 20, 30)")

    try:
        args = parser.parse_args()
    except:
        parser.print_help()
        sys.exit(1);

    logging.basicConfig(format="[%(name)s] %(message)s", level=args.verbose)

    app = QtWidgets.QApplication(sys.argv)
    gui = QiInterfaceBase(ip=args.ip,
                          port=args.port,
                          n_columns=args.nb_columns)
    gui.show()
    sys.exit(app.exec_())
