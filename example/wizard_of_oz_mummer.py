from __future__ import print_function
import os
import sys
import time
import argparse

from PyQt4 import QtGui
from PyQt4 import QtCore

import qi

from gui import QiInterfaceBase
from utils import animations
from image_position_getter import ImagePositionGetter

if sys.version_info.major == 2:
    import ConfigParser as configparser
else:
    import configparser as configparser

import logging
import pprint


class QiInterfaceWizardOfOz(QiInterfaceBase):
    def __init__(self,
                 ip,
                 port,
                 scenario_file_name,
                 qcm_file_name = "",
                 columns = 4,
                 no_camera = False,
                 no_connect = False,
                 x = -1, y = -1):

        self.lang = "en"

        self.scenario_file_name = scenario_file_name
        self.qcm_file_name = qcm_file_name

        self.use_anim = QtGui.QCheckBox("Animation")
        self.use_anim.setChecked(False)

        # Whether to call stand_init_faster after speaking
        self.init_after_speak = QtGui.QCheckBox("Stand init after speak")
        self.init_after_speak.setChecked(False)

        self.groups = []

        super(type(self), self).__init__(ip,
                                         port,
                                         with_camera=0,
                                         no_connect=no_connect,
                                         columns=columns,
                                         x=x, y=y)

        self.robot.set_language("en")
        self.robot.set_default_speed(80)


    def _create_misc_buttons(self):
        """"""
        gpe = QtGui.QGroupBox("Speech")
        layout = QtGui.QVBoxLayout()

        # The check box for using animation or not
        layout.addWidget(self.use_anim)
        layout.addWidget(self.init_after_speak)

        # Add combo box to select language
        self.combo = QtGui.QComboBox()
        for l in self.robot.get_available_languages():
            self.combo.addItem(l)
        self.combo.addItem("French")
        self.combo.activated[str].connect(
            lambda: self.robot.set_language(str(self.combo.currentText())))
        self.robot.set_language(str(self.combo.currentText()))
        layout.addWidget(self.combo)

        # The image of street light for satisfaction
        image = "http://198.18.0.1/apps/idiap/satisfactory-study.png"
        btn = self._show_image_btn("Satisfaction", image)
        layout.addWidget(btn)

        qr_image = "http://198.18.0.1/apps/idiap/idiap-qr.png"
        btn = self._show_image_btn("QR", qr_image)
        layout.addWidget(btn)

        # A black image
        black_image = "http://198.18.0.1/apps/idiap/black.png"
        btn = self._show_image_btn("Black", black_image)
        btn.setStyleSheet("background-color: black; color: white;")
        layout.addWidget(btn)

        # Pointing buttons
        layout.addWidget(self._show_on_the_right_btn())
        layout.addWidget(self._show_on_the_left_btn())
        layout.addWidget(self._show_on_the_left_below_btn())
        layout.addWidget(self._show_behind_btn())
        layout.addWidget(self._stand_init_fast_btn())
        layout.addWidget(self._nod_btn())

        gpe.setLayout(layout)
        return gpe


    def _add_say_button(self, speech, name=""):
        """Return a button to trigger a speech sentence. If name is empty, use
        the speech as name

        """
        if len(name) == 0:
            name = speech
        btn = QtGui.QPushButton(name)
        btn.clicked.connect(lambda: self._speak(speech))
        return btn


    def _load_scenario_file(self, file_name):
        """ """
        config = configparser.ConfigParser()
        config.optionxform = str
        config.read(file_name)
        for section in config.sections():
            vbox = QtGui.QVBoxLayout()
            for (key, val) in config.items(section):
                vbox.addWidget(self._add_say_button(val, key))
            gpe = QtGui.QGroupBox(section)
            gpe.setLayout(vbox)
            self.groups.append(gpe)
            # self.grid.addWidget(box, self.row_id, self.col_id)
            # self._increment_indices()


    def _speak(self, text):
        """Say"""
        lang = self.combo.currentText()

        if self.use_anim.isChecked():
            self.robot.animated_speech(text)
            if self.init_after_speak.isChecked():
                self.robot.stand_init_fast()
        else:
            words0 = text.split()
            words1 = []
            # Remove tags ^start, ^wait, etc.
            for w in words0:
                if len(w) > 0:
                    if w[0] != "^":
                        words1.append(w)

            processed_text = " ".join(words1)
            self.robot.say(processed_text)


    def _load_qcm_file(self, file_name):
        """"""
        config = configparser.ConfigParser()
        config.optionxform = str
        config.read(file_name)
        gpe = QtGui.QGroupBox("QCM", self)
        # layout = QtGui.QVBoxLayout()
        layout = QtGui.QGridLayout()
        col = 0
        row = 0
        col_max = 2
        for section in config.sections():
            btn = QtGui.QPushButton(section, self)
            btn.setCheckable(1)

            q_and_a = {}
            for (key, val) in config.items(section):
                q_and_a[key] = val

            btn.toggled.connect(lambda checked, q=q_and_a: [
                self.robot.display_qcm_question(q),
                time.sleep(0.2),
                self.robot.say(q["question"]),
                time.sleep(0.5),
                self.robot.say("{}".format(q["answer1"])),
                time.sleep(0.5),
                self.robot.say("{}".format(q["answer2"])),
                time.sleep(0.5),
                self.robot.say("{}".format(q["answer3"])),
                time.sleep(0.5),
                self.robot.say("or... {}".format(q["answer4"])),
                time.sleep(0.5)
            ] if checked else [
                self.robot.display_qcm_question(q, display_answer=1),
                time.sleep(0.2),
                self.robot.say("The correct answer is,"),
                time.sleep(0.2),
                self.robot.say(q[q["answer"]]),
            ])

            layout.addWidget(btn, row, col)
            col+=1
            if col == col_max:
                row += 1
                col = 0
        gpe.setLayout(layout)

        self.groups.append(gpe)

        # self.grid.addWidget(gpe, self.row_id, self.col_id)


    def _create_gui(self):
        """
        """
        self.setWindowTitle("MuMMER Wizard-of-Oz")

        self.grid = QtGui.QGridLayout()

        self.groups.append(self._create_misc_buttons())

        # self._increment_indices()
        # self.grid.addWidget(self._stand_init_fast_btn(),
        #                     self.row_id, self.col_id)

        if len(self.scenario_file_name) > 0:
            self.row_id += 1
            self.col_id = 0
            self._load_scenario_file(self.scenario_file_name)

        if len(self.qcm_file_name) > 0:
            self.row_id += 1
            self.col_id = 0
            self._load_qcm_file(self.qcm_file_name)

        for gpe in self.groups:
            gpe.setMaximumHeight(400)
            self.grid.addWidget(gpe, self.row_id, self.col_id)
            self._increment_indices()

        self.setLayout(self.grid)


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
    parser.add_argument("--scenarios",
                        type=str,
                        default="",
                        help="Scenario file .ini")
    parser.add_argument("--qcm",
                        type=str,
                        default="",
                        help="Questions file for QCM .ini")
    parser.add_argument("-x",
                        type=int,
                        default=-1,
                        help="Position of the window")
    parser.add_argument("-y",
                        type=int,
                        default=-1,
                        help="Position of the window")
    parser.add_argument('--no-connect',
                        action='store_true',
                        help="Don't connect to robot")
    parser.add_argument("-c", "--columns",
                        type=int,
                        default=5,
                        help="Number of columns in interface")

    opts = parser.parse_args()

    logging.basicConfig(format="[%(name)s] %(message)s",
                        level=logging.INFO)

    app = QtGui.QApplication(sys.argv)
    gui = QiInterfaceWizardOfOz(opts.ip,
                                opts.port,
                                opts.scenarios,
                                opts.qcm,
                                columns = opts.columns,
                                x = opts.x,
                                y = opts.y,
                                no_connect = opts.no_connect)
    gui.show()
    sys.exit(app.exec_())
