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


class QiInterfaceWizardOfOzDepth(QiInterfaceBase):
    def __init__(self,
                 ip,
                 port,
                 no_camera = False,
                 no_connect = False):

        self.grid = QtGui.QGridLayout()

        super(type(self), self).__init__(ip,
                                         port,
                                         no_camera=True,
                                         no_connect=False)

        self.setLayout(self.grid)


    def _create_where_to_go(self):
        box = QtGui.QGroupBox("Where to go")
        vbox = QtGui.QVBoxLayout()

        s = "Where would you like to go?"
        btn = QtGui.QPushButton(s)
        btn.clicked.connect(lambda state, s=s: self.robot.say(s))
        vbox.addWidget(btn)

        btn = QtGui.QPushButton("Display destinations")
        url = "http://198.18.0.1/apps/idiap/where-to-go.html"
        btn.clicked.connect(lambda state, url=url: self.robot.load_url(url))
        vbox.addWidget(btn)

        s = "Does your friend agree with that?"
        btn = QtGui.QPushButton(s)
        btn.clicked.connect(lambda state, s=s: self.robot.say(s))
        vbox.addWidget(btn)

        box.setLayout(vbox)
        return box


    def _create_discussion(self):
        gpe = QtGui.QGroupBox("Discussion")
        layout = QtGui.QVBoxLayout()

        questions = [
            ("Are you afraid of A I?", ""),
            ("Are you afraid of robots?", ""),
            ("Do you think that the United States will declare war to North Korea?", "USA NK"),
            ("What do you like at e-diape ?", "What do you like at Idiap?"),
            ("What do you enjoy in your home country?", ""),
            ("Can you tell me what back propagation is?", ""),
            ("Why should you not train on the test set?", ""),
            ("What is a random variable?", ""),
            ("Are you sure?", ""),
            ("How many cantons are there in Switzerland?", ""),
            ("How many do you know?", ""),
        ]
        for (q,d) in questions:
            layout.addWidget(self._say_btn(q, d if len(d) > 0 else q))

        # layout.addWidget(self._say_btn("Are you afraid of A I ?"))
        # layout.addWidget(self._say_btn("Are you afraid of robots ?"))
        # layout.addWidget(self._say_btn("Do you think that the United States will declare war to North Korea ?", "USA, Korea"))

        gpe.setLayout(layout)
        return gpe


    def _create_performance(self):
        """Buttons to ask people to move in the room"""
        gpe = QtGui.QGroupBox("Performance")
        layout = QtGui.QVBoxLayout()

        s = "Hi I'm pepper, choose your performance today"
        btn = QtGui.QPushButton(s)
        btn.clicked.connect(lambda state, s=s: self.robot.say(s))
        layout.addWidget(btn)

        btn = QtGui.QPushButton("Display performance")
        url = "http://198.18.0.1/apps/idiap/{}.html".format("performance")
        btn.clicked.connect(lambda state, url=url: self.robot.load_url(url))
        layout.addWidget(btn)

        gpe.setLayout(layout)
        return gpe


    def _create_quizz_box(self):
        """Create buttons to display quizz"""
        gpe = QtGui.QGroupBox("Quizz", self)
        layout = QtGui.QVBoxLayout()

        for s in ["god", "orwell", "ouagadougou", "perfect-number"]:
            url = "http://198.18.0.1/apps/idiap/{}.html".format(s)
            btn = QtGui.QPushButton(s, self)
            btn.clicked.connect(lambda state, url=url: self.robot.load_url(url))
            layout.addWidget(btn)

        gpe.setLayout(layout)
        return gpe


    def _create_gui(self):
        """
        """
        self.setWindowTitle("MuMMER depth data set")

        self.grid.addWidget(self._create_quizz_box(), self.row_id, self.col_id)
        self._increment_indices()

        self.grid.addWidget(self._create_where_to_go(), self.row_id, self.col_id,)
        self._increment_indices()

        self.grid.addWidget(self._create_performance(), self.row_id, self.col_id,)
        self._increment_indices()

        self.grid.addWidget(self._create_discussion(), self.row_id, self.col_id,)
        self._increment_indices()



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

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(message)s",
                        datefmt="%H:%M:%S")

    opts = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(message)s",
                        datefmt="%H:%M:%S")

    app = QtGui.QApplication(sys.argv)
    gui = QiInterfaceWizardOfOzDepth(opts.ip, opts.port)
    gui.show()
    sys.exit(app.exec_())
