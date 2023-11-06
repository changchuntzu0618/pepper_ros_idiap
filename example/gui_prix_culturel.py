import os
import sys
import time
import argparse

from PyQt4 import QtGui
from PyQt4 import QtCore

import qi
import logging

from gui import QiInterfaceBase
from utils import animations


class QiInterfacePrixCulturel(QiInterfaceBase):
    """GUI interface with tools only needed for Prix Culturel 2017"""

    def __init__(self, ip, port):
        """
        """
        super(type(self), self).__init__(ip, port, with_camera=False)


    def _mute_mouvement(self, time_ms, anim="explain1"):
        """Do some movements as if speaking for 'time_ms' and then goes back
        to StandInit position

        """
        s = '^start({' + anim + '}) \\pau=' + str(time_ms) + '\\ ^stop({' + anim + '}) ^pCall(ALRobotPosture.goToPosture("StandInit", 0.4))'
        print(s)
        self.robot.animated_speech(s.format(**animations))


    def _create_speech(self):
        """Create the 3 moments where Pepper speaks"""

        pwd = os.path.dirname(os.path.realpath(__file__))
        pwd = os.path.join(pwd, "..", "resources")

        layout = QtGui.QHBoxLayout()

        # Pepper present Idiap + Mali
        file_name = os.path.join(pwd, "1711-prix-culturels-presentation.txt")
        layout.addWidget(self._animated_speech_btn("Presentation",
                                                   file_name,
                                                   "French"))

        # Pepper thanks Mali
        file_name = os.path.join(pwd, "1711-prix-culturels-merci.txt")
        layout.addWidget(self._animated_speech_btn("Merci",
                                                   file_name,
                                                   "French"))

        # Pepper leaves the stage
        file_name = os.path.join(pwd, "1711-prix-culturels-sortie.txt")
        layout.addWidget(self._animated_speech_btn("Sortie",
                                                   file_name,
                                                   "French"))


        return layout


    def _create_mute_btn(self, name, time_ms, anim="explain1"):
        """Reutrn a button triggering _mute_mouvement"""
        btn = QtGui.QPushButton(name, self)
        btn.pressed.connect(lambda: self._mute_mouvement(time_ms, anim))
        return btn


    def _create_tablet(self):
        """Create buttons to show images on tablet"""

        # Group of buttons to show images on tablet
        group = QtGui.QGroupBox("Tablet control", self)
        layout = QtGui.QHBoxLayout()

        # Black screen
        black_image = "http://198.18.0.1/apps/idiap/black.png"
        mali_image  = "http://198.18.0.1/apps/idiap/mali-1280.jpg"

        layout.addWidget(self._show_image_btn("Black", black_image))
        layout.addWidget(self._show_image_btn("Mali", mali_image))

        group.setLayout(layout)

        return group


    def _create_mute_mouvements(self):
        """Create all the buttons
        """
        mvt_group = QtGui.QGroupBox("Mute movements", self)
        mvt_layout = QtGui.QVBoxLayout()

        text = "Mali van Valenberg est avec nous depuis Lausanne ou [...] Bonsoir Mali, comment allez-vous?"
        anim = "explain1"
        time_ms = 7000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("Bonsoir Pepper, ca va bien.", self))

        #
        text = "Pas trop le trac avant la representation de ce soir?"
        anim = "explain1"
        time_ms = 3000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("[...] J'ai toujours le trac, c'est important.", self))

        #
        text = "Une 1ere question que j'aimerais [...] Quel fut votre sentiment lorsque vous avez appris l'attribution de ce prix ?"
        anim = "explain2"
        time_ms = 7000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("C'est pas un prix [...] je considere pas que c'est quelque chose specialement pour moi", self))

        #
        text = "Petite, vous etiez fan de votre professeure de solfege, vous avez [...] pourquoi ce changement de cap?"
        anim = "explain1"
        time_ms = 12000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("[...] je me sens apporter qch plus qu'en musique", self))

        #
        text = "Puis, c'est la montee a Paris [...] Qu'est-ce que vous avez decouvert et appris dans la ville lumiere?"
        anim = "explain1"
        time_ms = 8000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("[...] rugueux, il faut, il faut se familiariser aussi la dedans", self))

        #
        text = "Apres ce sejour a Paris, [...] et qu'est-ce que cela veut dire pour vous?"
        anim = "explain1"
        time_ms = 10000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("[...] et de recreer ici sur un plateau, au cinema", self))

        #
        text = "Vous jouez non seulement sur [...] Alors vous etes plutot actrice ou comedienne?"
        anim = "explain1"
        time_ms = 12000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("[...] une energie plu en montagne russe qui est chouette a explorer", self))

        #
        text = "Une derniere question. Vous ne faites pas que jouer, [...] c'est juste par plaisir ou par defi?"
        anim = "explain1"
        time_ms = 8000
        mvt_layout.addWidget(self._create_mute_btn(text, time_ms, anim))
        mvt_layout.addWidget(QtGui.QLabel("... de rythme de sonnorite je m'amuse avant tout", self))

        mvt_group.setLayout(mvt_layout)

        return mvt_group


    def _create_gui(self):
        """
        """
        emmergency_box = QtGui.QHBoxLayout()
        emmergency_box.addWidget(self._interrupt_btn())
        emmergency_box.addWidget(self._quit_btn())


        # Main layout
        hbox = QtGui.QGridLayout(self)
        hbox.addLayout(self._create_speech(), 0, 0)
        hbox.addWidget(self._create_tablet(), 1, 0)
        hbox.addWidget(self._create_mute_mouvements(), 2, 0)
        hbox.addLayout(emmergency_box, 3, 0)

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

    opts = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(message)s",
                        datefmt="%H:%M:%S")

    app = QtGui.QApplication(sys.argv)
    gui = QiInterfacePrixCulturel(opts.ip, opts.port)
    gui.show()
    sys.exit(app.exec_())
