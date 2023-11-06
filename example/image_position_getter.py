# coding: utf-8

from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

import almath
import cv2


class ImagePositionGetter(QtWidgets.QGraphicsView):
    """Display an image and return position of mouth on image when image
       is clicked

    """
    position = QtCore.pyqtSignal(tuple)

    def __init__(self, robot, rate=500, parent=None):
        """
        """
        super(type(self), self).__init__(parent)
        self.width = 320
        self.height = 240
        self.setFixedSize(self.width+10, self.height+10)
        self.image = QtGui.QImage()
        self.robot = robot

        self.pixMapItem = QtWidgets.QGraphicsPixmapItem(QtGui.QPixmap(self.image), None)
        self.pixMapItem.mousePressEvent = self.pixelSelect

        self.setScene(QtWidgets.QGraphicsScene(self))
        self.scene().addItem(self.pixMapItem)

        self.cv2_image = None
        self.startTimer(rate)


    def pixelSelect(self, event):
        """
        """
        height = self.cv2_image.shape[0]
        width = self.cv2_image.shape[1]

        x = float(event.pos().x())
        y = float(event.pos().y())
        x = x - width/2
        y = y - height/2

        cam = self.robot.alvideo.getActiveCamera()
        hfov = self.robot.alvideo.getHorizontalFOV(cam)
        vfov = self.robot.alvideo.getVerticalFOV(cam)
        yaw_to_move = x/width*hfov
        pitch_to_move = y/height*vfov

        # "Head" will be interpreted as ["HeadYaw", "HeadPitch"]
        names  = ["HeadYaw", "HeadPitch"]
        angles = [-yaw_to_move*almath.TO_DEG, pitch_to_move*almath.TO_DEG]
        speed = [1.0, 1.0]
        self.robot.move_joint(names, angles, speed)

        # To move the robot from the wheel
        # angle2 = -yaw_to_move*almath.TO_DEG / 2.0
        # angles = [angle2, pitch_to_move*almath.TO_DEG]
        # # print("Move {}".format(angle2))

        # self.robot.move_joint(names, angles, speed)
        # self.robot.move_to(0, 0, angle2)


    def timerEvent(self, event):
        """Called periodically. Retrieve a nao image, and update the widget."""
        cv2_image = self.robot.get_image()
        cv2_image = cv2.resize(cv2_image, (self.width, self.height))
        self.cv2_image = cv2_image
        self.image = QtGui.QImage(cv2_image.data,
                                  cv2_image.shape[1],
                                  cv2_image.shape[0],
                                  QtGui.QImage.Format_RGB888)

        self.pixMapItem.setPixmap(QtGui.QPixmap(self.image))
        self.update()
