######################################################################
# Copyright (c) 2018 Idiap Research Institute <http://www.idiap.ch/>
######################################################################

from __future__ import print_function

import os
import sys
import time
import logging
import argparse


from pprint import pprint

from utils import *

import numpy as np
import cv2


import functools

import qi
import almath
import vision_definitions

logger = logging.getLogger(__name__)


class Robot:
    """Wrapper class around useful functions or procedures"""
    def __init__(self, session):
        """Constructor

        Args:
            session : qi.Session()

        Examples:
            >>> session = qi.Session()
            >>> session.connect("tcp://192.168.1.1:9559")
            >>> robot = Robot(session)

        """
        self.session = session

        if self.session.isConnected():
            self.alleds         = session.service("ALLeds")
            self.altts          = session.service("ALTextToSpeech")
            self.alvideo        = session.service("ALVideoDevice")
            self.almotion       = session.service("ALMotion")
            self.almemory       = session.service("ALMemory")
            self.altablet       = session.service("ALTabletService")
            self.alposture      = session.service("ALRobotPosture")
            self.alfacedet      = session.service("ALFaceDetection")
            self.alanimated     = session.service("ALAnimatedSpeech")
            self.alnotification = session.service("ALNotificationManager")

        # self.sub = self.almemory.subscriber("in_love")
        # self.sub.signal.connect(functools.partial(self._in_love))


    def robot_is_connected(self):
        """Return true if the robot is connected"""
        if not self.session.isConnected():
            logger.error("The robot is not connected")
            return False
        else:
            return True


    def move_to(self, x = 0.0, y = 0.0, theta = 0.0):
        """Move to

        Args:
            x     :
            y     :
            theta : In degree

        """
        if not self.robot_is_connected(): return

        logger.info("Moving to {} {} {}".format(x, y, theta))

        self.almotion.moveTo(x, y, theta*almath.TO_RAD)


    def set_language(self, language="fr"):
        """Set the language

        Args:
            language : string which can be "fr", "en", etc.

        """
        if not self.robot_is_connected(): return

        logger.info("Setting language to {}".format(language))

        if language.lower() in ["fr", "french", "francais"]:
            self.altts.setLanguage("French")
        elif language.lower() in ["en", "english"]:
            self.altts.setLanguage("English")
        else:
            self.altts.setLanguage(language)

    def disable_altraker(self):
        """Work around to stop tracker when the robot tracks on its own"""
        if not self.robot_is_connected(): return
        logger.info("Disabling face tracker")
        self.alfacedet.pause(1)

    def enable_altraker(self):
        """Should enable tracker again (not possible to test properly)"""
        if not self.robot_is_connected(): return
        logger.info("Enabling face tracker")
        self.alfacedet.pause(0)

    def enable_breathing(self, chain_name="Arms"):
        """Enablen the breathing"""
        if not self.robot_is_connected(): return
        logger.info("Enabling ")
        self.almotion.setBreathEnabled(chain_name, True)

    def disable_breathing(self, chain_name="Body"):
        """Disable all breathing"""
        if not self.robot_is_connected(): return
        self.almotion.setBreathEnabled(chain_name, False)

    def set_default_speed(self, speed):
        """Call setParameter("defaultVoiceSpeed", 90) from ALTextToSpeech

        Args:
            speed : Integer (100 normal)

        """
        if not self.robot_is_connected(): return
        logger.info("Set voice speed to {}".format(speed))
        self.altts.setParameter("defaultVoiceSpeed", speed)

    def set_speed(self, speed=100):
        """Set the speed

        Args:
            speed : Acceptable range is [50-400]
        """
        if not self.robot_is_connected(): return
        logger.info("Setting language speed to {}".format(speed))
        self.altts.setParameter("speed", speed)

    def say(self, string_text):
        """Say some string text without animation movement

        Args:
           string_text : The text to say as a string
           language    :

        """
        if not self.robot_is_connected(): return
        logger.info("Say")
        self.altts.say(string_text)


    def animated_speech(self,
                        formatted_text = "",
                        formatted_file = "",
                        configuration = {"bodyLanguageMode": "contextual"}):
        """Calls animated speech with input text or file.

        formatted_file and formatted_text are exclusive. Calls
        formatted_text if both are provided.

        Args:
           formatted_text : The text as a string
           formatted_file : The text in a file
           language       :
           configuration  :

        Examples:
           formatted_text = "^start({hey1}) Hi, I am pepper."

        """
        if not self.robot_is_connected(): return

        logger.info("Animated speech")

        # By default use formatted_text
        speech = formatted_text

        # If a formatted_file is provided, use it
        if len(formatted_file) > 0:
            with open(formatted_file, "r") as fd:
                speech = fd.read().replace("\n", " ")

        speech = speech.format(**animations)

        self.alanimated.say(speech, configuration)


    def interrupt_animated_speech(self):
        """Stop the animated speech and goes back to init posture"""
        if not self.robot_is_connected(): return
        self.alanimated._stopAll(True)
        self.alposture.goToPosture("StandInit", 0.4)


    def get_volume(self):
        """Return the volume"""
        if not self.robot_is_connected(): return
        return self.altts.getVolume()


    def set_volume(self, level):
        """Set the volume"""
        if not self.robot_is_connected(): return

        if level < 0.0: level = 0.0
        if level > 1.0: level = 1.0
        logger.info("Setting volume to {:g}".format(level))
        self.altts.setVolume(level)


    def get_image(self,
                  resolution  = vision_definitions.kQVGA,
                  color_space = vision_definitions.kRGBColorSpace,
                  fps = 10):
        """Returns an image from one of the cameras as RGB (not BGR).

        See
        http://doc.aldebaran.com/2-4/family/pepper_technical/video_3D_pep.html
        for possible values of resolution, color spaces, etc.

        Args:
            resolution  : Size of the image (kQVGA, kQQVGA, etc.)
            color_space : Colors (kRGBColorSpace, kYuvColorSpace, etc.)

        Examples:
            >>> image = robot.get_image()
            >>> cv2.imshow("Robot", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

        """
        if not self.robot_is_connected(): return np.zeros((240,320,3))

        # logger.info("Get an image") # Too much print
        id_sub = self.alvideo.subscribe("Robot", resolution, color_space, fps)
        nao_image = self.alvideo.getImageRemote(id_sub)
        height = nao_image[1]
        width  = nao_image[0]
        np_array = np.fromstring(str(nao_image[6]), np.uint8)
        cv2_image = np.reshape(np_array, (height, width, 3))
        self.alvideo.unsubscribe(id_sub)
        return cv2_image


    def wake_up(self):
        """Wake up the robot"""
        if not self.robot_is_connected(): return
        logger.info("Waking up")
        self.almotion.wakeUp()


    def rest(self):
        """Rest the robot"""
        if not self.robot_is_connected(): return
        logger.info("Rest")
        self.almotion.rest()


    def move_joint(self,
                   joint_name = ["HeadYaw", "HeadPitch"],
                   angle_in_degree = [0.0, 0.0],
                   time_in_sec = [2.0, 2.0],
                   is_absolute = False):
        """Move the robot's head. By default, place it in the middle.

        HeadYaw > 0 turns the head on the left, HeadYaw < 0 on the
        right, HeadPitch > 0 goes down, HeadPitch < 0 goes up.

        Args:
            joint_name      :
            angle_in_degree :
            time_in_sec     :
            is_absolute     : Indicate whether angle_in_degree is relative
                              to current position

        """
        if not self.robot_is_connected(): return
        logger.info("Moving {} of {} deg in {} secs". \
                     format(joint_name, angle_in_degree, time_in_sec))

        angle = [a*almath.TO_RAD for a in angle_in_degree]

        self.almotion.angleInterpolation(joint_name, angle, time_in_sec, is_absolute)


    def nod(self):
        """Simple procedure to make the robot say "yes" with the head"""
        if not self.robot_is_connected(): return
        logger.info("Nodding")
        names = ["HeadPitch"]
        angle_lists = [15.0, 0.0]
        time_lists  = [0.4, 1.2]
        self.move_joint(names, angle_lists, time_lists)


    def set_stiffnesses(self, join_name, stiffness):
        """
        """
        if not self.robot_is_connected(): return
        self.almotion.setStiffnesses(join_name, stiffness)


    def bow(self):
        """Make a reverence"""
        if not self.robot_is_connected(): return
        logger.info("Bowing")

        formatted_text = "^start({bowshort1}) ^wait({bowshort1})". \
                         format(**animations)
        self.animated_speech(formatted_text = formatted_text)


    def apply_posture(self, posture="StandInit", speed=0.4):
        """Apply a predefined posture

        See
        http://doc.aldebaran.com/2-4/naoqi/motion/alrobotposture.html
        for available postures.

        """
        if not self.robot_is_connected(): return
        logger.info("Posture {} at {}".format(posture, speed))
        self.alposture.goToPosture(posture, speed)


    def photo_pose(self):
        """"""
        if not self.robot_is_connected(): return

        logger.info("Photo pose")
        names, times, keys = trajectory_for_photo_pose()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Photo")


    def raise_both_hands(self):
        """Raise both hand to take an object like a plate."""
        if not self.robot_is_connected(): return

        logger.info("Raising both hands")
        names, times, keys = trajectory_for_raise_both_hands()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Raise both hands")


    def stand_init_fast(self):
        """A faster version of stand init (no lag)"""
        if not self.robot_is_connected(): return
        logger.info("Stand init fast")
        names, times, keys = trajectory_for_stand_init_fast()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Stand init fast")


    def hold_glass(self):
        """Place right arm in a position as if holding a glass"""
        if not self.robot_is_connected(): return
        logger.info("Holding glass")
        names, times, keys = trajectory_for_hold_glass()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Hold glass")


    def show_behind(self):
        """Show behing the person"""
        if not self.robot_is_connected(): return

        logger.info("Showing behind")
        names, times, keys = trajectory_for_showing_behind()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Showing behind")


    def show_on_the_right(self):
        """Show something on the right"""
        if not self.robot_is_connected(): return

        logger.info("Showing on the right")
        names, times, keys = trajectory_for_showing_on_the_right()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Showing on the right")


    def show_on_the_right_high(self):
        """"""
        if not self.robot_is_connected(): return

        logger.info("Showing on the right high")
        names, times, keys = trajectory_for_showing_on_the_right_high()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Showing on the right high")


    def show_on_the_left_below(self):
        """"""
        if not self.robot_is_connected(): return

        logger.info("Showing on the left below")
        names, times, keys = trajectory_for_showing_on_the_left_below()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Showing on the left beow")


    def show_on_the_left(self):
        """Show something on the left"""
        if not self.robot_is_connected(): return
        logger.info("Showing on the left")
        names, times, keys = trajectory_for_showing_on_the_left()
        try:
            self.almotion.angleInterpolationBezier(names, times, keys)
            self.almotion.setMoveArmsEnabled(0,0)
        except: # BaseException, err:
            print("Showing on the left")


    def show_image(self, image_file_on_robot):
        """Display an image on the robot.

        The image should be accessible from the robot, either already
        on the robot at a specific location, or provide a URL if
        connected from the internet.

        When the image is in location

          /home/nao/.local/share/PackageManager/apps/idiap/html

        then the path would be

          http://198.18.0.1/apps/idiap/idiap-1600.png

        """
        if not self.robot_is_connected(): return
        logger.info("Showing image {}".format(image_file_on_robot))
        self.altablet.showImageNoCache(image_file_on_robot)
        time.sleep(0.1)
        self.altablet.showImageNoCache(image_file_on_robot)

    def load_url(self, url):
        """Load the content of a Internet web page"""
        if not self.robot_is_connected(): return

        logger.info("Loading <{}>".format(url))
        self.altablet.showWebview()
        self.altablet.loadUrl(url)
        time.sleep(0.1)


    def print_notifications(self):
        """Print the (problem) notifications.

        Use it when Pepper make a strange sound to see what is going
        on (hot motors, low battery, etc.)

        """
        if not self.robot_is_connected(): return

        notifications = self.alnotification.notifications()

        for n in notifications:
            d = dict(n)
            print("ID {}".format(d["id"]))
            print(" message: {}".format(d["message"]))
            print(" severity: {}".format(d["severity"]))
            print(" remove on read: {}".format(d["removeOnRead"]))


    def get_available_languages(self):
        """Return the list of available languages, and an empty list of the
        robot is not connected

        """
        l = []
        if self.session.isConnected():
            l = self.altts.getAvailableLanguages()
        return l


    def update_content_url_page(self, content, id_name):
        """Update"""
        if not self.robot_is_connected(): return

        js = "var x = document.getElementById('{}').innerHTML = '{}';". \
                 format(id_name, content)
        # self.altablet.reloadPage(1)
        # time.sleep(0.1)
        self.altablet.executeJS(js)


    def display_qcm_question(self,
                             q_and_a,
                             display_answer = 0,
                             url = "http://198.18.0.1/apps/idiap/qcm.html"):
        """Display a question and 4 answers

        Args:
            q_and_a : A dict with keys ["question", "answer", "answerX"] X in {1,2,3,4}
            url     : The .html page with tags whose names are the keys of q_and_a

        Example:

            q_and_a = { "question": "What is the capital of France?",
                        "answer1" : "London",
                        "answer2" : "Paris",
                        "answer3" : "Berlin",
                        "answer4" : "Madrid",
                        "answer"  : "answer1" }

        """
        if not self.robot_is_connected(): return

        keys = ["answer{}".format(i) for i in range(1,5)] + ["question", "answer"]

        keys_ok = True
        for key in keys:
            if key not in q_and_a:
                logger.error("Key {} is missing".format(key))
                keys_ok = False

        if not keys_ok:
            return

        js = ""

        if display_answer > 0:
            for k, v in q_and_a.items():
                if k == "answer": continue
                if k == "question": continue

                js += "x = document.getElementById('{}');".format(k)
                # Put answer in bold and others in grey
                if not k == q_and_a["answer"]:
                    js += "x.style.color = '#cccccc';"
                else:
                    js += "x.style.fontWeight = 'bold';"

        else:
            for k, v in q_and_a.items():
                if k == "answer": continue

                js += "x = document.getElementById('{}');".format(k)
                js += "x.innerHTML = '{}';".format(v)

                # Only question loads the URL. We assume display
                # answer on top of question
                self.load_url(url)

        time.sleep(0.3) # Need otherwise the js is not executed...
        self.altablet.executeJS(js)


    def center_body_with_head(self):
        """Turn body and head in opposite direction so that the head and body
           are aligned but facing the original line of sight

        """
        if not self.robot_is_connected(): return

        yaw = self.almotion.getAngles(["HeadYaw"], 1)

        if len(yaw) > 0:
            yaw = yaw[0]*almath.TO_DEG
        else:
            logger.error("Problem to get the yaw")
            return

        angle_to_turn = -yaw

        time_in_sec = 2.0

        # Slow down the head mvt if large angle
        # if angle_to_turn > 30: time_in_sec += 1
        # if angle_to_turn > 60: time_in_sec += 1

        logger.info("Yaw is {} deg head time is {}". \
                     format(yaw, time_in_sec))

        joint_name = ["HeadYaw", "HeadPitch"]
        angle = [angle_to_turn*almath.TO_RAD, 0]
        times_in_sec = [time_in_sec, time_in_sec]
        is_absolute = False

        # Both functions will be run "roughly together"
        self.almotion.moveTo(0, 0,
                             -angle_to_turn*almath.TO_RAD,
                             _async=True)

        self.almotion.angleInterpolation(joint_name,
                                         angle,
                                         times_in_sec,
                                         is_absolute)
