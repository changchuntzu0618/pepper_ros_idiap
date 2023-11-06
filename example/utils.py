######################################################################
# Copyright (c) 2018 Idiap Research Institute <http://www.idiap.ch/>
######################################################################

from __future__ import print_function

import os
import sys
import time

import qi
import almath
import vision_definitions

animations = {
    "hey1": "animations/Stand/Gestures/Hey_1", # hand on top
    "hey3": "animations/Stand/Gestures/Hey_3",
    "hey4": "animations/Stand/Gestures/Hey_4",
    "hey6": "animations/Stand/Gestures/Hey_6",
    "explain1": "animations/Stand/Gestures/Explain_1",
    "explain2": "animations/Stand/Gestures/Explain_2", # Geste avec bras
    "explain3": "animations/Stand/Gestures/Explain_3",
    "explain4": "animations/Stand/Gestures/Explain_4",
    "explain5": "animations/Stand/Gestures/Explain_5",
    "desperate1": "animations/Stand/Gestures/Desperate_1",
    "desperate2": "animations/Stand/Gestures/Desperate_2",
    "desperate4": "animations/Stand/Gestures/Desperate_4",
    "desperate5": "animations/Stand/Gestures/Desperate_5",
    "embarrassed1": "animations/Stand/Emotions/Neutral/Embarrassed_1",
    "but1": "animations/Stand/Gestures/But_1",
    "me1": "animations/Stand/Gestures/Me_1",
    "me2": "animations/Stand/Gestures/Me_2",
    "me4": "animations/Stand/Gestures/Me_4",
    "me7": "animations/Stand/Gestures/Me_7",
    "please1": "animations/Stand/Gestures/Please_1",
    "enthusiastic4": "animations/Stand/Gestures/Enthusiastic_4",
    "enthusiastic5": "animations/Stand/Gestures/Enthusiastic_5",
    "thinking1": "animations/Stand/Gestures/Thinking_1",
    "thinking3": "animations/Stand/Gestures/Thinking_3",
    "thinking4": "animations/Stand/Gestures/Thinking_4",
    "thinking6": "animations/Stand/Gestures/Thinking_6",
    "yes1": "animations/Stand/Gestures/Yes_1",
    "yes2": "animations/Stand/Gestures/Yes_2",
    "yes3": "animations/Stand/Gestures/Yes_3",
    "you1": "animations/Stand/Gestures/You_1",
    "you4": "animations/Stand/Gestures/You_4",
    "showsky1": "animations/Stand/Gestures/ShowSky_1",
    "no1": "animations/Stand/Gestures/No_1",
    "no2": "animations/Stand/Gestures/No_2",
    "no3": "animations/Stand/Gestures/No_3",
    "happy4": "animations/Stand/Emotions/Positive/Happy_4",
    "peaceful1": "animations/Stand/Emotions/Positive/Peaceful_1",
    "bored1" : "animations/Stand/Emotions/Negative/Bored_1",

    "bodytalk1": "animations/Stand/BodyTalk/BodyTalk_1",
    "bodytalk2": "animations/Stand/BodyTalk/BodyTalk_2",
    "bodytalk3": "animations/Stand/BodyTalk/BodyTalk_3",
    "bodytalk4": "animations/Stand/BodyTalk/BodyTalk_4",
    "bodytalk5": "animations/Stand/BodyTalk/BodyTalk_5",
    "bodytalk6": "animations/Stand/BodyTalk/BodyTalk_6",
    "bodytalk7": "animations/Stand/BodyTalk/BodyTalk_7",
    "bodytalk8": "animations/Stand/BodyTalk/BodyTalk_8",
    "bodytalk9": "animations/Stand/BodyTalk/BodyTalk_9",
    "bodytalk10": "animations/Stand/BodyTalk/BodyTalk_10",
    "bodytalk11": "animations/Stand/BodyTalk/BodyTalk_11",
    "bodytalk12": "animations/Stand/BodyTalk/BodyTalk_12",
    "bodytalk13": "animations/Stand/BodyTalk/BodyTalk_13",
    "bodytalk14": "animations/Stand/BodyTalk/BodyTalk_14",
    "bodytalk15": "animations/Stand/BodyTalk/BodyTalk_15",
    "bodytalk16": "animations/Stand/BodyTalk/BodyTalk_16",

    "showtablet1": "animations/Stand/Gestures/ShowTablet_1",
    "showtablet2": "animations/Stand/Gestures/ShowTablet_2",
    "showtablet3": "animations/Stand/Gestures/ShowTablet_3",

    "bowshort1":   "animations/Stand/Gestures/BowShort_1",
}

def trajectory_for_stand_init_fast():
    """"""
    names = list()
    times = list()
    keys = list()

    time_in_sec = 1.2

    names.append("HeadPitch")
    times.append([time_in_sec])
    keys.append([[-0.297592, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([time_in_sec])
    keys.append([[0.0184078, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([time_in_sec])
    keys.append([[-0.0214758, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([time_in_sec])
    keys.append([[-0.0168738, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([time_in_sec])
    keys.append([[0.00153399, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([time_in_sec])
    keys.append([[-0.0859029, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([time_in_sec])
    keys.append([[-1.15816, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([time_in_sec])
    keys.append([[0.574692, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.49256, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([time_in_sec])
    keys.append([[0.124253, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([time_in_sec])
    keys.append([[0.11194, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([time_in_sec])
    keys.append([[0.0138059, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([time_in_sec])
    keys.append([[0.561437, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([time_in_sec])
    keys.append([[0.572056, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.47876, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([time_in_sec])
    keys.append([[-0.105845, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([time_in_sec])
    keys.append([[0.454022, [3, -0.333333, 0], [3, 0, 0]]])

    return names, times, keys



def trajectory_for_photo_pose():
    names = list()
    times = list()
    keys = list()

    time_in_sec = 1.5

    names.append("HeadPitch")
    times.append([time_in_sec])
    keys.append([[-0.380427, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([time_in_sec])
    keys.append([[0.0153399, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([time_in_sec])
    keys.append([[-0.0291457, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([time_in_sec])
    keys.append([[-0.0122719, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([time_in_sec])
    keys.append([[-0.00460196, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([time_in_sec])
    keys.append([[-0.0674951, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([time_in_sec])
    keys.append([[-1.738, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([time_in_sec])
    keys.append([[0.630053, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.44501, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([time_in_sec])
    keys.append([[1.55392, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([time_in_sec])
    keys.append([[-1.81323, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([time_in_sec])
    keys.append([[0.374291, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([time_in_sec])
    keys.append([[1.70425, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([time_in_sec])
    keys.append([[0.665202, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.08146, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([time_in_sec])
    keys.append([[-1.53245, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([time_in_sec])
    keys.append([[1.23943, [3, -0.333333, 0], [3, 0, 0]]])

    return names, times, keys



def trajectory_for_showing_on_the_left():
    """"""
    names = list()
    times = list()
    keys = list()

    time_in_sec = 1.5

    names.append("HeadPitch")
    times.append([time_in_sec])
    keys.append([[-0.168738, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([time_in_sec])
    keys.append([[0.783864, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([time_in_sec])
    keys.append([[-0.0414176, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([time_in_sec])
    keys.append([[0.00613594, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([time_in_sec])
    keys.append([[-0.00766993, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([time_in_sec])
    keys.append([[-0.00872665, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([time_in_sec])
    keys.append([[-1.23792, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([time_in_sec])
    keys.append([[0.567663, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([time_in_sec])
    keys.append([[-0.145728, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([time_in_sec])
    keys.append([[1.02317, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([time_in_sec])
    keys.append([[0.737812, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([time_in_sec])
    keys.append([[0.523088, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([time_in_sec])
    keys.append([[1.22872, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([time_in_sec])
    keys.append([[0.599297, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.56926, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([time_in_sec])
    keys.append([[-0.121185, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([time_in_sec])
    keys.append([[0.0137641, [3, -0.333333, 0], [3, 0, 0]]])


    return names, times, keys

def trajectory_for_showing_on_the_left_below():
    """"""
    names = list()
    times = list()
    keys = list()

    time_in_sec = 1.5

    names.append("HeadPitch")
    times.append([time_in_sec])
    keys.append([[0.0506213, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([time_in_sec])
    keys.append([[0.91732, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([time_in_sec])
    keys.append([[-0.0199418, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([time_in_sec])
    keys.append([[0, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([time_in_sec])
    keys.append([[0.00766993, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([time_in_sec])
    keys.append([[-0.242369, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([time_in_sec])
    keys.append([[-1.22105, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([time_in_sec])
    keys.append([[0.517575, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.19651, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([time_in_sec])
    keys.append([[0.897379, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([time_in_sec])
    keys.append([[-0.757838, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([time_in_sec])
    keys.append([[0.760854, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([time_in_sec])
    keys.append([[0.573709, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([time_in_sec])
    keys.append([[0.593146, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.14742, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([time_in_sec])
    keys.append([[-0.0874369, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([time_in_sec])
    keys.append([[0.693326, [3, -0.333333, 0], [3, 0, 0]]])

    return names, times, keys


def trajectory_for_showing_behind():
    """"""
    names = list()
    times = list()
    keys = list()

    time_in_sec = 1.5

    names.append("HeadPitch")
    times.append([time_in_sec])
    keys.append([[-0.220893, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([time_in_sec])
    keys.append([[-0.194816, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([time_in_sec])
    keys.append([[-0.0291457, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([time_in_sec])
    keys.append([[-0.0153399, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([time_in_sec])
    keys.append([[0.00766993, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([time_in_sec])
    keys.append([[-0.00920391, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([time_in_sec])
    keys.append([[-1.21338, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([time_in_sec])
    keys.append([[0.591388, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.6475, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([time_in_sec])
    keys.append([[0.076699, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([time_in_sec])
    keys.append([[-0.48632, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([time_in_sec])
    keys.append([[0.495476, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([time_in_sec])
    keys.append([[0.299126, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([time_in_sec])
    keys.append([[0.463093, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([time_in_sec])
    keys.append([[-0.340544, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([time_in_sec])
    keys.append([[-0.636602, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([time_in_sec])
    keys.append([[0.730142, [3, -0.333333, 0], [3, 0, 0]]])

    return names, times, keys


def trajectory_for_showing_on_the_right_high():
    """"""
    names = list()
    times = list()
    keys = list()

    time_in_sec = 1.5


    names.append("HeadPitch")
    times.append([time_in_sec])
    keys.append([[-0.477068, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([time_in_sec])
    keys.append([[-0.773126, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([time_in_sec])
    keys.append([[-0.0306797, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([time_in_sec])
    keys.append([[-0.00460196, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([time_in_sec])
    keys.append([[-0.00613594, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([time_in_sec])
    keys.append([[-0.0874369, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([time_in_sec])
    keys.append([[-1.21031, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([time_in_sec])
    keys.append([[0.59051, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.49103, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([time_in_sec])
    keys.append([[0.0905049, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([time_in_sec])
    keys.append([[-0.300706, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([time_in_sec])
    keys.append([[0.00872665, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([time_in_sec])
    keys.append([[0.576777, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([time_in_sec])
    keys.append([[0.531634, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([time_in_sec])
    keys.append([[-0.920388, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([time_in_sec])
    keys.append([[-0.599787, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([time_in_sec])
    keys.append([[0.131882, [3, -0.333333, 0], [3, 0, 0]]])

    return names, times, keys


def trajectory_for_showing_on_the_right():
    """"""
    names = list()
    times = list()
    keys = list()

    time_in_sec = 1.5

    names.append("HeadPitch")
    times.append([time_in_sec])
    keys.append([[0.0475533, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("HeadYaw")
    times.append([time_in_sec])
    keys.append([[-1.26093, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("HipPitch")
    times.append([time_in_sec])
    keys.append([[-0.0260777, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("HipRoll")
    times.append([time_in_sec])
    keys.append([[-0.0107379, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("KneePitch")
    times.append([time_in_sec])
    keys.append([[0.00613594, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("LElbowRoll")
    times.append([time_in_sec])
    keys.append([[-0.113515, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("LElbowYaw")
    times.append([time_in_sec])
    keys.append([[-1.71806, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("LHand")
    times.append([time_in_sec])
    keys.append([[0.682777, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("LShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.76561, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("LShoulderRoll")
    times.append([time_in_sec])
    keys.append([[0.076699, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("LWristYaw")
    times.append([time_in_sec])
    keys.append([[0.0459781, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("RElbowRoll")
    times.append([time_in_sec])
    keys.append([[0.305262, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("RElbowYaw")
    times.append([time_in_sec])
    keys.append([[1.70579, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("RHand")
    times.append([time_in_sec])
    keys.append([[0.591388, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("RShoulderPitch")
    times.append([time_in_sec])
    keys.append([[1.13361, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("RShoulderRoll")
    times.append([time_in_sec])
    keys.append([[-1.52478, [3, -0.333333, 0], [3, 0.16, 0]]])

    names.append("RWristYaw")
    times.append([time_in_sec])
    keys.append([[0.95564, [3, -0.333333, 0], [3, 0.16, 0]]])


    return names, times, keys


def trajectory_for_raise_both_hands():
    """Pepper raises its hand like to take a plate"""
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.96])
    keys.append([[-0.521554, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([0.96])
    keys.append([[0.0536892, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([0.96])
    keys.append([[-0.0444853, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([0.96])
    keys.append([[-0.0291457, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([0.96])
    keys.append([[-0.00766993, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([0.96])
    keys.append([[-0.788466, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([0.96])
    keys.append([[-0.883573, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([0.96])
    keys.append([[0.664323, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([0.96])
    keys.append([[0.625864, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([0.96])
    keys.append([[0.0536892, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([0.96])
    keys.append([[-1.82243, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([0.96])
    keys.append([[0.855961, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([0.96])
    keys.append([[0.888175, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([0.96])
    keys.append([[0.674868, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([0.96])
    keys.append([[0.641204, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([0.96])
    keys.append([[-0.130388, [3, -0.333333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([0.96])
    keys.append([[1.82082, [3, -0.333333, 0], [3, 0, 0]]])

    return names, times, keys


def trajectory_for_hold_glass():
    """Raise right hand like to take a glass"""

    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.96, 1.44])
    keys.append([[-0.383495, [3, -0.333333, 0], [3, 0.16, 0]],
                 [-0.383495, [3, -0.16, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([0.96, 1.44])
    keys.append([[0.0184078, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.0184078, [3, -0.16, 0], [3, 0, 0]]])

    names.append("HipPitch")
    times.append([0.96, 1.44])
    keys.append([[-0.0260777, [3, -0.333333, 0], [3, 0.16, 0]],
                 [-0.0260777, [3, -0.16, 0], [3, 0, 0]]])

    names.append("HipRoll")
    times.append([0.96, 1.44])
    keys.append([[-0.00460196, [3, -0.333333, 0], [3, 0.16, 0]],
                 [-0.00460196, [3, -0.16, 0], [3, 0, 0]]])

    names.append("KneePitch")
    times.append([0.96, 1.44])
    keys.append([[0.00306797, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.00306797, [3, -0.16, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([0.96, 1.44])
    keys.append([[-0.113515, [3, -0.333333, 0], [3, 0.16, 0]],
                 [-0.113515, [3, -0.16, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([0.96, 1.44])
    keys.append([[-1.71806, [3, -0.333333, 0], [3, 0.16, 0]],
                 [-1.71806, [3, -0.16, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([0.96, 1.44])
    keys.append([[0.681898, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.681898, [3, -0.16, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([0.96, 1.44])
    keys.append([[1.76561, [3, -0.333333, 0], [3, 0.16, 0]],
                 [1.76561, [3, -0.16, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([0.96, 1.44])
    keys.append([[0.078233, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.078233, [3, -0.16, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([0.96, 1.44])
    keys.append([[0.0352399, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.0352399, [3, -0.16, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([0.96, 1.44])
    keys.append([[1.06765, [3, -0.333333, 0], [3, 0.16, 0]],
                 [1.06765, [3, -0.16, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([0.96, 1.44])
    keys.append([[0.757787, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.757787, [3, -0.16, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([0.96, 1.44])
    keys.append([[0.626538, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.626538, [3, -0.16, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([0.96, 1.44])
    keys.append([[0.987884, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.987884, [3, -0.16, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([0.96, 1.44])
    keys.append([[-0.190214, [3, -0.333333, 0], [3, 0.16, 0]],
                 [-0.190214, [3, -0.16, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([0.96, 1.44])
    keys.append([[0.323632, [3, -0.333333, 0], [3, 0.16, 0]],
                 [0.323632, [3, -0.16, 0], [3, 0, 0]]])

    return names, times, keys


def in_love(session, seconds):
    """Make eyes and shoulders LEDs blink red for "seconds" long. At the
    end, reset all LEDs to default (white for all, and blue for ears)

    """
    altablet = session.service("ALTabletService")
    res = altablet.showImage("http://198.18.0.1/apps/idiap/fox.gif")
    # res = altablet.showImage("http://198.18.0.1/apps/idiap/beating-heart-1280.gif")
    # time.sleep(1) # Wait for image to be loaded
    # res = altablet.showImage("http://198.18.0.1/apps/idiap/beating-heart-1280.gif")

    time.sleep(3.0) # Wait for image to be loaded

    # leds.rotateEyes(0x00ff0000, 1, 10)
    alleds = session.service("ALLeds")
    alleds.setIntensity("ChestLeds", 1)

    alleds.off("AllLeds")
    alleds.on("AllLedsRed")
    # alleds.on("AllLedsGreen")
    # alleds.on("AllLedsBlue")
    r_intensity = 1
    # g_intensity = 84.0/255
    # b_intensity = 167.0/255
    delta = -0.1
    # 255 84 167
    start_time = time.time()
    while time.time() - start_time < seconds:
        alleds.setIntensity("AllLedsRed", r_intensity)
        # alleds.setIntensity("AllLedsGreen", g_intensity)
        # alleds.setIntensity("AllLedsBlue", b_intensity)
        r_intensity = r_intensity + delta
        if r_intensity < 0.1: delta = +0.1
        if r_intensity > 0.9: delta = -0.1
        print(r_intensity)
        time.sleep(0.02)

    alleds.reset("AllLeds")
    res = altablet.showImage("http://198.18.0.1/apps/idiap/black.png")
