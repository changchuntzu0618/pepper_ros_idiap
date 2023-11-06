#!/usr/bin/env python

from __future__ import print_function

"""Test rest

  python test_rest.py

"""

import argparse
import os
import sys
import time

import qi

from robot import Robot

def mycallback(value):
    print("val:", value)

if __name__ == "__main__":
    """
    """
    parser = argparse.ArgumentParser()

    parser.add_argument("--ip",
                        type=str,
                        default=os.environ.get("NAO_IP", None),
                        help="Robot IP address")
    parser.add_argument("--port",
                        type=int,
                        default=9559,
                        help="Naoqi port number")

    opts = parser.parse_args()

    session = qi.Session()

    try:
        session.connect("tcp://" + opts.ip + ":" + str(opts.port))
    except RuntimeError:
        print ("Cannot connect ip {} on port {}".format(opts.ip, opts.port))
        sys.exit(1)

    # robot = Robot(session)

    memory = session.service("ALMemory")
    subscriber = memory.subscriber("ALSoundLocalization/SoundLocated")
    subscriber.signal.connect(mycallback)

    alsound = session.service("ALSoundLocalization")
    alsound.setParameter("Sensitivity", 0.6)
    print(dir(alsound))
    name = "sounddet"
    alsound.subscribe(name)
    time.sleep(1)

    # [ [time(sec), time(usec)],

    #   [azimuth(rad), elevation(rad), confidence, energy],

    #   [Head Position[6D]] in FRAME_TORSO
    #   [Head Position[6D]] in FRAME_ROBOT
    # ]
    for _ in range(10):
        print(alsound.getEventList())
        time.sleep(1)

    alsound.unsubscribe(name)
    time.sleep(1)
