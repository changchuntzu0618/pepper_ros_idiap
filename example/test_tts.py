#!/usr/bin/env python

from __future__ import print_function

"""Test text to speech

  python test_tts.py --lang en "Hello, I am Pepper"

"""

import os
import sys
import argparse

import qi

from robot import Robot

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
    parser.add_argument("--lang",
                        type=str,
                        default="fr",
                        help="Language")
    parser.add_argument("text",
                        type=str,
                        default="Pepper",
                        help="The text")

    opts = parser.parse_args()

    session = qi.Session()

    try:
        session.connect("tcp://" + opts.ip + ":" + str(opts.port))
    except RuntimeError:
        print ("Cannot connect ip {} on port {}".format(opts.ip, opts.port))
        sys.exit(1)

    robot = Robot(session)
    robot.set_language(opts.lang)
    robot.say(opts.text)
