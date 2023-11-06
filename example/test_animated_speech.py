#!/usr/bin/env python

from __future__ import print_function

"""Test the animated speech.

  python test_animated_speech.py --lang en "^start({hey1}) Hi, I am pepper."

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
    parser.add_argument("--formatted-text",
                        type=str,
                        default="",
                        help="The text")
    parser.add_argument("--formatted-file",
                        type=str,
                        default="",
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
    robot.animated_speech(formatted_text = opts.formatted_text,
                          formatted_file = opts.formatted_file)

    robot.stand_init_fast()
