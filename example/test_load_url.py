#!/usr/bin/env python

from __future__ import print_function

"""Test showing an image

  python test_show_image.py --image http://198.18.0.1/apps/idiap/black.png

"""

import os
import sys
import time
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
    parser.add_argument("--url",
                        type=str,
                        default="https://www.google.com",
                        help="URL to load")

    opts = parser.parse_args()

    session = qi.Session()

    try:
        session.connect("tcp://" + opts.ip + ":" + str(opts.port))
    except RuntimeError:
        print ("Cannot connect ip {} on port {}".format(opts.ip, opts.port))
        sys.exit(1)

    robot = Robot(session)
    robot.load_url(opts.url)
