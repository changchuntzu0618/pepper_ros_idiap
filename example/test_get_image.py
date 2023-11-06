#!/usr/bin/env python

from __future__ import print_function

"""Test the cameras

  python test_get_image.py

Press 'q' to quit.

"""

import os
import sys
import argparse

import qi

import cv2

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

    opts = parser.parse_args()

    session = qi.Session()

    try:
        session.connect("tcp://" + opts.ip + ":" + str(opts.port))
    except RuntimeError:
        print ("Cannot connect ip {} on port {}".format(opts.ip, opts.port))
        sys.exit(1)

    robot = Robot(session)

    while 1:
        frame = robot.get_image()
        cv2.imshow("Robot camera", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        if cv2.waitKey(30) == ord('q'): # 'q'
            break
