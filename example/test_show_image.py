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
    parser.add_argument("--image",
                        type=str,
                        default="",
                        help="Image path on robot")

    opts = parser.parse_args()

    session = qi.Session()

    try:
        session.connect("tcp://" + opts.ip + ":" + str(opts.port))
    except RuntimeError:
        print ("Cannot connect ip {} on port {}".format(opts.ip, opts.port))
        sys.exit(1)

    image_path_on_robot = opts.image

    # This image idiap-1600.png should be in
    #
    #    /home/nao/.local/share/PackageManager/apps/idiap/html/idiap-1600.png
    #
    if len(image_path_on_robot) == 0:
        image_path_on_robot = "http://198.18.0.1/apps/idiap/idiap-1600.png"

    robot = Robot(session)

    robot.show_image(image_path_on_robot)

    print("The first time you call that, it may not work. Run it again...")
