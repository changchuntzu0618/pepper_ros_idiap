#!/usr/bin/env python

from __future__ import print_function

"""Test moving the joints of the head

  python test_move_joint.py

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

    opts = parser.parse_args()

    session = qi.Session()

    try:
        session.connect("tcp://" + opts.ip + ":" + str(opts.port))
    except RuntimeError:
        print ("Cannot connect ip {} on port {}".format(opts.ip, opts.port))
        sys.exit(1)

    robot = Robot(session)

    # Go in the middle
    robot.move_joint(is_absolute = True)

    # Left
    robot.move_joint(joint_name      = ["HeadYaw"],
                     angle_in_degree = [30],
                     time_in_sec     = [1.0])

    # Right
    robot.move_joint(joint_name      = ["HeadYaw"],
                     angle_in_degree = [-40],
                     time_in_sec     = [1.0])

    # Down
    robot.move_joint(joint_name      = ["HeadPitch"],
                     angle_in_degree = [10],
                     time_in_sec     = [1.0])

    # Up
    robot.move_joint(joint_name      = ["HeadPitch"],
                     angle_in_degree = [-20],
                     time_in_sec     = [1.0])

    # Go in the middle
    robot.move_joint(is_absolute = True)
