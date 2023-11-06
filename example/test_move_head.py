#!/usr/bin/env python

import os
import sys
import argparse

import qi

from robot import Robot

if __name__ == "__main__":
    """
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("-N",
                        type=int,
                        default=2,
                        help="Nb of times")
    parser.add_argument("-a", "--amplitude",
                        type=float,
                        default=40.0,
                        help="Amplitude of movement")
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
    robot.move_joint(is_absolute=True)
    time_in_sec = 3.0

    amplitude = opts.amplitude/2.0
    robot.move_joint(joint_name      = ["HeadYaw"],
                     angle_in_degree = [amplitude],
                     time_in_sec     = [time_in_sec])
    amplitude = -opts.amplitude

    for n in range(opts.N):
        # Right
        robot.move_joint(joint_name      = ["HeadYaw"],
                         angle_in_degree = [amplitude],
                         time_in_sec     = [time_in_sec])
        amplitude *= -1

    # Go in the middle
    robot.move_joint(is_absolute=True)
