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

    almotion = session.service("ALMotion")
    alfacedetection = session.service("ALFaceDetection")
    altracker = session.service("ALTracker")

    # print(dir(motion_service))
    # print(dir(tracker_service))


    print("altracker.getActiveTarget {}".format(altracker.getActiveTarget()))
    print("altracker.getRegisteredTargets {}".format(altracker.getRegisteredTargets()))
    print("altracker.getSupportedTargets {}".format(altracker.getSupportedTargets()))
    print("altracker.getTargetPosition {}".format(altracker.getTargetPosition()))
    print("altracker.getTargetCoordinates {}".format(altracker.getTargetCoordinates()))
    print("altracker.isActive {}".format(altracker.isActive()))
    print("altracker.isSearchEnabled {}".format(altracker.isSearchEnabled()))
    print("altracker.isTargetLost {}".format(altracker.isTargetLost()))

    print("alfacedetection.isTrackingEnabled {}".format(alfacedetection.isTrackingEnabled()))
