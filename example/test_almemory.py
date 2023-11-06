#!/usr/bin/env python

from __future__ import print_function

"""Test wake up

  python test_wake_up.py

"""

import os
import sys
import argparse
import pprint
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


    almemory = session.service("ALMemory")
    alperception = session.service("ALPeoplePerception")
    altracker = session.service("ALTracker")

    print("alperception.isFaceDetectionEnabled {}".format(alperception.isFaceDetectionEnabled()))

    # pprint.pprint(almemory.getDataListName())

    print("altracker.getTargetCoordinates() {}".format(altracker.getTargetCoordinates()))
    print("altracker.getTargetCoordinates() {}".format(altracker.getTargetPosition()))
    altracker.unregisterAllTargets()
    print("altracker.getTargetCoordinates() {}".format(altracker.getTargetPosition()))

    names = almemory.getDataListName()

    for name in names:
        data = almemory.getData(name)
        print((name, data))
