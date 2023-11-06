

#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use Tracking Module to Track a Face"""

import qi
import argparse
import sys
import time


def main(session, faceSize):
    """
    This example shows how to use ALTracker with face.
    """
    # Get the services ALTracker and ALMotion.

    motion_service = session.service("ALMotion")
    tracker_service = session.service("ALTracker")

    # First, wake up.
    # motion_service.wakeUp()

    # Add target to track.
    targetName = "Face"
    faceWidth = faceSize
    tracker_service.registerTarget(targetName, faceWidth)

    # Then, start tracker.
    tracker_service.track(targetName)

    print "ALTracker successfully started, now show your face to robot!"
    print "Use Ctrl+c to stop this script."

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user"
        print "Stopping..."

    # Stop tracker.
    tracker_service.stopTracker()
    tracker_service.unregisterAllTargets()
    motion_service.rest()

    print "ALTracker stopped."


if __name__ == "__main__":
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

    main(session, 10)
