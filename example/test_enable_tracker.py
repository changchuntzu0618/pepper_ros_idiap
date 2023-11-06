#!/usr/bin/env python

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

    tracking_enabled = True
    face_service = session.service("ALFaceDetection")
    tracker_service = session.service("ALTracker")

    print("Will set tracking to {} on the robot...".format(tracking_enabled))
    face_service.enableTracking(tracking_enabled)
    # pprint.pprint(dir(tracker_service))
    face_service.pause(0)
    print("tracker_service.isActive() {}".format(tracker_service.isActive()))
    # print("tracker_service.isActive() {}".format(tracker_service.isActive()))
    # print("tracker_service.isRunning() {}".format(tracker_service.isRunning()))

    print("tracker_service.getActiveTarget() {}".format(tracker_service.getActiveTarget()))
    print("tracker_service.getRegisteredTargets() {}".format(tracker_service.getRegisteredTargets()))

    tracker_service.registerTarget("Face", 50)
    print("tracker_service.getRegisteredTargets() {}".format(tracker_service.getRegisteredTargets()))
    print("tracker_service.getActiveTarget() {}".format(tracker_service.getActiveTarget()))

    tracker_service.track("Face")

    # print("tracker_service.getTargetPosition() {}".format(tracker_service.getTargetPosition()))

    # tracker_service.unregisterAllTargets()

    print("tracker_service.getRegisteredTargets() {}".format(tracker_service.getRegisteredTargets()))

    # print("tracker_service.getActiveTarget() {}".format(tracker_service.getActiveTarget()))
    # face_service.(tracking_enabled)
    print("face_service.isPaused() {}".format(face_service.isPaused()))

    print("Is tracking now enabled on the robot? {}".format(face_service.isTrackingEnabled()))

    print("tracker_service.getRegisteredTargets() {}".format(tracker_service.getRegisteredTargets()))
    # tracker_service.stopTracker()
