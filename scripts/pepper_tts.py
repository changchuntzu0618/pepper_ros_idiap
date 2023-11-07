#!/usr/bin/env python
import os
import qi
import argparse
import sys

import rospy
from std_msgs.msg import String

class robot:
    def __init__(self, session):
        self.session = session

    def give_to_pepper(self, data):

        text = data.data
        rospy.loginfo('Heard message: "%s" ' % text)
        # Get the service ALTextToSpeech.

        # tts = self.session.service("ALTextToSpeech")
        # tts.setLanguage("English")
        # tts.say(text)


    def listener(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber('rasa_response', String, self.give_to_pepper())

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=os.environ.get("NAO_IP",None),
                        help="Robot IP address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    
    # session = qi.Session()
    try:
        # session.connect("tcp://" + args.ip + ":" + str(args.port))
        session=None
        Robot=robot(session)
        Robot.listener(session)

    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    
    
