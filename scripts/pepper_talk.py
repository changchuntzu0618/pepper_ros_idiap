#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import qi
import argparse
import sys

import rospy
from std_msgs.msg import String


class robot:
    def __init__(self, session):
        self.tts=session.service("ALTextToSpeech")
        self.tts.setLanguage("English")
        self.sub=rospy.Subscriber('/rasa/rasa_response', String, self.give_to_pepper)
    

    def give_to_pepper(self, data):

        pepper_say=data.data

        # Get the service ALTextToSpeech.
        rospy.loginfo('Pepper say: "%s" ' % pepper_say)
        self.tts.say(str(pepper_say))

        while self.tts.isSpeaking():
            print('Pepper is talking')
        print('Finish talking')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=os.environ.get("NAO_IP",None),
                        help="Robot IP address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    
    session = qi.Session()
    rospy.init_node('pepper_say')
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
        # session=None
        Robot=robot(session)

    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    rospy.spin()
    
    

    
