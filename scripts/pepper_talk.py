#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
The code includes the section for sending text and "say" commands to Pepper. It receives the text 
from another node, sends it to Pepper, and Pepper will articulate it either in the 'ALTextToSpeech' 
mode (just saying it) or in the 'ALAnimatedSpeech' mode (accompanied by movements while talking).
"""

import os
import qi
import argparse
import sys

import rospy
from std_msgs.msg import String
from pepper_ros.msg import PepperTalkTime


class robot:
    def __init__(self, session):
        ## for using ALTextToSpeech
        # self.tts=session.service("ALTextToSpeech")
        # self.tts.setLanguage("English")

        # for using ALAnimatedSpeech (speak while moving)
        self.tts=session.service("ALAnimatedSpeech")
        # set the local configuration
        self.configuration = {"bodyLanguageMode":"contextual"}
        
        self.sub=rospy.Subscriber('/rasa/rasa_response', String, self.give_to_pepper)
        self.pub= rospy.Publisher('~talk_time', PepperTalkTime, queue_size=1)
    

    def give_to_pepper(self, data):

        pepper_say=data.data
        # Get the service ALTextToSpeech.
        rospy.loginfo('Pepper say: "%s" ' % pepper_say)

        talk_time=PepperTalkTime()

        start_stamp=rospy.get_rostime()

        ## for using ALTextToSpeech
        # self.tts.say(str(pepper_say))

        # for using ALAnimatedSpeech (speak while moving)
        self.tts.say(str(pepper_say),self.configuration)

        finish_stamp=rospy.get_rostime()
        talk_time.start_stamp=start_stamp
        talk_time.finish_stamp=finish_stamp
        self.pub.publish(talk_time)


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
        Robot=robot(session)

    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    rospy.spin()
    
    

    
