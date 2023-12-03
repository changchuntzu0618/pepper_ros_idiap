#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from end_of_speech.msg import EndOfSpeech
import requests

sender = "user"

rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"


class RASA:
    def __init__(self):
        # self.sub=rospy.Subscriber('/human_dialogue', String, self.rasa_response)
        self.sub=rospy.Subscriber('/end_of_speech/eos', EndOfSpeech, self.rasa_response)
        
        self.pub= rospy.Publisher('~rasa_response', String, queue_size=1)
        self.get_emotion=False

    def rasa_response(self, eos_msg):
        self.text=eos_msg.final_utterance
        self.confidence=eos_msg.confidence
        self.header=eos_msg.header
        rospy.loginfo('Heard User said: "%s", with confidence "%f" ' % (self.text, self.confidence))

        self.results = requests.post(
            rasa_endpoint, json={"sender": sender, "message": self.text}
        ).json()
        
        
        if self.results:
            self.emotion=rospy.wait_for_message('/fer/emotion', String, timeout=10)
            content='I am feeling '+str(self.emotion.data)
            self.results_emotion = requests.post(
                rasa_endpoint, json={"sender": sender, "message": content}
            ).json()
        
            for result in self.results_emotion:
                result=result['text']
                rospy.loginfo('Rasa response: "%s" ' % (result))
                self.pub.publish(result)
        else:
            for result in self.results:
                result=result['text']
                rospy.loginfo('Rasa response: "%s" ' % (result))
                self.pub.publish(result)



if __name__ == '__main__':
    rospy.init_node('rasa')
    Rasa=RASA()

    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
