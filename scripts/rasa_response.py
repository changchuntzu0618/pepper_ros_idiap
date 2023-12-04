#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from end_of_speech.msg import EndOfSpeech
import requests

sender = "user"

rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"

utter_think='Let me think about it'
detect_emotion='detect emotion'

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

        self.send_to_rasa(self.text)
        
       

    def send_to_rasa(self, text):
        results = requests.post(
            rasa_endpoint, json={"sender": sender, "message": text}
        ).json()
        print('first:',results)
        for result in results:
            result=result['text']
            if result==detect_emotion:
                self.emotion_to_rasa()
            elif result==utter_think:
                self.publish_to_result(result)
                self.utter_think_func()
            else:
                self.publish_to_result(result)
        
    
    def publish_to_result(self, result):
        rospy.loginfo('Rasa response: "%s" ' % (result))
        self.pub.publish(result)

    def emotion_to_rasa(self):
        # self.emotion=rospy.wait_for_message('/fer/emotion', String, timeout=10)
        # content='I am feeling '+str(self.emotion.data)

        # for testing
        self.emotion='sad'
        content='I am feeling '+str(self.emotion)
        self.send_to_rasa(content)
    
    def utter_think_func(self):
        self.send_to_rasa('start chatgpt') 


if __name__ == '__main__':
    rospy.init_node('rasa')
    Rasa=RASA()

    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
