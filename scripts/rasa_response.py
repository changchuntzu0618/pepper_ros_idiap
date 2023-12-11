#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from end_of_speech.msg import EndOfSpeech
from pepper_ros.srv import Emotion, EmotionResponse
from pepper_ros.msg import PepperTalkTime
import requests

sender = "user"

rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"

utter_think='Let me think about it'
detect_emotion_pepper='detect emotion pepper'
detect_emotion_user='detect emotion user'

class RASA:
    def __init__(self):
        # self.sub=rospy.Subscriber('/human_dialogue', String, self.rasa_response)
        self.sub=rospy.Subscriber('/end_of_speech/eos', EndOfSpeech, self.rasa_response)
        self.sub2=rospy.Subscriber('/pepper_say/talk_time', PepperTalkTime, self.get_pepper_talk_time)
        
        self.pub= rospy.Publisher('~rasa_response', String, queue_size=1)
        self.get_emotion=False

        self.srv_emotion = rospy.ServiceProxy('get_emotion', Emotion)

        self.talktime_buffer=None

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
        # print('first:',results)
        for result in results:
            result=result['text']
            if result==detect_emotion_user:
                self.get_emotion_user_talk()
            elif result==detect_emotion_pepper:
                print('get emotion pepper')
                self.get_emotion_pepper_talk()
            elif result==utter_think:
                self.publish_to_result(result)
                self.utter_think_func()
            else:
                self.publish_to_result(result)
    
  
    
    def publish_to_result(self, result):
        rospy.loginfo('Rasa response: "%s" ' % (result))
        self.pub.publish(result)
        self.send_time=rospy.Time.now()
    
    def get_pepper_talk_time(self, talk_time):
        self.talktime_buffer=talk_time
    
    def get_emotion_pepper_talk(self):
        emotion=self.srv_emotion('pepper',self.talktime_buffer.start_stamp, self.talktime_buffer.finish_stamp).emotion
        # for debugging
        # emotion='sad'
        rospy.loginfo('Get emotion (pepper talk): '+str(emotion))
        self.emotion_to_rasa(emotion)
    
    def get_emotion_user_talk(self):
        # self.emotion=rospy.wait_for_message('/fer/emotion', String, timeout=10)
        # content='I am feeling '+str(self.emotion.data)
        emotion = self.srv_emotion('user',None,None).emotion
        rospy.loginfo('Get emotion (user talk): '+str(emotion))
        self.emotion_to_rasa(emotion)

    def emotion_to_rasa(self, emotion):
        if emotion == 'no_person_detected' or emotion == 'no_face':
            emotion = 'neutral'
        # # for testing
        # self.emotion='sad'

        content='I am feeling '+str(emotion)
        rospy.loginfo('Send to RASA:' + content)
        self.send_to_rasa(content)
    
    def utter_think_func(self):
        print('start chatgpt')
        self.send_to_rasa('start chatgpt')
        

if __name__ == '__main__':
    rospy.init_node('rasa')
    Rasa=RASA()

    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
