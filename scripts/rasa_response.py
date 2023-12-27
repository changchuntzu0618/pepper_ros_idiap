#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from end_of_speech.msg import EndOfSpeech
from pepper_ros.srv import Emotion, EmotionResponse
from pepper_ros.msg import PepperTalkTime
import requests
import copy
import time

sender = "user"

rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"

utter_think='Let me think about it'
detect_emotion_pepper='detect emotion pepper'
detect_emotion_user='detect emotion user'

class RASA:
    def __init__(self):
        self.sub=rospy.Subscriber('/end_of_speech/eos', EndOfSpeech, self.rasa_response)
        
        self.pub= rospy.Publisher('~rasa_response', String, queue_size=1)

        self.srv_emotion = rospy.ServiceProxy('get_emotion', Emotion)

        self.talktime_buffer=None

    def rasa_response(self, eos_msg):
        """
        Process the end-of-speech message received from speech recognition node "/end_of_speech/eos", and 
        and forward the user's utterance to a Rasa system for further processing.

        Args:
            eos_msg (EndOfSpeech): End-of-speech message containing the final user utterance and confidence.
        """
        self.text=eos_msg.final_utterance
        if self.text=='' or self.text==' ':
            return
        self.confidence=eos_msg.confidence
        self.header=eos_msg.header
        if self.talktime_buffer is not None:
            # if the message is less than 4 seconds of previous messgae, then ignore it and consider it as pepper is talking
            if self.header.stamp.to_nsec()<=self.talktime_buffer.start_stamp.to_nsec() or self.header.stamp.to_nsec()-self.talktime_buffer.finish_stamp.to_nsec()<=4*10**9:
                print('It is pepper talking')
                return
        
        
        rospy.loginfo('Heard User said: "%s", with confidence "%f" ' % (self.text, self.confidence))

        self.send_to_rasa(self.text)
        
       
    def send_to_rasa(self, text):
        """
        Send the user's utterance to a Rasa system and get response.

        Args:
            text (str): User's utterance.
        """
        results = requests.post(
            rasa_endpoint, json={"sender": sender, "message": text}
        ).json()
        for result in results:
            result=result['text']
            if result==detect_emotion_user:
                self.get_emotion_user_talk()
            elif result==detect_emotion_pepper:
                self.get_emotion_pepper_talk()
            elif result==utter_think:
                self.publish_to_result(result)
                self.utter_think_func()
            else:
                self.publish_to_result(result)
            
    
    def publish_to_result(self, result):
        """
        Publish the response from Rasa to the topic "/rasa/rasa_response" which send the text to
        Pepper, and Pepper will say it.

        Args:
            result (str): Response from Rasa.
        """
        rospy.loginfo('Rasa response: "%s" ' % (result))
        self.pub.publish(result)
        self.talktime_buffer=rospy.wait_for_message('/pepper_say/talk_time', PepperTalkTime, timeout=10)
        self.talktime_buffer_send=copy.deepcopy(self.talktime_buffer)
        self.talktime_buffer_send.start_stamp=rospy.Time.from_sec((self.talktime_buffer.start_stamp.to_sec()+self.talktime_buffer.finish_stamp.to_sec())/2)
        self.talktime_buffer_send.finish_stamp=rospy.Time.from_sec(self.talktime_buffer.finish_stamp.to_sec()+2)
        
    def get_emotion_pepper_talk(self):
        """
        Get the emotion of user when Pepper is talking.
        """
        time.sleep(3) #wait for 3 second
        emotion=self.srv_emotion('pepper',self.talktime_buffer_send.start_stamp, self.talktime_buffer_send.finish_stamp).emotion
        rospy.loginfo('Get emotion (pepper talk): '+str(emotion))
        self.emotion_to_rasa(emotion)
    
    def get_emotion_user_talk(self):
        """
        Get the emotion of user when user is talking.
        """
        emotion = self.srv_emotion('user',None,None).emotion
        rospy.loginfo('Get emotion (user talk): '+str(emotion))
        self.emotion_to_rasa(emotion)

    def emotion_to_rasa(self, emotion):
        """
        Send the emotion to Rasa in the form of a sentence.

        Args:
            emotion (str): Emotion of user.
        """
        if emotion == 'no_person_detected' or emotion == 'no_face':
            emotion = 'neutral'

        content='I am feeling '+str(emotion)
        rospy.loginfo('Send to RASA:' + content)
        self.send_to_rasa(content)
    
    def utter_think_func(self):
        self.send_to_rasa('start chatgpt')
        

if __name__ == '__main__':
    rospy.init_node('rasa')
    Rasa=RASA()

    rospy.spin()
