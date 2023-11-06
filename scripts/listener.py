from naoqi import ALProxy
import requests

import rospy
from std_msgs.msg import String

sender = "user"

rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"

tts = ALProxy("ALTextToSpeech", "localhost", 36751)

def send_to_rasa(data):

    text = data.data
    rospy.loginfo('Heard message: "%s"  -- sending it to Rasa...' % text)

    results = requests.post(
        rasa_endpoint, json={"sender": sender, "message": text}
    ).json()

   

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, send_to_rasa)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
