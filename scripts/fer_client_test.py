import rospy
from pepper_ros.srv import Emotion, EmotionResponse

if __name__ == "__main__":
    rospy.init_node('caller')

    srv_emotion = rospy.ServiceProxy('get_emotion', Emotion)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        emotion = srv_emotion('user',None,None).emotion
        rospy.loginfo('Get emotion: '+str(emotion))

        r.sleep()