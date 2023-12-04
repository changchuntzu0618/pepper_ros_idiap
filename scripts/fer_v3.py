import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pepper_ros.msg import EndOfSpeech, MummerAsr,AudioBuffer
from pepper_ros.srv import Emotion, EmotionResponse
import cv2
from cv_bridge import CvBridge, CvBridgeError
from deepface import DeepFace
import os
import copy

BASE=os.path.dirname(os.path.abspath(__file__))
TMP=os.path.join(BASE,'tmp.jpg')
SAD_EMOTION=['angry', 'fear', 'sad', 'disgust']
HAPPY_EMOTION=['happy', 'surprise']
NEUTRAL_EMOTION=['neutral']

backends = [
  'opencv', 
  'ssd', 
  'dlib', 
  'mtcnn', 
  'retinaface', 
  'mediapipe',
  'yolov8',
  'yunet',
  'fastmtcnn',
]

class FER:
    def __init__(self):
        self.rate = rospy.Rate(1) # ROS Rate at 1Hz (1 cycle/sec)
        self.bridge = CvBridge()
        self.emotion_buffer={}
        self.last_time=0

        self.sub3=rospy.Subscriber('/naoqi_driver_node/camera/front/image_raw',Image, self.fer)
        # self.pub= rospy.Publisher('~emotion', String, queue_size=1)

        # self.srv = rospy.Service('get_emotion', Emotion, self.emotion_callback)
        
    def emotion_callback(self,req):
        # get the person strat speaking time and end speaking time
        # start_stamp=req.start_stamp
        # end_stamp=req.end_stamp


        # detect_emotion=self.emotion_buffer[time_stamp]
        # emotion=detect_emotion['emotion']
        # emotion_prob=detect_emotion['emotion_prob']
        # resp=EmotionResponse()
        # resp.time_stamp=time_stamp
        # resp.emotion=emotion
        # resp.emotion_prob=emotion_prob
        # return resp
        pass
        
        
    def fer(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imwrite(TMP, cv_image)
        time_stamp=(data.header.stamp).to_nsec()
        print(time_stamp)
        print(self.last_time)
        # store the emotion buffer every 0.5 second
        if time_stamp-self.last_time>=0.5*(10**9):
            self.emotion_buffer[time_stamp]={}
        try:
            objs = DeepFace.analyze(img_path = TMP, 
                actions = ['emotion'],
                silent=True,
            )
            detected_emotion=objs[0]['dominant_emotion']
            if detected_emotion in SAD_EMOTION:
                detected_emotion='sad'
            elif detected_emotion in HAPPY_EMOTION:
                detected_emotion='happy'
            else: 
                detected_emotion='neutral'
            #'emotion': {'angry': 1.620439812541008, 'disgust': 2.1869497324189524e-07, 'fear': 0.6565135437995195, 'happy': 0.013205509458202869, 'sad': 2.0564544945955276, 'surprise': 0.009354727808386087, 'neutral': 95.64403295516968}
            emotion_prob=objs[0]['emotion'] 

            # store the emotion buffer every 0.5 second
            if time_stamp-self.last_time>=0.5*(10**9):
                print('store emotion buffer')
                self.emotion_buffer[time_stamp]['emotion']=detected_emotion
                self.emotion_buffer[time_stamp]['emotion_prob']=emotion_prob

            bbox=objs[0]['region']
            x1=int(bbox['x'])
            y1=int(bbox['y'])
            w=int(bbox['w'])
            h=int(bbox['h'])
            cv_image=cv2.rectangle(cv_image, (x1, y1), (x1+w, y1+h), color=(255,0,0), thickness=2)
            cv_image=cv2.putText(cv_image,str(detected_emotion),(x1,y1),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),1) 


        except:
            rospy.loginfo('No face detected')
            detected_emotion='no_face'
            if time_stamp-self.last_time>=0.5*(10**9):
                self.emotion_buffer[time_stamp]['emotion']=detected_emotion
                self.emotion_buffer[time_stamp]['emotion_prob']=None

        print(self.emotion_buffer)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        if time_stamp-self.last_time>=0.5*(10**9):
            self.last_time=copy.copy(time_stamp)
        

            


if __name__ == '__main__':
    rospy.init_node('fer')
    Fer=FER()
    # r = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     Fer.publish_flag()
    #     r.sleep()


    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
    cv2.destroyAllWindows()
