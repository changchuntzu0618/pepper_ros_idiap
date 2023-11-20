import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pepper_ros.msg import EndOfSpeech, MummerAsr,AudioBuffer
import cv2
from cv_bridge import CvBridge, CvBridgeError
from deepface import DeepFace
import os

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
        self.bridge = CvBridge()
        self.flag=0
        self.detected_emotions=[]
        self.sub1=rospy.Subscriber('/mummer_asr/result', MummerAsr, self.call_back1)
        self.sub2=rospy.Subscriber('/end_of_speech/eos', EndOfSpeech,self.call_back2)
        # self.sub1 = rospy.Subscriber("/noise_filter_node/result", AudioBuffer, self.call_back1, queue_size=1)
        self.sub3=rospy.Subscriber('/naoqi_driver_node/camera/front/image_raw',Image, self.fer)
        self.pub= rospy.Publisher('~emotion', String, queue_size=1)
        self.pub2=rospy.Publisher('~flag', String, queue_size=1)
        
    def call_back1(self, data):
        self.flag=1
        
    def call_back2(self, data):
        self.flag=2

    def publish_flag(self):
        self.pub2.publish(str(self.flag))
        
    def fer(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.flag==1:
            
            cv2.imwrite(TMP, cv_image)
            try:
                objs = DeepFace.analyze(img_path = TMP, 
                    actions = ['emotion'],
                )
                detected_emotion=objs[0]['dominant_emotion']
                if detected_emotion in SAD_EMOTION:
                    detected_emotion='sad'
                elif detected_emotion in HAPPY_EMOTION:
                    detected_emotion='happy'
                else: 
                    detected_emotion='neutral'
                self.detected_emotions.append(detected_emotion)
                rospy.loginfo('Detected emotion: "%s" ' % (detected_emotion))
                # self.pub.publish(detected_emotion)

                bbox=objs[0]['region']
                x1=int(bbox['x'])
                y1=int(bbox['y'])
                w=int(bbox['w'])
                h=int(bbox['h'])
                cv_image=cv2.rectangle(cv_image, (x1, y1), (x1+w, y1+h), color=(255,0,0), thickness=2)
                cv_image=cv2.putText(cv_image,str(detected_emotion),(x1,y1),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),1) 
                # print(objs)

            except:
                rospy.loginfo('No face detected')
                detected_emotion='no_face'
                self.detected_emotions.append(detected_emotion)
            

        elif self.flag==2:
            # TODO: check with Emmanual this method, or ask for his suggestion for better method
            # Since now flag 1 will happend in the bery end of the speech, so between flag 1-flag 2, it is actually the period of
            # user finish talking and waiting for reply
            num_use=int((len(self.detected_emotions)/2))
            if num_use==0: num_use=1   
            self.detected_emotions=self.detected_emotions[:num_use]

            # Get the most frequent emotion in this period
            pub_emotion=max(self.detected_emotions,key=self.detected_emotions.count)
            self.pub.publish(pub_emotion)
            print(len(self.detected_emotions))
            print(self.detected_emotions)
            self.detected_emotions=[]
            self.flag=0


        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

            


if __name__ == '__main__':
    rospy.init_node('fer')
    Fer=FER()
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        Fer.publish_flag()
        r.sleep()


    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
    cv2.destroyAllWindows()
