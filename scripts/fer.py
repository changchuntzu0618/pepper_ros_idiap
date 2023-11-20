import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from deepface import DeepFace
import os

BASE=os.path.dirname(os.path.abspath(__file__))
TMP=os.path.join(BASE,'tmp.jpg')
SAD_EMOTION=['angry', 'fear', 'neutral', 'sad', 'disgust']
HAPPY_EMOTION=['happy', 'surprise']

class FER:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub=rospy.Subscriber('/naoqi_driver_node/camera/front/image_raw',Image, self.fer)
        self.pub= rospy.Publisher('~emotion', String, queue_size=1)
        

    def fer(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imwrite(TMP, cv_image)
        try:
            objs = DeepFace.analyze(img_path = TMP, 
                actions = ['emotion']
            )
            detected_emotion=objs[0]['dominant_emotion']
            if detected_emotion in SAD_EMOTION:
                detected_emotion='sad'
            elif detected_emotion in HAPPY_EMOTION:
                detected_emotion='happy'
            rospy.loginfo('Detected emotion: "%s" ' % (detected_emotion))
            self.pub.publish(detected_emotion)

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
            self.pub.publish('no_face')
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

            


if __name__ == '__main__':
    rospy.init_node('fer')
    Fer=FER()

    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
    cv2.destroyAllWindows()
