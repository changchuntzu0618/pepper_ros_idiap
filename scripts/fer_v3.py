import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pepper_ros.msg import EndOfSpeech, MummerAsr,AudioBuffer
from pepper_ros.srv import Emotion, EmotionResponse
from perception_msgs.msg import TrackedPersonArray
from perception_msgs.msg import VoiceActivityArray
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
        self.image=None
        self.face_image=None
        self.detected_ppl=None

        self.is_speaking = set([])
        self.speaking_time = {}

  

        self.sub_image=rospy.Subscriber('/naoqi_driver_node/camera/front/image_raw',Image, self.fer)
        self.sub_tracker=rospy.Subscriber('/wp2/track',TrackedPersonArray, self.__track_cb, queue_size=10)
        self.sub_voise=rospy.Subscriber('/wp2/voice',VoiceActivityArray, self.__voice_cb, queue_size=10)

        self.pub= rospy.Publisher('~emotion', String, queue_size=1)

        self.srv = rospy.Service('get_emotion', Emotion, self.emotion_callback)
        
    def emotion_callback(self,req):
        if self.detected_ppl is not None:
            all_emotion=[]
            # get the person strat speaking time and end speaking time
            start_time=self.detected_ppl['start']
            end_time=self.detected_ppl['end']
            face_box=self.detected_ppl['box'] 
            x0, x1, y0, y1 = self.scale_bounding_box(self.image,
                                                            face_box.h, face_box.w,
                                                            face_box.height, face_box.width,
                                                            face_box.image_height,
                                                            face_box.image_width)
            face_box=[x0,y0,x1-x0,y1-y0]
            # print(self.emotion_buffer)
            for time_stamp in self.emotion_buffer.keys():
                if time_stamp >= start_time and time_stamp <= end_time:
                    for detect_emotion in self.emotion_buffer[time_stamp]:
                        # print('detect_emotion:',detect_emotion)
                        emotion_box=detect_emotion['box']
                        # TODO: use a function to compare the face_box (face from face track, the face who is speaking) and emotion_box(face whihch the emotion detected)
                        # if self.are_bounding_boxes_almost_same(face_box, emotion_box):
                        all_emotion.append(detect_emotion['emotion'])

            self.detected_ppl=None
            print('all_emotion:',all_emotion)
            # get the most frequent emotion
            def emotion_priority(emotion):
                # Define the priority of emotions
                if emotion == "no_face":
                    return 0
                else:
                    return 1
            # Find the emotion with the maximum count, considering the custom sorting key
            pub_emotion = max(set(all_emotion), key=lambda x: (all_emotion.count(x), emotion_priority(x)))
            # pub_emotion=max(all_emotion,key=all_emotion.count)
                        
            resp=EmotionResponse()
            resp.emotion=pub_emotion
            rospy.loginfo('Publish emotion: "%s" ' % (pub_emotion))

            # TODO: add time_stamp and emotion_prob
            # resp.time_stamp=time_stamp
            # resp.emotion_prob=emotion_prob
            return resp
        else:
            rospy.loginfo('No person detected')
            resp=EmotionResponse()
            resp.emotion='no_person_detected'
            return resp
        
    def __track_cb(self, imsg):
        """
        """
        for i,p in enumerate(imsg.data):
            all_person_id=[]
            all_person_id.append(p.person_id)
            all_person_id.extend(p.alternate_ids)
            common=self.is_speaking & set(all_person_id)
            if common:
                speaking_id = list(common)[0]
                
                self.speaking_time[speaking_id]['box']=p.box
                print('final:',self.speaking_time[speaking_id])
                self.detected_ppl={'start':self.speaking_time[speaking_id]['start'],
                                    'end':self.speaking_time[speaking_id]['end'],
                                    'box':self.speaking_time[speaking_id]['box']}
                self.speaking_time={}
        

    def __voice_cb(self, imsg):
        """
        """

        self.is_speaking.clear()
        
        for m in imsg.data:
            if m.person_id not in self.speaking_time:
                self.speaking_time[m.person_id] = {}
                self.speaking_time[m.person_id]['start'] = None
                self.speaking_time[m.person_id]['end'] = None
                self.speaking_time[m.person_id]['start_count'] = 0
                self.speaking_time[m.person_id]['end_count'] = 0

            if m.is_speaking:
                # print('Speaking!!!!')
                self.speaking_time[m.person_id]['end_count']=0
                self.speaking_time[m.person_id]['start_count'] += 1
                if self.speaking_time[m.person_id]['start_count'] >= 3:
                    self.speaking_time[m.person_id]['start']=imsg.header.stamp.to_nsec()
            else:
                # print('Not Speaking!!!!')
                self.speaking_time[m.person_id]['start_count']=0
                self.speaking_time[m.person_id]['end_count'] += 1
                if self.speaking_time[m.person_id]['end_count'] >= 3:
                    if self.speaking_time[m.person_id]['start'] is not None:
                        self.is_speaking.add(m.person_id)
                        self.speaking_time[m.person_id]['end']=imsg.header.stamp.to_nsec()
        # print(self.speaking_time)
        
    def fer(self, data):
        # TODO: clean emotion_buffer after some times (maybe after user finish a sentence)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image=copy.deepcopy(cv_image)
        except CvBridgeError as e:
            print(e)

        cv2.imwrite(TMP, cv_image)
        time_stamp=(data.header.stamp).to_nsec()

        # store the emotion buffer every 0.1 second
        if time_stamp-self.last_time>=0.1*(10**9):
            self.emotion_buffer[time_stamp]=[]
        
        try:
            all_objs = DeepFace.analyze(img_path = TMP, 
                actions = ['emotion'],
                silent=True,
            )
        
            for objs in all_objs:
                # print('objs:',objs)
                detected_emotion=objs['dominant_emotion']
                if detected_emotion in SAD_EMOTION:
                    detected_emotion='sad'
                elif detected_emotion in HAPPY_EMOTION:
                    detected_emotion='happy'
                else: 
                    detected_emotion='neutral'
                #'emotion': {'angry': 1.620439812541008, 'disgust': 2.1869497324189524e-07, 'fear': 0.6565135437995195, 'happy': 0.013205509458202869, 'sad': 2.0564544945955276, 'surprise': 0.009354727808386087, 'neutral': 95.64403295516968}
                emotion_prob=objs['emotion'] 

                bbox=objs['region']
                x1=int(bbox['x'])
                y1=int(bbox['y'])
                w=int(bbox['w'])
                h=int(bbox['h'])

                # store the emotion buffer every 0.1 second
                if time_stamp-self.last_time>=0.1*(10**9):
                    info={}
                    info['emotion']=detected_emotion
                    info['emotion_prob']=emotion_prob
                    info['box']=[x1,y1,w,h]
                    self.emotion_buffer[time_stamp].append(info)
                
                cv_image=cv2.rectangle(cv_image, (x1, y1), (x1+w, y1+h), color=(255,0,0), thickness=2)
                cv_image=cv2.putText(cv_image,str(detected_emotion),(x1,y1),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),1) 

        except:
            rospy.loginfo('No face detected')
            detected_emotion='no_face'
            if time_stamp-self.last_time>=0.5*(10**9):
                info={}
                info['emotion']=detected_emotion
                info['emotion_prob']=None
                info['box']=None
                self.emotion_buffer[time_stamp].append(info)

        # print(self.emotion_buffer)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        if time_stamp-self.last_time>=0.1*(10**9):
            self.last_time=copy.copy(time_stamp)
        
    def scale_bounding_box(self,image,
                       h, w, height, width,
                       image_height=-1, image_width=-1):
        """Adapt the coordinates of the input bounding box.


        Args:
            image  : Image in which to project the bounding box.
            h      :
            w      :
            height :
            width  :
            image_height : Height of image in which the bounding box is.
            image_width  :

        Returns:
            x0, x1, y0, y1 : Start and end points of bounding box

        """
        if image_height < 0:
            image_height = image.shape[0]
        if image_width < 0:
            image_width = image.shape[1]
        W = float(image.shape[1])/float(image_width)
        H = float(image.shape[0])/float(image_height)
        x0 = int(w*W)
        y0 = int(h*H)
        x1 = int((w + width)*W)
        y1 = int((h + height)*H)

        return x0, x1, y0, y1

    def are_bounding_boxes_almost_same(self, box1, box2, tolerance=0.1):
        """
        Check if two bounding boxes are almost the same.

        Parameters:
        - box1, box2: Tuple or list representing (x, y, width, height) of the bounding box.
        - tolerance: Tolerance level for considering the boxes almost the same.

        Returns:
        - True if the boxes are almost the same, False otherwise.
        """
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        # Calculate the percentage difference in each dimension
        percent_diff_x = abs(x1 - x2) / max(w1, w2)
        percent_diff_y = abs(y1 - y2) / max(h1, h2)
        percent_diff_w = abs(w1 - w2) / max(w1, w2)
        percent_diff_h = abs(h1 - h2) / max(h1, h2)

        # Check if all differences are within the tolerance
        return (
            percent_diff_x <= tolerance
            and percent_diff_y <= tolerance
            and percent_diff_w <= tolerance
            and percent_diff_h <= tolerance
        )

            


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
