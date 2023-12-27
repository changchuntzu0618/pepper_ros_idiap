"""
The code includes a facial emotion recognition module that creates an emotion buffer, updating it 
every 0.1 seconds to store user emotions. Additionally, it functions as a service node, offering 
the most frequent user emotion during both user and Pepper's speaking periods.
"""
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
import numpy as np

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
        self.previous_detected_ppl=None

        self.pub_emotion=None

        self.is_speaking = set([])
        self.speaking_time = {}

        self.sub_image=rospy.Subscriber('/naoqi_driver_node/camera/front/image_raw',Image, self.fer)
        self.sub_tracker=rospy.Subscriber('/wp2/track',TrackedPersonArray, self.__track_cb, queue_size=10)
        self.sub_voise=rospy.Subscriber('/wp2/voice',VoiceActivityArray, self.__voice_cb, queue_size=10)

        self.pub= rospy.Publisher('~emotion', String, queue_size=1)

        self.srv = rospy.Service('get_emotion', Emotion, self.emotion_callback)
    
    def get_most_frequenct_emotion(self,all_emotion):
        """
        Determine and return the most frequently occurring emotion from a list of emotions. 
        Additionaly, the 'no_face' emotion is excluded from consideration. 

        Args:
            all_emotion (list): A list containing emotional labels, where 'no_face' may be present.

        Returns:
            str: The most frequently occurring emotion in the input list, or 'no_face' if the list
                is empty after excluding 'no_face'.
        """

        # get rid of no_face in all emotion
        all_emotion=[x for x in all_emotion if x != 'no_face']
        if all_emotion==[]:
            pub_emotion='no_face'
        else:
            #publish most frequency emotion
            pub_emotion=max(all_emotion,key=all_emotion.count)
        return pub_emotion 

        
    def emotion_callback(self,req):
        """
        Callback function for the "get_emotion" service to retrieve emotion information.

        This function is called in response to requests for emotion information. It processes
        the request based on the specified mode ('user' or 'pepper'). 'User' mode is getting user emotion 
        in the period when the user is talking. 'Pepper' mode is detect user emotion in the period when 
        the pepper is talking. Then it resonse with the most frequent emotion, time stamps, 
        and emotion probabilities.

        Args:
            req (GetEmotionRequest): The request object specifying the mode and, in the case of
                                    'pepper' mode, start and finish timestamps.

        Returns:
            EmotionResponse: The response object containing emotion information, including the
                            most frequent emotion, time stamps, and emotion probabilities.
        """
        if req.mode=='user':
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
                emotion_buffer=copy.deepcopy(self.emotion_buffer)
                emotion_time_stamp=[]
                emotion_prob=[]
                for time_stamp in emotion_buffer.keys():
                    # delete self.emotion_buffer which time_stamp is smaller than start_time
                    if time_stamp < start_time:
                        del self.emotion_buffer[time_stamp]
                    elif time_stamp >= start_time and time_stamp <= end_time:
                        for detect_emotion in emotion_buffer[time_stamp]:

                            # For single-person: assume there is only one person in the image
                            all_emotion.append(detect_emotion[0]['emotion'])

                            ## For multi-person TODO: make it work when the robot is moving (head, body), bec when robot is moving iou is not working
                            # emotion_box=detect_emotion['box']
                            # if emotion_box is None:
                            #     continue
                            # # print('face_box:',face_box)
                            # # print('emotion_box:',emotion_box)
                            # iou=self.calculate_iou(face_box, emotion_box)
                            # # print('iou:',iou)
                            # if iou>0.5:
                            #     all_emotion.append(detect_emotion['emotion'])
                            #     emotion_time_stamp.append(time_stamp)
                            #     emotion_prob.append(list(detect_emotion['emotion_prob'].values()))


                            # print(emotion_buffer)
                            # print(self.detected_ppl)
                self.detected_ppl=None
                print('all_emotion:',all_emotion)
                pub_emotion=self.get_most_frequenct_emotion(all_emotion)
                            
                resp=EmotionResponse()
                resp.emotion=pub_emotion

                self.pub_emotion=copy.copy(pub_emotion)

                rospy.loginfo('Publish emotion (User talk): "%s" ' % (pub_emotion))

                if emotion_time_stamp==[]:
                    resp.time_stamp=[0,0]
                else:
                    emotion_time_stamp.sort()
                    resp.time_stamp=[emotion_time_stamp[0],emotion_time_stamp[-1]]
                if emotion_prob==[]:
                        resp.emotion_prob= [0] * 7
                else:
                    resp.emotion_prob=list(np.mean(np.array(emotion_prob), axis=0))

                return resp
            else:
                rospy.loginfo('No person detected for emotion')
                resp=EmotionResponse()
                resp.emotion='no_person_detected'
                resp.time_stamp=[rospy.get_rostime().to_nsec(),rospy.get_rostime().to_nsec()]
                resp.emotion_prob= [0] * 7
                return resp
        elif req.mode=='pepper':
            start_time=req.start_stamp.to_nsec()
            end_time=req.finish_stamp.to_nsec()
            all_emotion=[]
            emotion_time_stamp=[]
            emotion_prob=[]
            emotion_buffer=copy.deepcopy(self.emotion_buffer)
            for time_stamp in emotion_buffer.keys():
                if time_stamp >= start_time and time_stamp <= end_time:
                    for detect_emotion in emotion_buffer[time_stamp]:
                        # Get the emotion of the person who is speaking/detected before
                        if self.previous_detected_ppl is not None:
                            # For single-person: assume there is only one person in the image
                            all_emotion.append(detect_emotion[0]['emotion'])

                            ## For multi-person TODO: make it work when the robot is moving (head, body), bec when robot is moving iou is not working
                            # face_box=self.previous_detected_ppl['box'] 
                            # x0, x1, y0, y1 = self.scale_bounding_box(self.image,
                            #                                                 face_box.h, face_box.w,
                            #                                                 face_box.height, face_box.width,
                            #                                                 face_box.image_height,
                            #                                                 face_box.image_width)
                            # face_box=[x0,y0,x1-x0,y1-y0]
                            # emotion_box=detect_emotion['box']
                            # if emotion_box is None:
                            #     # print('emotion box is none')
                            #     continue
                            # iou=self.calculate_iou(face_box, emotion_box)
                            # if iou>0.5:
                            #     all_emotion.append(detect_emotion['emotion'])
                            #     emotion_time_stamp.append(time_stamp)
                            #     emotion_prob.append(list(detect_emotion['emotion_prob'].values()))
                            ##else: print('iou<0.5')


            print('all_emotion:',all_emotion)
            pub_emotion=self.get_most_frequenct_emotion(all_emotion)

                        
            resp=EmotionResponse()
            resp.emotion=pub_emotion

            self.pub_emotion=copy.copy(pub_emotion)

            rospy.loginfo('Publish emotion (Pepper talk): "%s" ' % (pub_emotion))

            if emotion_time_stamp==[]:
                resp.time_stamp=[0,0]
            else:
                emotion_time_stamp.sort()
                resp.time_stamp=[emotion_time_stamp[0],emotion_time_stamp[-1]]
            if emotion_prob==[]:
                    resp.emotion_prob= [0] * 7
            else:
                resp.emotion_prob=list(np.mean(np.array(emotion_prob), axis=0))
            
            return resp
        
    def __track_cb(self, imsg):
        """
        Callback function of node "wp2/track" for getting face tracking information.

        This function is designed to be a callback for face tracking messages, extracting person IDs,
        and updating detected person (who is speaking and also being detcted) based on the intersection
        of tracked IDs with speaking IDs.

        Args:
            imsg (tracking_message): Input face tracking message containing data about tracked persons.
        """

        for i,p in enumerate(imsg.data):
            all_person_id=[]
            all_person_id.append(p.person_id)
            all_person_id.extend(p.alternate_ids)

            common=self.is_speaking & set(all_person_id)
            if common:
                speaking_id = list(common)[0]
                self.speaking_time[speaking_id]['box']=p.box
                self.detected_ppl={'start':self.speaking_time[speaking_id]['start'],
                                    'end':self.speaking_time[speaking_id]['end'],
                                    'box':self.speaking_time[speaking_id]['box']}
                self.previous_detected_ppl=copy.deepcopy(self.detected_ppl)

                self.speaking_time={}
                self.is_speaking.clear()
        

    def __voice_cb(self, imsg):
        """
        Callback function of node "wp2/voice" for getting speaking status.

        This function is designed to be a callback for voice messages, analyzing the speaking
        behavior of different individuals. It maintains a record of speaking time intervals for
        each person and identifies when they start and stop speaking.

        Args:
            imsg (voice_message): Input voice message containing data about speaking behavior.

        """
        for m in imsg.data:
            if m.person_id not in self.speaking_time:
                self.speaking_time[m.person_id] = {}
                self.speaking_time[m.person_id]['start_flag'] = False
                self.speaking_time[m.person_id]['end_flag'] = False
                self.speaking_time[m.person_id]['start'] = None
                self.speaking_time[m.person_id]['end'] = None
                self.speaking_time[m.person_id]['start_count'] = 0
                self.speaking_time[m.person_id]['end_count'] = 0
                self.speaking_time[m.person_id]['speaking_sequence'] = []

            if m.is_speaking:
                # print('Speaking!!!!')
                self.speaking_time[m.person_id]['end_count']=0
                self.speaking_time[m.person_id]['start_count'] += 1
                if self.speaking_time[m.person_id]['start_count'] == 1:
                    self.speaking_time[m.person_id]['start']=imsg.header.stamp.to_nsec()
                self.speaking_time[m.person_id]['speaking_sequence'].append(m.is_speaking)
                if self.speaking_time[m.person_id]['start_count'] >= 3:
                    self.speaking_time[m.person_id]['start_flag'] = True
            else:
                # print('Not Speaking!!!!')
                self.speaking_time[m.person_id]['start_count']=0
                self.speaking_time[m.person_id]['end_count'] += 1
                
                self.speaking_time[m.person_id]['speaking_sequence'].append(m.is_speaking)
                if self.speaking_time[m.person_id]['end_count'] >= 3:
                    if self.speaking_time[m.person_id]['start_flag']:
                        self.is_speaking.add(m.person_id)
                        self.speaking_time[m.person_id]['end']=imsg.header.stamp.to_nsec()
                        self.speaking_time[m.person_id]['end_flag'] = True
        
    def fer(self, data):
        """
        Main function for facial emotion recognition. This function performs facial emotion recognition on 
        the input image, updating the emotion buffer every 0.1 seconds with detected emotions, their 
        probabilities, and bounding box information. The function uses the DeepFace library for analysis.

        Detected emotions are simplified to 'sad', 'happy', 'neutral', or 'no_face' based on
        the dominant emotion category.

        Args:
            data (Image): Input image data from node "/naoqi_driver_node/camera/front/image_raw"

        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image=copy.deepcopy(cv_image)
        except CvBridgeError as e:
            print(e)

        time_stamp=(data.header.stamp).to_nsec()

        # store the emotion buffer every 0.1 second
        if time_stamp-self.last_time>=0.1*(10**9):
            self.emotion_buffer[time_stamp]=[]
        
        try:
            all_objs = DeepFace.analyze(img_path = cv_image,
                actions = ['emotion'],
                silent=True,
            )
        
            for objs in all_objs:
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
            # rospy.loginfo('No face detected')
            detected_emotion='no_face'
            if time_stamp-self.last_time>=0.1*(10**9):
                info={}
                info['emotion']=detected_emotion
                info['emotion_prob']=None
                info['box']=None
                self.emotion_buffer[time_stamp].append(info)

        cv_image=cv2.putText(cv_image,'publish emotion:'+str(self.pub_emotion),(10,40),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),1)

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
    
    def calculate_iou(self, box1, box2):
        '''
        Calculate the Intersection over Union (IoU) between two bounding boxes.

        IoU is a measure of the overlap between two bounding boxes. It is calculated as the
        ratio of the area of intersection to the area of union between the two boxes.

        Args:
            box1 (tuple): A tuple containing the coordinates of the first bounding box
                        in the format (x1, y1, width1, height1).
            box2 (tuple): A tuple containing the coordinates of the second bounding box
                        in the format (x2, y2, width2, height2).

        Returns:
            float: The Intersection over Union (IoU) between the two bounding boxes,
                ranging from 0.0 (no overlap) to 1.0 (complete overlap).

        Example:
            >>> box1 = (10, 20, 30, 40)
            >>> box2 = (25, 30, 30, 30)
            >>> iou_value = calculate_iou(box1, box2)
            >>> print(iou_value)
            0.42857142857142855
        '''
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        # Calculate the intersection coordinates
        x_intersection = max(x1, x2)
        y_intersection = max(y1, y2)
        w_intersection = min(x1 + w1, x2 + w2) - x_intersection
        h_intersection = min(y1 + h1, y2 + h2) - y_intersection

        # Check for non-overlapping boxes
        if w_intersection <= 0 or h_intersection <= 0:
            return 0.0

        # Calculate area of intersection and union
        area_intersection = w_intersection * h_intersection
        area_union = w1 * h1 + w2 * h2 - area_intersection

        # Calculate IoU
        iou = area_intersection / area_union

        return iou


if __name__ == '__main__':
    rospy.init_node('fer')
    Fer=FER()
    rospy.spin()
    cv2.destroyAllWindows()