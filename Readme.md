# Interaction Manager for Human-Robot Interactions

Nowadays, an increasing number of social robots are becoming common in our daily lives, serving various purposes like service robots in restaurants or hotels and guided robots in museums or airports. The conversation is the main part when interacting with these robots. While current technologies enable robots to easily recognize what people say and give out a correct response, natural human conversation and interaction include not only verbal behavior but also non-verbal aspects such as gestures, emotions, and gaze direction. By incorporating both verbal and non-verbal behaviors, conversations become more engaging, contributing to an enhanced user experience.

In this project, we aim to enhance the user's communication experience with social robots by developing a multimodal communication system. Beyond the dialogue system employed for human-robot interactions, we introduce a non-verbal behavior, user emotions, into the dialogue, allowing the robot to express and respond to the user's emotional state. Furthermore, we establish a fully functional real-time robotics system for human-robot interaction. Users can engage in a full dialogue with the robot, providing a neutral and immersive conversational experience. A demonstration video is available at [here](https://youtu.be/GYSCGwcJkY4).


This project used [rasa](https://rasa.com/), an open-source conversational AI platform, and [deepFace](https://github.com/serengil/deepface), a Python framework designed for face recognition and facial attribute analysis.
    
## Pre-request
Computer Master: A computer with Ubuntu 16 and ROS Kinetic

    ros package:

        naoqi_bridge (from idiap mummer)

        naoqi_bridge_msgs (from idiap mummer)

        naoqi_driver (from idiap mummer)

        nao_interaction (from idiap mummer)

        mummer_asr (from idiap mummer)

        noise_reduction (from idiap mummer)

        pepper_ros_idiap (this repo)

Computer Remote: Anothor computer (Tested on Ubuntu 20.04 with ROS Noetic)

    ros package:
        perception_msgs (from idiap mummer)

        pepper_ros_idiap (this repo)

```
pip3 install rasa
pip3 install deepface
```

## Get package
```
git clone --recursive https://github.com/changchuntzu0618/pepper_ros_idiap.git
```

## Build
```

cd your_ws
catkin_make

```

## Run
### Set up network for ROS communication between Master and Remote
```
roscore
```
### Main
#### On Computer Master

Here for testing, we used the idiap computer which has ubuntu 16 and already contained Mummer system inside. Some of the package are from Idiap Mummer system directely from computer.
```
# From Mummer system
# run person_manager/demos/demo_perception.sh (it will open a tmux)
bash /home/mummer/mummer_ws/src/person_manager/demos/demo_perception.sh

# Only need to runing following command in all three tmux window
# Window 1:robot (crtl-b -> 1)
# Get the sensor data from Pepper
DISPLAY=:0 roslaunch naoqi_driver naoqi_driver.launch network_interface:=enp5s0
# Align color image and depth image
DISPLAY=:0 roslaunch naoqi_driver register_depth.launch

# Window 2: perception (crtl-b -> 2)
# Before running following command source env first
source ~/mummer_ws/devel/setup.bash
# perception manager code it will give out topic for face tracker and is_speaking
# change manager_visu to 0 to close the visulization window
DISPLAY=:0 roslaunch person_manager perception.launch naoqi:=1 tracker_delay:=50 tracker_scale:=0.5 tracker_detector:=openheadpose tracker_particles:=50 tracker_min_height:=0.12 manager_keep_threshold:=0.02 tracker_visu:=1 manager_visu:=1 with_audio:=1


# Open a new terminal
# Mummur ASR: for speech-to-text, set use_extra_mic to ture for using external microphone 
roslaunch mummer_asr_launch mummer_asr.launch use_extra_mic:=false target_language:=en-UK

# To start Listen what user said
rosservice call /mummer_asr/resume
```

```
# Open a new terminal
# Send command to Pepper
cd ../pepper_ros_idiap
rosrun pepper_ros pepper_talk.py
```

#### On Computer Remote (local computer which contain this repo)
```
cd ../pepper_ros_idiap

# open a new terminal
cd rasa
rasa run

# open a new termial
# Remember to "export OPENAI_API_KEY=..." beofre running following command if using chatgpt in actions
cd rasa
rasa run actions

# open a new terminal
# get the result of ASR and send to rasa and then publish response
rosrun pepper_ros rasa_response.py 

# open a new terminal
# strat facial expression recognition and publish deetcted emotion of user in every speech
rosrun pepper_ros fer.py
```


## Description of main scripts
#### [rasa_resonse.py](./scripts/rasa_response.py)

The code covers the section responsible for interfacing with Rasa, the conversational system utilized in this project. It transmits the recognized user utterance and emotion to Rasa, retrieves the response, and then sends out that response.

#### [fer.py](./scripts/fer.py)

The code includes a facial emotion recognition module that creates an emotion buffer, updating it every 0.1 seconds to store user emotions. Additionally, it functions as a service node, offering the most frequent user emotion during both user and Pepper's speaking periods.

#### [pepper_talk.py](./scripts/pepper_talk.py)

The code includes the section for sending text and "say" commands to Pepper. It receives the text from another node, sends it to Pepper, and Pepper will articulate it either in the 'ALTextToSpeech' mode (just saying it) or in the 'ALAnimatedSpeech' mode (accompanied by movements while talking). Moreover, it publishes information about the time period during which Pepper is talking.