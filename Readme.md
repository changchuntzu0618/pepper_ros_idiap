# Idiap semster project: HRI Pepper with RASA via ROS

Useful tutorial link: 
    Naoqi Python API: http://doc.aldebaran.com/2-5/naoqi/index.html
    
## Pre-request
Computer Master: A computer with Ubuntu 16 with ROS Kinetic

    ros package:

        naoqi_bridge

        naoqi_bridge_msgs

        naoqi_driver

        nao_interaction

        mummer_asr

        noise_reduction

        pepper_ros_idiap (this repo)

Computer Remote: Anothor computer (Tested on Ubuntu 20.04 with ROS Noetic)

    rasa installed

    ros package:
        perception_msgs (from idiap mummer gitlab)

        pepper_ros_idiap (this repo)

## Run
### Set up network for ROS communication between Master and Remote
### Main
On Computer Master (idiap big computer)
```
#Mummer system
# run person_manager/demos/demo_perception.sh (it will open a tmux)
bash /home/mummer/mummer_ws/src/person_manager/demos/demo_perception.sh

#Only need to runing following command in all three tmux window

# Window 1:robot (crtl-b -> 1)
# Get the sensor data from Pepper
DISPLAY=:0 roslaunch naoqi_driver naoqi_driver.launch network_interface:=enp5s0
#Align color image and depth image
DISPLAY=:0 roslaunch naoqi_driver register_depth.launch

# Window 2: perception (crtl-b -> 2)
#Before running following command source env first
source ~/mummer_ws/devel/setup.bash
# perception manager code it will give out topic for face tracker and is_speaking
#change manager_visu to 0 to close the visulization window
DISPLAY=:0 roslaunch person_manager perception.launch naoqi:=1 tracker_delay:=50 tracker_scale:=0.5 tracker_detector:=openheadpose tracker_particles:=50 tracker_min_height:=0.12 manager_keep_threshold:=0.02 tracker_visu:=1 manager_visu:=1 with_audio:=1

#Open seperate terminal
#Mummur ASR: for speech-to-text, set use_extra_mic to ture for using external microphone 
roslaunch mummer_asr_launch mummer_asr.launch use_extra_mic:=false target_language:=en-UK

# To start Listen what user said
rosservice call /mummer_asr/resume
```

On Computer Remote
```
# open rasa and rasa actions in two terminals
rasa run

#Remember to "export OPENAI_API_KEY=..." beofre running following command if using chatgpt in actions
rasa run actions

# get the result of ASR and send to rasa and then publish response
rosrun pepper_ros rasa_response.py 

# strat facial expression recognition and publish deetcted emotion of user in every speech
rosrun pepper_ros fer.py
```

On Computer Master
```
#Send command to Pepper
rosrun pepper_ros pepper_talk.py
```