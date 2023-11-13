# HRI: Pepper with RASA a multimodel conversasional system
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

    ros pachage:
    
        pepper_ros_idiap (this repo)

## Run
### Set up network for ROS communication between Master and Remote
### Main
On Computer Master
```
# Get the sensor data from Pepper
roslaunch naoqi_driver naoqi_driver.launch network_interface:=enp5s0

#Mummur ASR: for speech-to-text
roslaunch mummer_asr_launch mummer_asr.launch use_extra_mic:=false target_language:=en-UK

# To start Listen what user said
rosservice call /mummer_asr/resume
```

On Computer Remote
```
# open rasa and rasa actions in two terminals
rasa run
rasa run actions

# get the result of ASR and send to rasa and then publish response
rosrun pepper_ros rasa_response.py 
```

On Computer Master
```
#Send command to Pepper
rosrun pepper_ros pepper_talk.py
```