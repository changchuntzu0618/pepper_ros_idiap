import qi

ip='192.168.1.4'
port=9559

session = qi.Session()
session.connect("tcp://" + ip + ":" + str(port))
tts=session.service("ALTextToSpeech")
tts.stopAll()