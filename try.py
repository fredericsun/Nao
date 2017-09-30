import xml.etree.ElementTree as ET
import threading
import time
import random
import signal
import sys
import numpy as np

from gaze import Gaze
from gesture import Gesture
from naoqi import ALModule
from naoqi import ALBroker
from naoqi import ALProxy

##########################
# GLOBAL VARIABLES
##########################
IP = "nao.local"
PORT = 9559
wordList = ["hello", "nice to meet you", "bye", "see you", "I am finished", "I have a question", "good", "can you repeat?", "I am ready"]
QuestionList = []
linkList = []
AnswerList = []

# keep a list of all subscribers so that if the script is killed, we can unsubscribe from everything
subscribedTo = {}

robot = None
speechMem = None

##########################
# EXIT HANDLING
##########################
def handle_exit(signum, frame):
    for s in subscribedTo:
        s.unsubscribe(subscribedTo[s])

    print "All proxies terminated."
    sys.exit()

signal.signal(signal.SIGTERM, handle_exit)
signal.signal(signal.SIGINT, handle_exit)

##########################
# NAO FUNCTIONALITY
##########################
class Nao(ALModule):
    def __init__(self, name):
        self.name = name
        ALModule.__init__(self, self.name)
        self.gaze = Gaze()
        self.gesture = Gesture()
        self.loop_lock = False
        autoMove = ALProxy("ALAutonomousMoves", IP, PORT)
        autoMove.setExpressiveListeningEnabled(False)

    def getName(self):
        return self.name

    def speechRecognition(self, sec):
        print "Started speech recognition"
        speechRec = ALProxy("ALSpeechRecognition", IP, PORT)
        speechRec.setAudioExpression(False)
        speechRec.setLanguage("English")
        speechRec.setVocabulary(wordList, False)

        print "Subscribing to events"
        # update the list of subscribed
        subscribedTo[speechRec] = "speechID"
        speechRec.subscribe("speechID")
        global speechMem
        speechMem = ALProxy("ALMemory", IP, PORT)

        #time.sleep(sec)
        speechMem.subscribeToEvent("WordRecognized",
            "robot",
            "onWordRecognized")

        print "Subscribed"
        while sec > 0:
            if (self.loop_lock):
                self.loop_lock = False
                break

            timeToSleep = min(sec, 0.5)
            sec -= 0.5
            time.sleep(timeToSleep)

        speechMem.unsubscribeToEvent("WordRecognized",
            "robot")
        val = speechMem.getData("WordRecognized")
        speechRec.pause(True)

        # remove from list of subscribed
        speechRec.unsubscribe("speechID")
        del subscribedTo[speechRec]

        if val == []:
            return None
        return val[0]

    def onWordRecognized(self, *_args):
        print "Nao has recognized a word!"
        self.loop_lock = True

    def faceDetection(self):
        faceDet = ALProxy("ALFaceDetection", IP, PORT)
        period = 500

        # update the subscribed list
        subscribedTo[faceDet] = "faceID"
        faceDet.subscribe("faceID", period, 0.0)

        faceMem = ALProxy("ALMemory", IP, PORT)
        val = faceMem.getData("FaceDetected")
        print val

        # remove from subscribed list
        faceDet.unsubscribe("faceID")
        del subscribedTo[faceDet]

        if (val and isinstance(val, list) and len(val) == 2):
            return True
        return False

    def faceTracker(self, faceSize):
        tracker = ALProxy("ALTracker", IP, PORT)
        targetName = "Face"
        tracker.registerTarget(targetName, faceSize)
        tracker.track(targetName)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user"
        tracker.stopTracker()
        tracker.unregisterAllTargets()

    def greet(self):
        #greet = ALProxy("ALAnimatedSpeech", IP, PORT)
        greet = ALProxy("ALTextToSpeech", IP, PORT)
        #motionProxy = ALProxy("ALMotion", IP, PORT)
        #postureProxy = ALProxy("ALRobotPosture", IP, PORT)
        #motionProxy.wakeUp()
        #postureProxy.goToPosture("Stand", 0.5)
        #greet.say("Hello. ^start(animations/Stand/Gestures/Hey_1) My name is Nao. Nice to meet you! ^wait(animations/Stand/Gestures/Hey_1)")
        greet.say("Hello. I'm here to make a delivery.")

    def farewell(self):
        farewell = ALProxy("ALTextToSpeech", IP, PORT)
        #motionProxy = ALProxy("ALMotion", IP, PORT)
        #exr = ALProxy("ALMotion", IP, PORT)
        #exr.setStiffnesses("RArm", 1.0)
        #names = ["RShoulderPitch", "RWristYaw", "RShoulderRoll", "RElbowRoll", "RElbowYaw"]
        #angles = [-1.57, 1.3, -0.79, 0.5, 2.0]
        #fractionMaxSpeed = 0.2
        #exr.post.setAngles(names, angles, fractionMaxSpeed)
        farewell.say("Goodbye.")
        #motionProxy.rest()

    def openLefthand(self):
        openl = ALProxy("ALMotion", IP, PORT)
        openl.openHand('LHand')
        time.sleep(3)

    def openRighthand(self):
        openr = ALProxy("ALMotion", IP, PORT)
        openr.openHand('RHand')
        time.sleep(3)

    def closeLefthand(self):
        closel = ALProxy("ALMotion", IP, PORT)
        closel.closeHand('LHand')

    def closeRighthand(self):
        closer = ALProxy("ALMotion", IP, PORT)
        closer.closeHand('RHand')

    def leftarmExtended(self):
        exl = ALProxy("ALMotion", IP, PORT)
        exl.setStiffnesses("LArm", 1.0)
        names = ["LShoulderPitch", "LWristYaw", "LShoulderRoll", "LElbowRoll", "LElbowYaw"]
        angles = [0.3, -1.3, 0, -0.5, -2.0]
        fractionMaxSpeed = 0.2
        exl.setAngles(names, angles, fractionMaxSpeed)

    def rightarmExtended(self):
        exr = ALProxy("ALMotion", IP, PORT)
        exr.setStiffnesses("RArm", 1.0)
        names = ["RShoulderPitch", "RWristYaw", "RShoulderRoll", "RElbowRoll", "RElbowYaw"]
        angles = [0.3, 1.3, 0, 0.5, 2.0]
        fractionMaxSpeed = 0.2
        exr.setAngles(names, angles, fractionMaxSpeed)

    def leftarmRetracted(self):
        rel = ALProxy("ALMotion", IP, PORT)
        names = ["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw", "LHand"]
        angles = [1.4495880603790283, 0.10426998138427734, -0.9924559593200684, -0.8345379829406738,
                  -1.5938677787780762, 0.6855999827384949]
        rel.setAngles(names, angles, 0.2)


    def rightarmRetracted(self):
        rer = ALProxy("ALMotion", IP, PORT)
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw", "RHand"]
        angles = [1.4495880603790283, -0.10426998138427734, 0.9924559593200684, 0.8345379829406738, 1.5938677787780762,
                  -0.6855999827384949]
        rer.setAngles(names, angles, 0.2)

    def touchDetected(self):
        touch = ALProxy("ALTouch", IP, PORT)
        status = touch.getStatus()
        return status

    def waitUntilTouchDetected(self):
        status = False
        while (status == False):
            rawStatus = self.touchDetected()
            print rawStatus
            for array in rawStatus:
                if array[0] == 'RHand' or array[0] == 'RHand/Touch/Left' or array[0] == 'RHand/Touch/Right' or array[0] == 'RHand/Touch/Back':
                    if array[1] == True:
                        status = array[1]
                        break
            time.sleep(.1)
        return status

    def comment(self, speech):
        comment = ALProxy("ALTextToSpeech", IP, PORT)
        comment.say(speech)

    def question(self):
        question = ALProxy("ALTextToSpeech", IP, PORT)
        question.say(QuestionList[0], configuration)

    def answer(self, mark):
        answer = ALProxy("ALTextToSpeech", IP, PORT)
        answer.say(random.choice(handoff_question[mark]))

    def instruct(self, instruction):
        instruct = ALProxy("ALTextToSpeech", IP, PORT)
        instruct.say(instruction)

    def repeat(self):
        repeat = ALProxy("ALTextToSpeech", IP, PORT)
        repeat.say("Sorry I did not get it. Can you repeat the question?")

    def soundDetected(self, sec):
        sound = ALProxy("ALSoundDetection", IP, PORT)
        sound.setParameter("Sensitivity", 0.3)

        # update the list of subscribed
        subscribedTo[sound] = "soundID"
        sound.subscribe("soundID")

        time.sleep(sec)
        soundMem = ALProxy("ALMemory", IP, PORT)
        val = soundMem.getData("SoundDetected")

        # remove sound from subscribed
        sound.unsubscribe("soundID")
        del subscribedTo[sound]

        return val

    def movementDetected(self, sec):
        move = ALProxy("ALMovementDetection", IP, PORT)

        # updated the subscribed list
        subscribedTo[move] = "moveID"
        move.subscribe("moveID")

        time.sleep(sec)
        moves = ALProxy("ALMemory", IP, PORT)
        val = moves.getData("MovementDetection/MovementInfo")

        # remove move from subscribe list
        move.unsubscribe("moveID")
        del subscribedTo[move]

        return val
'''
    def gaze_at_human(self):
        head_at_human = ALProxy("ALMotion", IP, PORT)
        names = ["HeadPitch", "HeadYaw"]
        angles = [0, 0]
        head_at_human.setAngles(names, angles, 0.2)

    def gaze_intimacy(self):
        head_intimacy = ALProxy("ALMotion", IP, PORT)
        angle_list = [0.1396, -0.1396]
        time_between = np.random.normal(4.75, 1.39)
        time.sleep(time_between)
        head_intimacy.setAngles("HeadYaw", random.choice(angle_list), 0.2)
        time_length = np.random.normal(1.96, 0.32)
        time.sleep(time_length)
        self.gaze_at_human()

    def gaze_cognition(self):
        head_cognition = ALProxy("ALMotion", IP, PORT)
        head_cognition.setAngles("HeadPitch", -0.1396)
        time_length = np.random.normal(3.54, 1.26)
        time.sleep(time_length)

    def gaze_conf_hand(self, side):
        head_hand = ALProxy("ALMotion", IP, PORT)
        names = ["HeadPitch", "HeadYaw"]
        if side == "left":
            angles = [0.3359041213989258, 0.3819241523742676]
        else:
            angles = [0.37885594367980957, -0.6075060367584229]
        head_hand.setAngles(names, angles, 0.2)
'''
##########################
# MICROINTERACTIONS
##########################
class Greeter:
    def __init__(self, robot, groupid, microid, speech_token):
        self.groupid = groupid
        self.microid = microid
        self.robot = robot
        self.speech_token = speech_token

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        startState_Ignored = "Start_Silent_Ignored"
        startState_Present = "Start_Silent_Present"
        newState = None
        wait_time = 5
        while True:
            if last_final_state is None or last_final_state == "human_ready":
                self.robot.gaze.addBehavior("Greeter", "GAZE_AT", None)
                print startState_Present + "\n"
                #self.robot.gaze_at_human()
                newState = "Speaking_Present_Greeting"
                print newState + "\n"
                self.robot.gaze.killBehavior("Greeter", "GAZE_AT")

            elif last_final_state == "human_ignore":
                self.robot.gaze.addBehavior("Greeter", "GAZE_AT", None)
                print startState_Ignored + "\n"
                #self.robot.gaze_at_human()
                newState = "Speaking_Ignored_Greeting"
                print newState + "\n"
                self.robot.gaze.killBehavior("Greeter", "GAZE_AT")

            if newState == "Speaking_Present_Greeting":
                self.robot.gaze.addBehavior("Greeter", "GAZE_AT", None)
                self.speech_token.acquire()
                self.robot.greet()
                self.speech_token.release()
                newState = "Waiting_Silent_Present"
                print newState + "\n"
                self.robot.gaze.killBehavior("Greeter", "GAZE_AT")

            elif newState == "Speaking_Ignored_Greeting":
                self.robot.gaze.addBehavior("Greeter", "GAZE_AT", None)
                self.speech_token.acquire()
                self.robot.greet()
                self.speech_token.release()
                newState = "Waiting_Silent_Ignored"
                print newState + "\n"
                self.robot.gaze.killBehavior("Greeter", "GAZE_AT")

            if newState == "Waiting_Silent_Present" or "Waiting_Silent_Ignored":
                self.robot.gaze.addBehavior("Greeter", "GAZE_AT", None)
                speech = self.robot.speechRecognition(wait_time)
                print speech
                if speech != "":
                    newState = "Silent_ReturnedGreeting_End"
                    print newState + "\n"
                    output = "human_ready"
                    self.robot.gaze.killBehavior("Greeter", "GAZE_AT")
                    break
                else:
                    newState = "Silent_Ignored_End"
                    print newState + "\n"
                    output = "human_ignore"
                    self.robot.gaze.killBehavior("Greeter", "GAZE_AT")
                    break

        out_list.append(output)

class Farewell:
    def __init__(self, robot, groupid, microid, speech_token):
        self.groupid = groupid
        self.microid = microid
        self.robot = robot
        self.speech_token = speech_token

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        startState_Ignored = "Start_Silent_Ignored"
        startState_Present = "Start_Silent_Present"
        newState = None
        wait_time = 5
        while True:
            if last_final_state is None or last_final_state == "human_ready":
                self.robot.gaze.addBehavior("Farewell", "GAZE_AT", None)
                print startState_Present + "\n"
                #self.robot.gaze_at_human()
                newState = "Speaking_Present_BiddingFarewell"
                print newState + "\n"
                self.robot.gaze.killBehavior("Farewell", "GAZE_AT")

            elif last_final_state == "human_ignore":
                self.robot.gaze.addBehavior("Farewell", "GAZE_AT", None)
                print startState_Ignored + "\n"
                #self.robot.gaze_at_human()
                newState = "Speaking_Ignored_BiddingFarewell"
                print newState + "\n"
                self.robot.gaze.killBehavior("Farewell", "GAZE_AT")

            if newState == "Speaking_Present_BiddingFarewell":
                self.robot.gaze.addBehavior("Farewell", "GAZE_AT", None)
                self.speech_token.acquire()
                self.robot.farewell()
                self.speech_token.release()
                newState = "Wait_Silent_Present"
                print newState + "\n"
                self.robot.gaze.killBehavior("Farewell", "GAZE_AT")

            elif newState == "Speaking_Ignored_BiddingFarewell":
                self.robot.gaze.addBehavior("Farewell", "GAZE_AT", None)
                self.speech_token.acquire()
                self.robot.farewell()
                self.speech_token.release()
                newState = "Wait_Silent_Ignored"
                print newState + "\n"
                self.robot.gaze.killBehavior("Farewell", "GAZE_AT")

            if newState == "Wait_Silent_Present" or "Wait_Silent_Ignored":
                self.robot.gaze.addBehavior("Farewell", "GAZE_AT", None)
                speech = self.robot.speechRecognition(wait_time)
                if speech == "see you" or speech == "bye":
                    newState = "Silent_End_Acknowledged"
                    print newState + "\n"
                    output = "human_ready"
                    self.robot.gaze.killBehavior("Farewell", "GAZE_AT")
                    break
                else:
                    newState = "Silent_Ignored_End"
                    print newState + "\n"
                    output = "human_ignore"
                    self.robot.gaze.killBehavior("Farewell", "GAZE_AT")
                    break
        out_list.append(output)

class Comment:
    def __init__(self, robot, groupid, microid, speechList, speech_token):
        self.robot = robot
        self.groupid = groupid
        self.microid = microid
        self.speechList = speechList
        self.speech_token = speech_token

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        startState_Ignored = "Start_Silent_Ignored"
        startState_Present = "Start_Silent_Present"
        startState_Busy = "Start_Silent_Busy"
        while True:
            if last_final_state == "human_ready":

                self.robot.gaze.addBehavior("Comment", "GAZE_AT", None)
                print startState_Present + "\n"
                newState = "Speaking_Present_Asking"
                print newState + "\n"
                self.robot.gaze.killBehavior("Comment", "GAZE_AT")

                if newState == "Speaking_Present_Asking":

                    self.robot.gaze.addBehavior("Comment", "GAZE_INTIMACY", None)
                    self.speech_token.acquire()
                    #t1 = threading.Thread(target=robot.comment, args=(self.speechList[0], ))
                    #t2 = threading.Thread(target=robot.gaze_intimacy, args=())
                    #t1.start()
                    #t2.start()
                    #t1.join()
                    #flag = True
                    #while(flag):
                    #   if t1.isAlive() != True:
                    #        flag = False
                    #        self.robot.gaze_at_human()
                    self.robot.comment(speechList[0])
                    self.speech_token.release()
                    self.robot.gaze.killBehavior("Comment", "GAZE_INTIMACY")

                    self.robot.gaze.addBehavior("Comment", "GAZE_AT", None)
                    newState = "Silent_Present_End"
                    self.robot.gaze.killBehavior("Comment", "GAZE_AT")
                    print newState + "\n"
                    output = "human_ready"
                    break

            elif last_final_state == "human_busy":

                self.robot.gaze.addBehavior("Comment", "GAZE_AT", None)
                print startState_Busy + "\n"
                newState = "Speaking_Busy_Asking"
                print newState + "\n"
                self.robot.gaze.killBehavior("Comment", "GAZE_AT")

                if newState == "Speaking_Busy_Asking":

                    self.robot.gaze.addBehavior("Comment", "GAZE_INTIMACY", None)
                    self.speech_token.acquire()
                    #t1 = threading.Thread(target=robot.comment, args=(self.speechList[0], ))
                    #t2 = threading.Thread(target=robot.gaze_intimacy, args=())
                    #t1.start()
                    #t2.start()
                    #t1.join()
                    #self.robot.comment(speechList[0])
                    #flag = True
                    #while(flag):
                    #    if t1.isAlive() != True:
                    #        flag = False
                    #        self.robot.gaze_at_human()
                    self.robot.comment(speechList[0])
                    self.speech_token.release()
                    self.robot.gaze.killBehavior("Comment", "GAZE_INTIMACY")

                    self.robot.gaze.addBehavior("Comment", "GAZE_AT", None)
                    newState = "Silent_End_Busy"
                    print newState + "\n"
                    output = "human_busy"
                    self.robot.gaze.killBehavior("Comment", "GAZE_AT")
                    break

            elif last_final_state is None or last_final_state == "human_ignore":

                self.robot.gaze.addBehavior("Comment", "GAZE_AT", None)
                print startState_Ignored + "\n"
                newState = "Speaking_Ignored_Asking"
                print newState + "\n"
                self.robot.gaze.killBehavior("Comment", "GAZE_AT")

                if newState == "Speaking_Ignored_Asking":

                    self.robot.gaze.addBehavior("Comment", "GAZE_INTIMACY", None)
                    self.speech_token.acquire()
                    #t1 = threading.Thread(target=robot.comment, args=(self.speechList[0], ))
                    #t2 = threading.Thread(target=robot.gaze_intimacy, args=())
                    #t1.start()
                    #t2.start()
                    #t1.join()
                    #flag = True
                    #while(flag):
                    #    if t1.isAlive() != True:
                    #        flag = False
                    #        self.robot.gaze_at_human()
                    self.robot.comment(speechList[0])
                    self.speech_token.release()
                    self.robot.gaze.killBehavior("Comment", "GAZE_INTIMACY")

                    self.robot.gaze.addBehavior("Comment", "GAZE_AT", None)
                    newState = "Silent_Ignored_End"
                    print newState + "\n"
                    output = "human_ignore"
                    self.robot.gaze.killBehavior("Comment", "GAZE_AT")
                    break
        out_list.append(output)

class Handoff:
    def __init__(self, robot, groupid, microid, larm_token, rarm_token, lhand_token, rhand_token, side, give_receive):
        self.robot = robot
        self.groupid = groupid
        self.microid = microid
        self.larm_token = larm_token
        self.rarm_token = rarm_token
        self.lhand_token = lhand_token
        self.rhand_token = rhand_token
        self.side = side
        self.give_receive = give_receive

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        wait_time = 15
        StartState = "Start_Present_Arm_Retracted"
        if self.side == None:
            self.side = "right"
        while True:
            if StartState == "Start_Present_Arm_Retracted":
                self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                print StartState + "\n"
                '''
                if self.side == "left":
                    self.larm_token.acquire()
                    self.robot.leftarmRetracted()
                    self.larm_token.release()
                    self.lhand_token.acquire()
                    self.robot.closeLefthand()
                    self.lhand_token.release()
                elif self.side == "right":
                    self.rarm_token.acquire()
                    self.robot.rightarmRetracted()
                    self.rarm_token.release()
                    self.rhand_token.acquire()
                    self.robot.closeRighthand()
                    self.rhand_token.release()
                '''
                newState = "Present_Extending_Arm_Extended"
                self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                print newState + "\n"

            if newState == "Present_Extending_Arm_Extended":
                if self.side == "left":
                    self.robot.gaze.addBehavior("Handoff", "GAZE_REFERENTIAL", "left")
                    self.larm_token.acquire()
                    self.robot.leftarmExtended()
                    #t1 = threading.Thread(target=robot.leftarmExtended, args=())
                    #t2 = threading.Thread(target=robot.gaze_conf_hand, args=(self.side))
                    #t1.start()
                    #t2.start()
                    #t1.join()
                    #t2.join()
                    self.larm_token.release()
                    self.robot.gaze.killBehavior("Handoff", "GAZE_REFERENTIAL")
                elif self.side == "right":
                    self.robot.gaze.addBehavior("Handoff", "GAZE_REFERENTIAL", "right")
                    self.rarm_token.acquire()
                    self.robot.rightarmExtended()
                    #t1 = threading.Thread(target=robot.rightarmExtended, args=())
                    #t2 = threading.Thread(target=robot.gaze_conf_hand, args=(self.side))
                    #t1.start()
                    #t2.start()
                    #t1.join()
                    #t2.join()
                    self.rarm_token.release()
                    self.robot.gaze.killBehavior("Handoff", "GAZE_REFERENTIAL")
                newState = "Releasing_Contacted_Arm_Extended"
                print newState + "\n"

            if self.give_receive == "receive":
                if newState == "Releasing_Contacted_Arm_Extended":
                    if self.side == "left":
                        self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                        self.lhand_token.acquire()
                        self.robot.openLefthand()
                        self.lhand_token.release()
                        self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                    elif self.side == "right":
                        self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                        self.rhand_token.acquire()
                        self.robot.openRighthand()
                        self.rhand_token.release()
                        self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                    newState = "Releasing_Present_Arm_Extended"
                    print newState + "\n"

                if newState == "Releasing_Present_Arm_Extended":
                    #touch = self.robot.touchDetected()
                    #time.sleep(wait_time)
                    touch = self.robot.waitUntilTouchDetected()
                    print touch
                    if self.side == "left":
                        if touch[4][1] == True:
                            self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                            self.lhand_token.acquire()
                            self.robot.closeLefthand()
                            self.lhand_token.release()
                            time.sleep(1)
                            self.larm_token.acquire()
                            self.robot.leftarmRetracted()
                            self.larm_token.release()
                            self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                            newState = "Present_End_Arm_Retracted"
                            print newState + "\n"
                            output = "human_ready"
                            time.sleep(2)
                            break
                    elif self.side == "right":
                        if touch[1][1] == True:
                            self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                            self.rhand_token.acquire()
                            self.robot.closeRighthand()
                            self.rhand_token.release()
                            time.sleep(1)
                            self.rarm_token.acquire()
                            self.robot.rightarmRetracted()
                            self.rarm_token.release()
                            self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                            newState = "Present_End_Arm_Retracted"
                            print newState + "\n"
                            output = "human_ready"
                            time.sleep(2)
                            break
            if self.give_receive == "give":
                if newState == "Releasing_Contacted_Arm_Extended":
                    #touch = self.robot.touchDetected()
                    #time.sleep(wait_time)
                    touch = self.robot.waitUntilTouchDetected()
                    print touch
                    if self.side == "left":
                        self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                        self.lhand_token.acquire()
                        self.robot.openLefthand()
                        self.lhand_token.release()
                        self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                    elif self.side == "right":
                        self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                        self.rhand_token.acquire()
                        self.robot.openRighthand()
                        self.rhand_token.release()
                        self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                    newState = "Releasing_Present_Arm_Extended"
                    print newState + "\n"

                if newState == "Releasing_Present_Arm_Extended":
                    time.sleep(5)
                    if self.side == "left":
                        self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                        self.lhand_token.acquire()
                        self.robot.closeLefthand()
                        self.lhand_token.release()
                        time.sleep(1)
                        self.larm_token.acquire()
                        self.robot.leftarmRetracted()
                        self.larm_token.release()
                        self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                    elif self.side == "right":
                        self.robot.gaze.addBehavior("Handoff", "GAZE_AT", None)
                        self.rhand_token.acquire()
                        self.robot.closeRighthand()
                        self.rhand_token.release()
                        time.sleep(1)
                        self.rarm_token.acquire()
                        self.robot.rightarmRetracted()
                        self.rarm_token.release()
                        self.robot.gaze.killBehavior("Handoff", "GAZE_AT")
                    newState = "Present_End_Arm_Retracted"
                    print newState + "\n"
                    output = "human_ready"
                    break
        out_list.append(output)

class Question:
    def __init__(self, robot, groupid, microid, speech_token):
        self.groupid = groupid
        self.microid = microid
        self.robot = robot
        self.speech_token = speech_token

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        StartState = "Start_Silent_Present_H_Silent"
        wait_time = 5

        while True:
            if StartState == "Start_Silent_Present_H_Silent":
                print StartState + "\n"
                newState = "Speaking_Present_H_Silent_Asking"
                print newState + "\n"

            if newState == "Speaking_Present_H_Silent_Asking":
                self.speech_token.acquire()
                self.robot.question()
                self.speech_token.release()
                newState = "Silent_Present_Listening_H_Silent"

            while Ture:
                if newState == "Silent_Present_Listening_H_Silent":
                    print newState + "\n"
                    val = self.robot.speechRecognition(wait_time)
                    if val is None:
                        newState = "Silent_Ignored_H_Silent_End"
                        print newState + "\n"
                        break
                    else:
                        newState = "Silent_Listening_H_Speaking_Answering"
                        print newState + "\n"

                    if newState == "Silent_Listening_H_Speaking_Answering":
                        if val in AnswerList:
                            newState = "Silent_Present_Listening_H_Silent"
                            print newState + "\n"
                            newState = "Silent_Present_H_Silent_End"
                            print newState + "\n"
                            break
                        elif val not in AnswerList:
                            newState = "Silent_Present_Listening_H_Silent"
                            print newState + "\n"
                            newState = "Speaking_RepeatAsk_Present_H_Silent"
                            print newState + "\n"
                            self.robot.repeat()
                            newState = "Silent_Present_Listening_H_Silent"

            if newState == "Silent_Ignored_H_Silent_End":
                output = "human_ignore"
                break

            if newState == "Silent_Present_H_Silent_End":
                idx = AnswerList.index(val)
                link = linkList[idx]
                if (link == "human_ready" or link == ""):
                    output = "human_ready"
                else:
                    output = "human_ignore"
                break

            '''
                if val not in AnswerList:
                    if (val == "Can you  repeat?"):
                        newState = "Silent_Present_Listening_H_Silent"
                    else:
                        newState = "Silent_Ignored_H_Silent_End"
                        output = "human_ignore"
                        break
                    print newState + "\n"
                elif val in AnswerList:
                    # get the index of the answer
                    idx = AnswerList.index(val)
                    link = linkList[idx]

                    if (link == "human_ready" or link == ""):
                        newState = "Silent_Listening_H_Speaking_Answering"
                    else:
                        newState = "Silent_Ignored_H_Silent_End"
                        output = "human_ignore"
                        break
                    print newState + "\n"

            if newState == "Silent_Listening_H_Speaking_Answering":
                newState = "Silent_Present_Listening_H_Silent"
                print newState + "\n"

            if newState == "Silent_Present_Listening_H_Silent":
                #if self.robot.speechRecognition(wait_time) == "Can you repeat?":
                if val == "Can you repeat?":
                    newState = "Silent_Present_Listening_H_Silent"
                    print newState + "\n"
                else:
                    newState = "Silent_Present_H_Silent_End"
                    print newState + "\n"
                    output = "human_ready"
                    break
            '''
        out_list.append(output)

class Answer:
    def __int__(self, robot, groupid, microid, speech_token):
        self.groupid = groupid
        self.microid = microid
        self.robot = robot
        self.speech_token = speech_token

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state):
        startState_Present = "Start_Silent_Present_H_Silent"
        startState_Ignored = "Start_Silent_Ignored_H_Silent"
        startState_Busy = "Start_Silent_Busy_H_Silent"
        wait_time = 5
        newState = None
        mark = None
        if last_final_state == None or last_final_state == "human_ready":
            print startState_Present + "\n"
            val = self.robot.speechRecognition(wait_time)
            key = False
            for i in range(len(handoff_question)):
                for j in range(len(handoff_question[i])):
                    if val == handoff_question[i][j]:
                        key = True
                        mark = i
                        newState = "Start_Silent_H_Speaking_Asking"
                        print newState + "\n"
            if key == False:
                newState = "Silent_Present_H_Silent_End"
                print newState + "\n"
                output = "human_ready"

        if last_final_state == "human_ignored":
            print startState_Ignored + "\n"
            val = self.robot.speechRecognition(wait_time)
            key = False
            for i in range(len(handoff_question)):
                for j in range(len(handoff_question[i])):
                    if val == handoff_question[i][j]:
                        key = True
                        mark = i
                        newState = "Start_Silent_H_Speaking_Asking"
                        print newState + "\n"
            if key == False:
                newState = "Silent_Present_H_Silent_End"
                print newState + "\n"
                output = "human_ignored"

        if last_final_state == "human_busy":
            print startState_Busy + "\n"
            val = self.robot.speechRecognition(wait_time)
            key = False
            for i in range(len(handoff_question)):
                for j in range(len(handoff_question[i])):
                    if val == handoff_question[i][j]:
                        key = True
                        mark = i
                        newState = "Start_Silent_H_Speaking_Asking"
                        print newState + "\n"
            if key == False:
                newState = "Silent_Present_H_Silent_End"
                print newState + "\n"
                output = "human_busy"

        if newState == "Start_Silent_H_Speaking_Asking":
            newState = "Start_Silent_Present_H_Silent"
            print newState + "\n"

        if newState == "Start_Silent_Present_H_Silent":
            newState = "Speaking_Present_H_Silent_Answering"
            print newState + "\n"

        if newState == "Speaking_Present_H_Silent_Answering":
            self.speech_token.acquire()
            self.robot.answer(mark)
            self.speech_token.release()
            newState = "Start_Silent_Present_H_Silent"
            print newState + "\n"

        if newState == "Start_Silent_Present_H_Silent":
            print 1

class Instruct:
    def __init__(self, robot, groupid, microid, speech_token, instruction):
        self.groupid = groupid
        self.microid = microid
        self.robot = robot
        self.speech_token = speech_token
        self.instruction = instruction

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        StartState = "Start_Silent_Listening_H_Silent"
        wait_time = 10
        while True:
            if StartState == "Start_Silent_Listening_H_Silent":
                self.robot.gaze.addBehavior("Instruct", "GAZE_AT", None)
                print StartState + "\n"
                newState = "Speaking_Listening_Instructing_H_Silent"
                print newState + "\n"
                self.robot.gaze.killBehavior("Instruct", "GAZE_AT")

            if newState == "Speaking_Listening_Instructing_H_Silent":
                self.robot.gaze.addBehavior("Instruct", "GAZE_REFERENTIAL", self.instruction)
                self.robot.gesture.addBehavior("Instruct", "GESTURE_DIECTIC", self.instruction)
                self.speech_token.acquire()
                self.robot.instruct(self.instruction)
                self.speech_token.release()
                newState = "Silent_Listening_Instructing_H_Silent"
                print newState + "\n"

            if newState == "Silent_Listening_Instructing_H_Silent":
                newState = "Waiting_Silent_H_Silent_Acting"
                print newState + "\n"
                self.robot.gesture.killBehavior("Instruct", "GESTURE_DIECTIC")
                self.robot.gaze.killBehavior("Instruct", "GAZE_REFERENTIAL")

            if newState == "Waiting_Silent_H_Silent_Acting":
                self.robot.gesture.addBehavior("Instruct", "GESTURE_NONE")
                val = self.robot.speechRecognition(wait_time)
                print val
                if val == "I have a question":
                    newState = "Waiting_Silent_H_Speaking_Breakdown_request"
                    print newState + "\n"
                elif val == "I am finished":
                    newState = "Waiting_Silent_H_Silent_Finish"
                    print newState + "\n"
                else:
                    newState = "Silent_H_Silent_End_Acting"
                    print newState + "\n"
                    output = "human_busy"
                    self.robot.gesture.killBehavior("Instruct", "GESTURE_NONE")
                    break

                self.robot.gesture.killBehavior("Instruct", "GESTURE_NONE")

            if newState == "Waiting_Silent_H_Speaking_Breakdown_request":
                newState = "Waiting_Silent_H_Silent_Breakdown_request"
                print newState + "\n"

            if newState == "Waiting_Silent_H_Silent_Breakdown_request":
                self.robot.gaze.addBehavior("Instruct", "GAZE_AT", None)
                newState = "Silent_H_Silent_End_Breakdown_request"
                print newState + "\n"
                self.robot.gaze.killBehavior("Instruct", "GAZE_AT")
                output = "human_ignore"
                break

            if newState == "Waiting_Silent_H_Silent_Finish":
                self.robot.gaze.addBehavior("Instruct", "GAZE_AT", None)
                newState = "Silent_H_Silent_Finish_End"
                print newState + "\n"
                self.robot.gaze.killBehavior("Instruct", "GAZE_AT")
                output = "human_ready"
                break

        out_list.append(output)

class Wait:
    def __init__(self, robot, groupid, microid, speechRecog, waitTime):
        self.robot = robot
        self.groupid = groupid
        self.microid = microid
        self.speechRecog = speechRecog
        self.waitTime = waitTime

        if self.speechRecog == None:
            self.speechRecog = False

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        startState_Present = "Start_Silent_Present"
        startState_Busy = "Start_Silent_Busy"
        startState_Ignored= "Start_Silent_Ignored"

        newState = None

        wait_time = self.waitTime

        while True:
            if last_final_state == "human_ready":
                print startState_Present + "\n"
                if self.speechRecog and robot.speechRecognition(wait_time) == "I am ready":
                    newState = "Start_Speech_Notify"
                else:
                    time.sleep(wait_time)
                    newState = "Silent_Present_End"
                    output = "human_ready"
                print newState + "\n"
                break

            elif last_final_state == "human_busy":
                print startState_Busy + "\n"
                if self.speechRecog and robot.speechRecognition(wait_time) == "I am ready":
                    newState = "Start_Speech_Notify"
                else:
                    time.sleep(wait_time)
                    newState = "Silent_Busy_End"
                    output = "human_busy"
                print newState + "\n"
                break

            elif last_final_state == None or last_final_state == "human_ignore":
                print startState_Ignored + "\n"
                if self.speechRecog and robot.speechRecognition(wait_time) == "I am ready":
                    newState = "Start_Speech_Notify"
                else:
                    time.sleep(wait_time)
                    newState = "Silent_Ignored_End"
                    output = "human_ignore"
                print newState + "\n"
                break

            if newState == "Start_Speech_Notify":
                print newState + "\n"
                output = "human_present"
                break

        out_list.append(output)

##########################
# GROUP OF CONCURRENT MICROINTERACTIONS
##########################
class Group:
    def __init__(self, name, target):
        self.name = name
        self.target = target

    def getName(self):
        return self.name

    def getTarget(self):
        return self.target

##########################
# RUN
##########################
if __name__ == "__main__":
    myBroker = ALBroker("myBroker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       IP,         # parent broker IP
       PORT)       # parent broker port

    global robot
    robot = Nao("robot")
    pool = []
    speech_token = threading.Lock()
    larm_token = threading.Lock()
    rarm_token = threading.Lock()
    lhand_token = threading.Lock()
    rhand_token = threading.Lock()

    #questionlist parser
    question = []
    answer = []
    handoff_question = []
    handoff_answer = []
    question_tree = ET.parse('handoff_questions.xml')
    quesiton_root = question_tree.getroot()
    for pair in quesiton_root.iterfind('pair'):
        question = []
        answer = []
        for q in pair.iterfind('question'):
            wordList.append(q.text)
            question.append(q.text)
        for a in pair.iterfind('answer'):
            answer.append(a.text)
        handoff_question.append(question)
        handoff_answer.append(answer)

    #xml parser
    interaction = []
    transition = []
    tree = ET.parse('interaction.xml')
    root = tree.getroot()
    protocols = {}

    #get group number
    group_num = 0
    for elem in root.iterfind('group'):
        group_num += 1
    print "Number of group:", group_num

    groupid = 1
    microid = 0
    init_state = 0
    for x in range(group_num):
        interaction.append([])

    #get the initial group AND the behavioral protocols for each group
    for group in root.iterfind('group'):
        if group.attrib['init'] == "true":
            init_state = int(group.attrib['id'])

        protocols[group] = []

        for protocol in group.iterfind('protocol'):
            MicroBehaviorPairs = {}
            for pair in protocol.iterfind('pair'):
                MidroBehaviorPairs[pair.attrib['micro']] = pair.attrib['beh']
            microFix = ""
            behFix = ""
            for fix in protocol.iterfind('fix'):
                microFix = fix.attrib['micro']
                behFix = fix.attrib['beh'] 

                protocols[group].append(Protocol(MicroBehaviorPairs, microFix, behFix))

    while groupid <= group_num:
        for elem in root[groupid].iterfind('micro'):
            name = elem.find('name')

            # greeter
            if name.text == "Greeter":
                greeter = Greeter(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(greeter)

            # farewell
            if name.text == "Farewell":
                farewell = Farewell(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(farewell)

            # questioning
            if name.text == "Questioning":
                for para in elem.iterfind('parameter'):
                    for word in para.iterfind('item'):
                        wordList.append(word.attrib['val'])
                        AnswerList.append(word.attrib['val'])
                        linkList.append(word.attrib['link'])
                    if para.text == "question":
                        QuestionList.append(para.attrib['val'])
                question = Question(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(question)

            # handoff
            if name.text == "Handoff":
                side = None
                give_receive = None
                for para in elem.iterfind('parameter'):
                    if para.text == "side":
                        side = para.attrib['val']
                    if para.text == "give-receive":
                        give_receive = para.attrib['val']
                handoff = Handoff(robot, groupid, microid, larm_token, rarm_token, lhand_token, rhand_token, side, give_receive)
                interaction[groupid - 1].append(handoff)

            # comment
            if name.text == "Comment":
                speechList = []
                for para in elem.iterfind('parameter'):
                    if para.text == "content":
                        speechList.append(para.attrib['val'])
                comment = Comment(robot, groupid, microid, speechList, speech_token)
                interaction[groupid - 1].append(comment)

            # instruction
            if name.text == "Instruction":
                param = None
                for para in elem.iterfind('parameter'):
                    if para.text == "Instruction":
                        param = para.attrib['val']
                instruct = Instruct(robot, groupid, microid, speech_token, param)
                interaction[groupid - 1].append(instruct)

            # answering
            if name.text == "Answering":
                answer = Answer(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(answer)

            # wait
            if name.text == "Wait":
                speechRecog = None
                waitTime = 5
                for para in elem.iterfind('parameter'):
                    if para.text == "allow_speech":
                        if para.attrib['val'] == "true":
                            speechRecog == True
                        else:
                            speechRecog == False
                    if para.text == "wait time (seconds)":
                        waitTime = int(para.attrib['val'])
                wait = Wait(robot, groupid, microid, speechRecog, waitTime)
                interaction[groupid - 1].append(wait)
            microid += 1
        groupid += 1

    for i in range(group_num):
        target = {}
        for elem in root.iterfind('transition'):
            cond = []
            source = elem.find('source').attrib['ref']
            tar = elem.find('target').attrib['ref']
            for guard in elem.iterfind('guard'):
                cond.append(guard.attrib['condition'])
            if source == str(i):
                if elem.find('guard') is not None:
                    target[int(tar)] = cond
                else:
                    target[int(tar)] = 0
        group = Group(i, target)
        transition.append(group)


#execution
    currGroup = init_state
    last_final_state = None
    loop = True
    while loop == True:

        # obtain the protocols for this group
        # (for now, assume that there are no protocols)

        if transition[currGroup].getTarget() == {}:
            loop = False
        target = transition[currGroup].getTarget()
        out_list = []
        status = None
        key = False
        #else_group = None

        for x in range (len(interaction[currGroup])):
            pool.append(threading.Thread(target=interaction[currGroup][x].execute, args=(last_final_state, out_list, )))
        for x in range(len(interaction[currGroup])):
            pool[x].start()
        for x in range(len(interaction[currGroup])):
            pool[x].join()
        pool[:] = []

        if "human_busy" in out_list:
            status = "human_busy"
        elif "human_ready" in out_list:
            status = "human_ready"
        elif "human_ignore" in out_list:
            status = "human_ignore"
        print status
        last_final_state = status

        for i in range(group_num):
            if target.has_key(i):
                if status in target[i]:
                    key = True
                    currGroup = i
                    break

        if key == False:
            loop = False

    print "done"
    '''
                elif "else" in target[i]:
                    else_group = i
            if else_group is not None:
                currGroup = else_group
    '''

'''
        j = 1
        else_group = None
        sign = False
        while j + currGroup < group_num:
            if (j + currGroup) in transition[currGroup].getTarget():
                i = currGroup + j
                index = transition[currGroup].getTarget().index(i)
                if transition[currGroup].getCondition()[index] == 0 or transition[currGroup].getCondition()[
                    index] == out_list[len(out_list) - 1]:
                    last_final_state = out_list[len(out_list) - 1]
                    currGroup = i
                    sign = True
                    break
                if transition[currGroup].getCondition()[index] == "else":
                    else_group = i
            if (j + currGroup) == group_num - 1 and else_group is not None:
                currGroup = else_group
                sign = True
                break
            j += 1
        if sign == False:
             currGroup += 1
'''













