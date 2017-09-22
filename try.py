import xml.etree.ElementTree as ET
import threading
import time
import random

from naoqi import ALProxy

IP = "nao.local"
PORT = 9559
wordList = ["hello", "Nice to meet you", "bye", "see you", "I am finished", "I have a question", "good", "Can you repeat?", "I am ready"]
QuestionList = []
AnswerList = []
Instruction = []

class Nao(object):
    name = ""
    def __init__(self, name):
        self.name = name

    def getName(self):
        return self.name

    def speechRecognition(self, sec):
        speechRec = ALProxy("ALSpeechRecognition", IP, PORT)
        speechRec.setLanguage("English")
        speechRec.setVocabulary(wordList, False)
        speechRec.subscribe("speechID")
        time.sleep(sec)
        speechMem = ALProxy("ALMemory", IP, PORT)
        val = speechMem.getData("WordRecognized")
        speechRec.pause(True)
        speechRec.unsubscribe("speechID")
        return val[0]


    def faceDetection(self):
        faceDet = ALProxy("ALFaceDetection", IP, PORT)
        period = 500
        faceDet.subscribe("faceID", period, 0.0)
        faceMem = ALProxy("ALMemory", IP, PORT)
        val = faceMem.getData("FaceDetected")
        print val
        faceDet.unsubscribe("faceID")
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
        greet = ALProxy("ALAnimatedSpeech", IP, PORT)
        motionProxy = ALProxy("ALMotion", IP, PORT)
        postureProxy = ALProxy("ALRobotPosture", IP, PORT)
        motionProxy.wakeUp()
        postureProxy.goToPosture("Stand", 0.5)
        greet.say("Hello. ^start(animations/Stand/Gestures/Hey_1) My name is Nao. Nice to meet you! ^wait(animations/Stand/Gestures/Hey_1)")

    def farewell(self):
        farewell = ALProxy("ALTextToSpeech", IP, PORT)
        motionProxy = ALProxy("ALMotion", IP, PORT)
        exr = ALProxy("ALMotion", IP, PORT)
        exr.setStiffnesses("RArm", 1.0)
        names = ["RShoulderPitch", "RWristYaw", "RShoulderRoll", "RElbowRoll", "RElbowYaw"]
        angles = [-1.57, 1.3, -0.79, 0.5, 2.0]
        fractionMaxSpeed = 0.2
        exr.post.setAngles(names, angles, fractionMaxSpeed)
        farewell.say("See you")
        motionProxy.rest()

    def openLefthand(self):
        openl = ALProxy("ALMotion", IP, PORT)
        openl.openHand('LHand')

    def openRighthand(self):
        openr = ALProxy("ALMotion", IP, PORT)
        openr.openHand('RHand')

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

    def comment(self, speech):
        comment = ALProxy("ALAnimatedSpeech", IP, PORT)
        configuration = {"bodyLanguageMode": "contextual"}
        comment.say(speech, configuration)

    def question(self):
        question = ALProxy("ALAnimatedSpeech", IP, PORT)
        configuration = {"bodyLanguageMode": "contextual"}
        question.say(QuestionList[0], configuration)

    def answer(self, mark):
        answer = ALProxy("ALTextToSpeech", IP, PORT)
        answer.say(random.choice(handoff_question[mark]))

    def instruct(self):
        instruct = ALProxy("ALAnimatedSpeech", IP, PORT)
        configuration = {"bodyLanguageMode": "contextual"}
        instruct.say(Instruction[0], configuration)

    def soundDetected(self, sec):
        sound = ALProxy("ALSoundDetection", IP, PORT)
        sound.setParameter("Sensitivity", 0.3)
        sound.subscribe("soundID")
        time.sleep(sec)
        soundMem = ALProxy("ALMemory", IP, PORT)
        val = soundMem.getData("SoundDetected")
        sound.unsubscribe("soundID")
        return val

    def movementDetected(self, sec):
        move = ALProxy("ALMovementDetection", IP, PORT)
        move.subscribe("moveID")
        time.sleep(sec)
        moves = ALProxy("ALMemory", IP, PORT)
        val = moves.getData("MovementDetection/MovementInfo")
        move.unsubscribe("moveID")
        return val

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
                print startState_Present + "\n"
                newState = "Speaking_Present_Greeting"
                print newState + "\n"

            elif last_final_state == "human_ignore":
                print startState_Ignored + "\n"
                newState = "Speaking_Ignored_Greeting"
                print newState + "\n"

            if newState == "Speaking_Present_Greeting":
                self.speech_token.acquire()
                self.robot.greet()
                self.speech_token.release()
                newState = "Waiting_Silent_Present"
                print newState + "\n"

            elif newState == "Speaking_Ignored_Greeting":
                self.speech_token.acquire()
                self.robot.greet()
                self.speech_token.release()
                newState = "Waiting_Silent_Ignored"
                print newState + "\n"

            if newState == "Waiting_Silent_Present" or "Waiting_Silent_Ignored":
                speech = self.robot.speechRecognition(wait_time)
                print speech
                if speech == "Nice to meet you":
                    newState = "Silent_ReturnedGreeting_End"
                    print newState + "\n"
                    output = "human_ready"
                    break
                else:
                    newState = "Silent_Ignored_End"
                    print newState + "\n"
                    output = "human_ignore"
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
                print startState_Present + "\n"
                newState = "Speaking_Present_BiddingFarewell"
                print newState + "\n"

            elif last_final_state == "human_ignore":
                print startState_Ignored + "\n"
                newState = "Speaking_Ignored_BiddingFarewell"
                print newState + "\n"

            if newState == "Speaking_Present_BiddingFarewell":
                self.speech_token.acquire()
                self.robot.farewell()
                self.speech_token.release()
                newState = "Wait_Silent_Present"
                print newState + "\n"

            elif newState == "Speaking_Ignored_BiddingFarewell":
                self.speech_token.acquire()
                self.robot.farewell()
                self.speech_token.release()
                newState = "Wait_Silent_Ignored"
                print newState + "\n"

            if newState == "Wait_Silent_Present" or "Wait_Silent_Ignored":
                speech = self.robot.speechRecognition(wait_time)
                if speech == "see you" or speech == "bye":
                    newState = "Silent_End_Acknowledged"
                    print newState + "\n"
                    output = "human_ready"
                    break
                else:
                    newState = "Silent_Ignored_End"
                    print newState + "\n"
                    output = "human_ignore"
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
                print startState_Present + "\n"
                newState = "Speaking_Present_Asking"
                print newState + "\n"
                if newState == "Speaking_Present_Asking":
                    self.speech_token.acquire()
                    self.robot.comment(self.speechList[0])
                    self.speech_token.release()
                    newState = "Silent_Present_End"
                    print newState + "\n"
                    output = "human_ready"
                    break

            elif last_final_state == "human_busy":
                print startState_Busy + "\n"
                newState = "Speaking_Busy_Asking"
                print newState + "\n"
                if newState == "Speaking_Busy_Asking":
                    self.speech_token.acquire()
                    self.robot.comment(self.speechList[0])
                    self.speech_token.release()
                    newState = "Silent_End_Busy"
                    print newState + "\n"
                    output = "human_busy"
                    break

            elif last_final_state is None or last_final_state == "human_ignore":
                print startState_Ignored + "\n"
                newState = "Speaking_Ignored_Asking"
                print newState + "\n"
                if newState == "Speaking_Ignored_Asking":
                    self.speech_token.acquire()
                    self.robot.comment(self.speechList[0])
                    self.speech_token.acquire()
                    newState = "Silent_Ignored_End"
                    print newState + "\n"
                    output = "human_ignore"
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
                print StartState + "\n"
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
                newState = "Present_Extending_Arm_Extended"
                print newState + "\n"

            if newState == "Present_Extending_Arm_Extended":
                if self.side == "left":
                    self.larm_token.acquire()
                    self.robot.leftarmExtended()
                    self.larm_token.release()
                elif self.side == "right":
                    self.rarm_token.acquire()
                    self.robot.rightarmExtended()
                    self.rarm_token.release()
                newState = "Releasing_Contacted_Arm_Extended"
                print newState + "\n"

            if self.give_receive == "receive":
                if newState == "Releasing_Contacted_Arm_Extended":
                    if self.side == "left":
                        self.lhand_token.acquire()
                        self.robot.openLefthand()
                        self.lhand_token.release()
                    elif self.side == "right":
                        self.rhand_token.acquire()
                        self.robot.openRighthand()
                        self.rhand_token.release()
                    newState = "Releasing_Present_Arm_Extended"
                    print newState + "\n"

                if newState == "Releasing_Present_Arm_Extended":
                    touch = self.robot.touchDetected()
                    time.sleep(wait_time)
                    print touch
                    if self.side == "left":
                        if touch[4][1] == True:
                            self.lhand_token.acquire()
                            self.robot.closeLefthand()
                            self.lhand_token.release()
                            time.sleep(1)
                            self.larm_token.acquire()
                            self.robot.leftarmRetracted()
                            self.larm_token.release()
                            newState = "Present_End_Arm_Retracted"
                            print newState + "\n"
                            output = "human_ready"
                            break
                    elif self.side == "right":
                        if touch[1][1] == True:
                            self.rhand_token.acquire()
                            self.robot.closeRighthand()
                            self.rhand_token.release()
                            time.sleep(1)
                            self.rarm_token.acquire()
                            self.robot.rightarmRetracted()
                            self.rarm_token.release()
                            newState = "Present_End_Arm_Retracted"
                            print newState + "\n"
                            output = "human_ready"
                            break
            if self.give_receive == "give":
                if newState == "Releasing_Contacted_Arm_Extended":
                    touch = self.robot.touchDetected()
                    time.sleep(wait_time)
                    print touch
                    if self.side == "left":
                        if touch[4][1] == True:
                            self.lhand_token.acquire()
                            self.robot.openLefthand()
                            self.lhand_token.release()
                    elif self.side == "right":
                        if touch[1][1] == True:
                            self.rhand_token.acquire()
                            self.robot.openRighthand()
                            self.rhand_token.release()
                    newState = "Releasing_Present_Arm_Extended"
                    print newState + "\n"

                if newState == "Releasing_Present_Arm_Extended":
                    time.sleep(5)
                    if self.side == "left":
                        self.lhand_token.acquire()
                        self.robot.closeLefthand()
                        self.lhand_token.release()
                        time.sleep(1)
                        self.larm_token.acquire()
                        self.robot.leftarmRetracted()
                        self.larm_token.release()
                    elif self.side == "right":
                        self.rhand_token.acquire()
                        self.robot.closeRighthand()
                        self.rhand_token.release()
                        time.sleep(1)
                        self.rarm_token.acquire()
                        self.robot.rightarmRetracted()
                        self.rarm_token.release()
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
                print newState + "\n"

            if newState == "Silent_Present_Listening_H_Silent":
                val = self.robot.speechRecognition(wait_time)
                print val
                if val not in AnswerList:
                    newState = "Silent_Ignored_H_Silent_End"
                    print newState + "\n"
                    output = "human_ignore"
                    break
                elif val in AnswerList:
                    newState = "Silent_Listening_H_Speaking_Answering"
                    print newState + "\n"

            if newState == "Silent_Listening_H_Speaking_Answering":
                newState = "Silent_Present_Listening_H_Silent"
                print newState + "\n"

            if newState == "Silent_Present_Listening_H_Silent":
                if self.robot.speechRecognition(wait_time) == "Can you repeat?":
                    newState = "Silent_Present_Listening_H_Silent"
                    print newState + "\n"
                else:
                    newState = "Silent_Present_H_Silent_End"
                    print newState + "\n"
                    output = "human_ready"
                    break
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
        StartState = "Start_Silent_Listening_H_Silent"
        wait_time = 5
        while True:
            if StartState == "Start_Silent_Listening_H_Silent":
                print StartState + "\n"
                newState = "Speaking_Listening_Instructing_H_Silent"
                print newState + "\n"

            if newState == "Speaking_Listening_Instructing_H_Silent":
                self.speech_token.acquire()
                self.robot.instruct()
                self.speech_token.release()
                newState = "Silent_Listening_Instructing_H_Silent"
                print newState + "\n"

            if newState == "Silent_Listening_Instructing_H_Silent":
                newState = "Waiting_Silent_H_Silent_Acting"
                print newState + "\n"

            if newState == "Waiting_Silent_H_Silent_Acting":
                val = self.robot.speechRecognition(wait_time)
                if val == "I have a question":
                    newState = "Waiting_Silent_H_Speaking_Breakdown_request"
                    print newState + "\n"
                elif self.robot.movementDetected(wait_time) != []:
                    newState = "Silent_H_Silent_End_Acting"
                    print newState + "\n"
                    output = "human_busy"
                    break
                elif val == "I am finished":
                    newState = "Waiting_Silent_H_Silent_Finish"
                    print newState + "\n"

            if newState == "Waiting_Silent_H_Speaking_Breakdown_request":
                newState = "Waiting_Silent_H_Silent_Breakdown_request"
                print newState + "\n"

            if newState == "Waiting_Silent_H_Silent_Breakdown_request":
                newState = "Silent_H_Silent_End_Breakdown_request"
                print newState + "\n"
                output = "human_ready"
                break

            if newState == "Waiting_Silent_H_Silent_Finish":
                newState = "Silent_H_Silent_Finish_End"
                print newState + "\n"
                output = "human_ready"
                break

        out_list.append(output)

class Wait:
    def __init__(self, robot, groupid, microid):
        self.robot = robot
        self.groupid = groupid
        self.microid = microid

    def getGroup(self):
        return self.groupid

    def getID(self):
        return self.microid

    def execute(self, last_final_state, out_list):
        startState_Present = "Start_Silent_Present"
        startState_Busy = "Start_Silent_Busy"
        startState_Ignored= "Start_Silent_Ignored"

        newState = None

        wait_time = 5

        while True:
            if last_final_state == "human_ready":
                print startState_Present + "\n"
                if robot.speechRecognition(wait_time) == "I am ready":
                    newState = "Start_Speech_Notify"
                else:
                    newState = "Silent_Present_End"
                    output = "human_ready"
                print newState + "\n"
                break

            elif last_final_state == "human_busy":
                print startState_Busy + "\n"
                if robot.speechRecognition(wait_time) == "I am ready":
                    newState = "Start_Speech_Notify"
                else:
                    newState = "Silent_Busy_End"
                    output = "human_busy"
                print newState + "\n"
                break

            elif last_final_state == None or last_final_state == "human_ignored":
                print startState_Ignored + "\n"
                if robot.speechRecognition(wait_time) == "I am ready":
                    newState = "Start_Speech_Notify"
                else:
                    newState = "Silent_Ignored_End"
                    output = "human_ignored"
                print newState + "\n"
                break

            if newState == "Start_Speech_Notify":
                print newState + "\n"
                output = "human_present"
                break

        out_list.append(output)

class Group:
    def __init__(self, name, target):
        self.name = name
        self.target = target

    def getName(self):
        return self.name

    def getTarget(self):
        return self.target


if __name__ == "__main__":
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
    tree = ET.parse('good_delivery.xml')
    root = tree.getroot()

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

     #get the initial group
    for group in root.iterfind('group'):
        if group.attrib['init'] == "true":
            init_state = int(group.attrib['id'])

    while groupid <= group_num:
        for elem in root[groupid].iterfind('micro'):
            name = elem.find('name')
            if name.text == "greeter":
                greeter = Greeter(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(greeter)
            if name.text == "farewell":
                farewell = Farewell(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(farewell)
            if name.text == "QuestionAnswer":
                for para in elem.iterfind('parameter'):
                    for word in para.iterfind('item'):
                        wordList.append(word.attrib['val'])
                        AnswerList.append(word.attrib['val'])
                    if para.text == "question":
                        QuestionList.append(para.attrib['val'])
                question = Question(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(question)
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
            if name.text == "Comment":
                speechList = []
                for para in elem.iterfind('parameter'):
                    if para.text == "content":
                        speechList.append(para.attrib['val'])
                comment = Comment(robot, groupid, microid, speechList, speech_token)
                interaction[groupid - 1].append(comment)
            if name.text == "inst_action":
                for para in elem.iterfind('parameter'):
                    if para.text == "Instruction":
                        Instruction.append(para.attrib['val'])
                instruct = Instruct(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(instruct)
            if name.text == "AnswerQuestion":
                answer = Answer(robot, groupid, microid, speech_token)
                interaction[groupid - 1].append(answer)
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
        last_final_state = status

        for i in range(group_num):
            if target.has_key(i):
                if status in target[i]:
                    key = True
                    currGroup = i
                    break

        if key == False:
            loop = False
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













