import operator
import threading
import time
from naoqi import ALProxy
from Protocol import Protocol

IP = "nao.local"
PORT = 9559


class Gesture():
    def __init__(self):
        self.Behaviors = {}  # all microinteractions and their current behaviors
        self.Protocols = []  # unchanging list of protocols for the group
        self.CurrMicrointeraction = None  # the currently running microinteraction
        self.CurrBehavior = None  # the currently running behavior

        # the active or queued threads
        # just as it is possible to have multiple different gaze behaviors competing
        #     for each other, it is possible for multiple different microinteractions
        #     to be undergoing the same gaze behavior
        self.GESTURE_NONE = {}
        self.GESTURE_DIECTIC = {}
        self.GESTURE_METAPHORIC = {}
        self.threadDicts = {"GESTURE_NONE": self.GESTURE_NONE,
                            "GESTURE_DIECTIC": self.GESTURE_DIECTIC,
                            "GESTURE_METAPHORIC": self.GESTURE_METAPHORIC}

        # self.loop_lock
        self.loop_lock = [True]

    def GestureNone(self, microinteraction):
        print "Gesture none!"
        motion = ALProxy("ALMotion", IP, PORT)
        posture = ALProxy("ALRobotPosture", IP, PORT)
        posture.goToPosture("Sit", 1.0)
        motion.rest()

    def GestureDiectic(self, microinteraction, para):
        arm = ALProxy("ALMotion", IP, PORT)
        arm.setStiffnesses("Body", 1.0)
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw", "RHand"]
        if para.strip() == "pick up a piece of bread and place it on the plate":
            t = 1
        elif para == "pick up the slices of ham and cheese, and place the ham on top of the bread, and the cheese on top of the ham":
            t = 5
        else:
            t = 5
        angles_sandwich = [0.7532360553741455, -0.4786500930786133, 0.31604599952697754, 0.7899680137634277, -0.6059720516204834, 0.7455999851226807]
        angles_plate = [0.7823820114135742, 0.3141592741012573, 0.5154659748077393, 0.6749181747436523, -0.7470998764038086, 0.806399941444397]
        arm.setAngles(names, angles_sandwich, 0.1)
        time.sleep(t)
        arm.setAngles(names, angles_plate, 0.1)
        time.sleep(t)
        print "Gesture diectic!"

    def GestureMetaphoric(self, microinteraction):
        print "Gesture metaphoric"

    def AddProtocols(self, protocols):  # sets the current priority-protocol
        for prot in protocols:
            self.Protocols.append(prot)
        self.Protocols.sort(key=operator.attrgetter('numMicros'))

    def RemoveProtocols(self):
        self.Protocols = []

    def addBehavior(self, microinteraction, behavior, para=None):
        # add the behavior to the list of currently-active behaviors
        self.Behaviors[microinteraction] = behavior
        if behavior == "GESTURE_NONE":
            self.GESTURE_NONE[microinteraction] = threading.Thread(target=self.GestureNone, args=(microinteraction,))
        if behavior == "GESTURE_DIECTIC":
            self.GESTURE_DIECTIC[microinteraction] = threading.Thread(target=self.GestureDiectic,
                                                                       args=(microinteraction, para))
        if behavior == "GESTURE_METAPHORIC":
            self.GESTURE_METAPHORIC[microinteraction] = threading.Thread(target=self.GestureMetaphoric,
                                                                        args=(microinteraction,))
        # choose a behavior to run based on the protocol that currently applies
        self.ChooseProcess()

    def killBehavior(self, microinteraction, behavior):
        # remove the behavior from the list of currently-active behaviors
        del self.Behaviors[microinteraction]
        # print the current processes
        for key, value in self.Behaviors.iteritems():
            print "~~~~"
            print key
            print value
            print "~~~~"

        # get the thread, remove it
        threads = self.threadDicts[behavior]
        thread = threads[microinteraction]
        del threads[microinteraction]

        # if the one we want to kill is the one that is currently running, kill it now and wait for it to die
        if self.CurrMicrointeraction == microinteraction:
            self.loop_lock[0] = False
            thread.join()
            self.CurrMicrointeraction = None
            self.CurrBehavior = None

        # reset the loop_lock
        self.loop_lock[0] = True

        # choose a behavior to run based on the protocol that currently applies
        print "Attempting to kill gesture..."
        self.ChooseProcess()

    def ChooseProcess(self):
        # print the current processes
        for key, value in self.Behaviors.iteritems():
            print "~~~~"
            print key
            print value
            print "~~~~"

        microinteraction = None
        behavior = None

        # is there a protocol that matches up with the current set of behaviors?
        protocol = None
        for prot in self.Protocols:
            goodProt = True
            for microinteraction, behavior in prot.iteritems():
                if behaviors[microinteraction] != behavior:
                    goodProt = False

            if goodProt:
                protocol = prot
                break

        if protocol != None:
            behavior = protocol.ChoiceBehavior
            microinteraction = protocol.ChoiceMicro
        else: # else, just pick the highest ranking behavior!
            for micro, beh in self.Behaviors.iteritems():
                if beh == "GESTURE_DIECTIC" or beh == "GESTURE_METAPHORIC":
                    microinteraction = micro
                    behavior = beh
                    break
                elif beh == "GESTURE_NONE":
                    microinteraction = micro
                    behavior = beh

        # kill the current behavior and start another
        if (self.CurrMicrointeraction != None):
            self.loop_lock[0] = False
            threads = self.threadDicts[self.CurrBehavior]
            thread = threads[self.CurrMicrointeraction]
            thread.join()
            del threads[microinteraction]
            print "gesture killed"

        # if there is a new process, start the new process
        if (behavior != None):
            self.loop_lock[0] = True
            threads = self.threadDicts[behavior]
            thread = threads[microinteraction]
            self.CurrMicrointeraction = microinteraction
            self.CurrBehavior = behavior
            thread.start()








