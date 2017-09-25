import operator
import threading
import time
import random
from naoqi import ALProxy
import numpy as np
from Protocol import Protocol

IP = "nao.local"
PORT = 9559

class Gaze():
	def __init__(self):
		self.Behaviors = {}       # all microinteractions and their current behaviors
		self.Protocols = []       # unchanging list of protocols for the group
		self.CurrMicrointeraction = None   # the currently running microinteraction
		self.CurrBehavior = None  # the currently running behavior

		# the active or queued threads
		# just as it is possible to have multiple different gaze behaviors competing
		#     for each other, it is possible for multiple different microinteractions
		#     to be undergoing the same gaze behavior
		self.GAZE_AT = {}
		self.GAZE_REFERENTIAL = {}
		self.GAZE_COGNITIVE = {}
		self.GAZE_INTIMATE = {}
		self.GAZE_ELSE = {}
		self.threadDicts = {"GAZE_AT": self.GAZE_AT,
							"GAZE_REFERENTIAL": self.GAZE_REFERENTIAL,
							"GAZE_COGNITIVE": self.GAZE_COGNITIVE,
							"GAZE_INTIMACY": self.GAZE_INTIMATE,
							"GAZE_ELSE": self.GAZE_ELSE}

		# self.loop_lock
		self.loop_lock = [True]

	def GazeAt(self, microinteraction):
		head_at_human = ALProxy("ALMotion", IP, PORT)
		names = ["HeadPitch", "HeadYaw"]
		angles = [0, 0]
		head_at_human.setAngles(names, angles, 0.2)
		print "Gaze at!"

	def GazeIntimacy(self, microinteraction):
		head_intimacy = ALProxy("ALMotion", IP, PORT)
		angle_list = [0.1396, -0.1396]
		while self.loop_lock[0] == True:
			head_intimacy.setAngles("HeadYaw", random.choice(angle_list), 0.2)
			print "Gaze intimacy!"
			time_length = np.random.normal(1.96, 0.32)
			time.sleep(time_length)
			self.GazeAt(microinteraction)
			time_between = np.random.normal(4.75, 1.39)
			time.sleep(time_between)

	def GazeCognitive(self, microinteraction):
		# look up and then down
		# call GazeIntimacy
		print "Gaze cognitive! Intimacy should follow."
		self.GazeIntimacy(microinteraction)

	def GazeReferential(self, microinteraction):
		print "Gaze referential!"

	def GazeElse(self, microinteraction):
		while self.loop_lock[0] == True:
			print "Gaze else!"
			time.sleep(1)

	def AddProtocols(self, protocols):    # sets the current priority-protocol
		for prot in protocols:
			self.Protocols.append(prot)
		self.Protocols.sort(key=operator.attrgetter('numMicros'))

	def RemoveProtocols(self):
		self.Protocols = []

	def addBehavior(self, microinteraction, behavior):
		# add the behavior to the list of currently-active behaviors
		self.Behaviors[microinteraction] = behavior
		if behavior == "GAZE_AT":
			self.GAZE_AT[microinteraction] = threading.Thread(target=self.GazeAt, args=(microinteraction, ))
		if behavior == "GAZE_REFERENTIAL":
			self.GAZE_REFERENTIAL[microinteraction] = threading.Thread(target=self.GazeReferential, args=(microinteraction, ))
		if behavior == "GAZE_COGNITIVE":
			self.GAZE_COGNITIVE[microinteraction] = threading.Thread(target=self.GazeCognitive, args=(microinteraction, ))
		if behavior == "GAZE_INTIMACY":
			self.GAZE_INTIMACY[microinteraction] = threading.Thread(target=self.GazeIntimacy, args=(microinteraction, ))
		if behavior == "GAZE_ELSE":
			self.GAZE_ELSE[microinteraction] = threading.Thread(target=self.GazeElse, args=(microinteraction, ))

		# choose a behavior to run based on the protocol that currently applies
		self.ChooseProcess()

	def killBehavior(self, microinteraction, behavior):
		# remove the behavior from the list of currently-active behaviors
		del self.Behaviors[microinteraction]
		# print the current processes
		for key,value in self.Behaviors.iteritems():
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
		self.ChooseProcess()

	def ChooseProcess(self):
		# print the current processes
		for key,value in self.Behaviors.iteritems():
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
			for microinteraction,behavior in prot.iteritems():
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
				if beh == "GAZE_ELSE" and microinteraction == None:
					microinteraction = micro
					behavior = beh
				elif beh == "GAZE_AT" and (behavior == None or behavior == "GAZE_ELSE"):
					microinteraction = micro
					behavior = beh
				if beh == "GAZE_REFERENTIAL" or beh == "GAZE_INTIMACY" or beh == "GAZE_COGNITIVE":
					microinteraction = micro
					behavior = beh
					break

		# kill the current behavior and start another
		if (self.CurrMicrointeraction != None):
			self.loop_lock[0] = False
			threads = self.threadDicts[self.CurrBehavior]
			thread = threads[self.CurrMicrointeraction]
			thread.join()
			del threads[microinteraction]

		# if there is a new process, start the new process
		if (behavior != None):
			self.loop_lock[0] = True
			threads = self.threadDicts[behavior]
			thread = threads[microinteraction]
			self.CurrMicrointeraction = microinteraction
			self.CurrBehavior = behavior
			thread.start()

		






