#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import argparse
import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped
from naoqi_driver.naoqi_node import NaoqiNode
from std_msgs.msg import String
from naoqi_bridge_msgs.srv import (
    SetFloatResponse,
    SetFloat,
    SetStringResponse,
    SetString,
    GetFloatResponse,
    GetFloat)
from std_srvs.srv import (
    Empty,
    SetBool,
    SetBoolResponse)

class NaoqiBasicAwareness (NaoqiNode):
    def __init__(self, human_tracked_event_watcher_session):
        NaoqiNode.__init__(self, 'naoqi_awareness')
        #sad ide poziv druge klase
        self.proxy = human_tracked_event_watcher_session

        self.pname = rospy.get_param('/pepper_name')

        self.setEngagementSrv = rospy.Service(self.pname+"/basic_awareness/set/EngagementMode", SetString, self.handleSetEngagementMode)
        self.setTrackingSrv = rospy.Service(self.pname+"/basic_awareness/set/TrackingMode", SetString, self.handleSetTrackingMode)
        self.enable_awarenesSrv = rospy.Service(self.pname+"/basic_awareness/enable", Empty, self.handleEnableBasicAwareness)
        self.disable_awarenesSrv = rospy.Service(self.pname+"/basic_awareness/disable", Empty, self.handleDisableBasicAwareness)

        #self.getHumanSrv = rospy.Service(self.pname+"/getHumanDistance", GetFloat, self.handleGetHumanDistance)
        rospy.loginfo("naoqi_basic_awareness is initialized")

    def handleSetEngagementMode(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.basic_awareness.setEngagementMode(req.data) #   "Unengaged" "SemiEngaged" "FullyEngaged"
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTrackingMode(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.proxy.basic_awareness.setTrackingMode(req.data) # "Head" "BodyRotation" "WholeBody" "MoveContextually"
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleEnableBasicAwareness(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.proxy.basic_awareness.setEnabled(True) #True of False
            res.success = True
            res.message = str("Enabled")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res


    def handleDisableBasicAwareness(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.proxy.basic_awareness.setEnabled(False) #True of False
            res.success = True
            res.message = str("Disabled")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
   

def connectQi(pip, pport):
    rospy.loginfo("Connecting to qi at %s:%d", pip, pport)

    # Initialize qi framework.
    connection_url = "tcp://" + pip + ":" + str(pport)
    app = qi.Application(["HumanTrackedEventWatcher", "--qi-url=" + connection_url])
    if app is None:
         rospy.logerr("Could not connect to qi fremework")
         exit(1)
    else:
        return app


class HumanTrackedEventWatcher(object):
    """ A class to react to HumanTracked and PeopleLeft events """

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(HumanTrackedEventWatcher, self).__init__()

        try:
            app.start()
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " +
                   str(args.port) + ".\n")

            sys.exit(1)

        session = app.session
        self.subscribers_list = []
        self.is_speech_reco_started = False

        self.pname = rospy.get_param('/pepper_name')

        self.memory = session.service("ALMemory")
        self.speech_reco = session.service("ALSpeechRecognition")
        self.basic_awareness = session.service("ALBasicAwareness")
        self.face_characteristics = session.service("ALFaceCharacteristics")
        #self.motion = session.service("ALMotion")
        self.connect_callback("ALBasicAwareness/HumanTracked", self.on_human_tracked)
        self.connect_callback("ALBasicAwareness/HumanLost", self.on_people_left)

        #self.pub = rospy.Publisher('/detected_human_pose', PoseStamped, queue_size=10) mora prvo inicijalizirati ros node
        #self.goal_test = PoseStamped()
        #self.pub.publish(self.goal_test)

    def connect_callback(self, event_name, callback_func):
        """ connect a callback for a given event """
        subscriber = self.memory.subscriber(event_name)
        subscriber.signal.connect(callback_func)
        self.subscribers_list.append(subscriber)

    def on_human_tracked(self, value):
        """ callback for event HumanTracked """
        print "Got HumanTracked: detected person with ID:", str(value)
        if value >= 0:  # found a new person

            # publish human pose to topic /robot/human_pose

            self.pub = rospy.Publisher(self.pname+"/human/pose", PoseStamped, queue_size=10)
            #self.start_speech_reco()
            position_human = self.get_people_perception_data(value)
            [x, y, z] = position_human
            print "The tracked person with ID", value, "is at the position:", \
                "x=", x, "/ y=",  y, "/ z=", z
            self.goal = PoseStamped()
            self.goal.header.frame_id = "torso" # "base_footprint" PositionInWorldFrame
            self.goal.header.stamp = rospy.Time.now()
            self.goal.pose.position.x = x 
            self.goal.pose.position.y = y
            self.goal.pose.position.z = z
            self.goal.pose.orientation.z = 0.
            self.goal.pose.orientation.w = 1.0

            i = 0
            self.pub.publish(self.goal)
            time.sleep(0.5)
            while i<10:
                rate = rospy.Rate(10) # 10hz
                self.pub.publish(self.goal)
                i = i+1
            rate.sleep()


            # publish human gender to topic /robot/detected_human_gender
            
            
            self.pub_gender = rospy.Publisher(self.pname+"/human/gender", String, queue_size=10)


            self.face_characteristics.analyzeFaceCharacteristics(value) #pozovi metodu za pronalazenje spola
            time.sleep(1)

            gender_human = self.get_people_gender_data(value)
            [gender, confidence] = gender_human
            print gender
            if gender == 0:
                gender_str = "Female"
            elif gender == 1:
                gender_str = "Male"
            print gender_str

            self.goal_gender = String()
            self.goal_gender.data = "Male" 
            time.sleep(0.5)

            i = 0
            self.pub_gender.publish(self.goal_gender)
            time.sleep(0.5)
            while i<10:
                rate = rospy.Rate(10) # 10hz
                self.pub.publish(self.goal_gender)
                i = i+1
            rate.sleep()


    def on_people_left(self, value):
        """ callback for event PeopleLeft """
        print "Got PeopleLeft: lost person", str(value)
        #self.stop_speech_reco()

    def start_speech_reco(self):
        """ start ASR when someone's detected in event handler class """
        #vokabular se mora napuniti prije pokretanja klase
        if not self.is_speech_reco_started:
            try:
                self.speech_reco.setVocabulary(["yes", "no"], False) # napraviti servis u  NaoqiBasicAwareness za puniti vocabular
            except RuntimeError:
                print "ASR already started"

            print "Starting speech recognition"
            self.speech_reco.subscribe("BasicAwareness_Test")
            self.is_speech_reco_started = True

    def stop_speech_reco(self):
        """ stop ASR when someone's detected in event handler class """
        if self.is_speech_reco_started:
            print "Stopping speech recognition"
            self.speech_reco.unsubscribe("BasicAwareness_Test")
            self.is_speech_reco_started = False

    def get_people_perception_data(self, id_person_tracked):
        """
            return information related to the person who has the id
            "id_person_tracked" from People Perception
        """
        memory_key = "PeoplePerception/Person/" + str(id_person_tracked) + \
                     "/PositionInRobotFrame"  #Torso world
        return self.memory.getData(memory_key)


    def get_people_gender_data(self, id_person_tracked):
        """
            return information related to the person who has the id
            "id_person_tracked" from People Perception ALFaceCharacteristics
        """
        memory_key = "PeoplePerception/Person/" + str(id_person_tracked) + \
                     "/GenderProperties"  # 0 - female, 1 - male
        return self.memory.getData(memory_key)


    def run(self):
        """
            ova metoda se nekoristi
        """
        # start
        print "Waiting for the robot to be in wake up position"
        self.motion.wakeUp()

        print "Starting BasicAwareness with the fully engaged mode"
        self.basic_awareness.setEngagementMode("FullyEngaged")
        self.basic_awareness.setTrackingMode("MoveContextually")
        self.basic_awareness.setEnabled(True)

        # loop on, wait for events until manual interruption
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, shutting down"
            # stop
            print "Stopping BasicAwareness"
            self.basic_awareness.setEnabled(False)

            self.stop_speech_reco()

            print "Waiting for the robot to be in rest position"
            self.motion.rest()

            sys.exit(0)


if __name__ == "__main__":

    #parser = argparse.ArgumentParser()
    #parser.add_argument("--ip", type=str, default="192.168.1.111", help="Robot IP address")
    #parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")

    #args = parser.parse_args()
    #print (" %s",args.ip)

    #app=connectQi(args.ip, args.port)


    pepper_ip = rospy.get_param('/pepper_ip')
    pepper_port = rospy.get_param('/pepper_port')
    pepper_name = rospy.get_param('/pepper_name')
    app=connectQi(pepper_ip, pepper_port)
    
    
    human_tracked_event_watcher = HumanTrackedEventWatcher(app)

    naoqi_basic_awareness = NaoqiBasicAwareness(human_tracked_event_watcher)


  
    #human_tracked_event_watcher.run()

    rospy.spin()

