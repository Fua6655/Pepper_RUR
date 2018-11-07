#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import sys
import time
import rospy

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from optparse import OptionParser
from naoqi_driver.naoqi_node import NaoqiNode
from std_msgs.msg import (String)
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
from naoqi_bridge_msgs.msg import (SoundLocated)
# Global variable to store the SoundGreeter module instance
SoundGreeter = None
memory = None


class NaoqiSoundModule (NaoqiNode):
    def __init__(self, sound_tracked_event_watcher_session):

        NaoqiNode.__init__(self, 'naoqi_sound_local')
        #sad ide poziv druge klase
        self.proxy = sound_tracked_event_watcher_session
        self.pname = rospy.get_param('/pepper_name')

        self.enable_soundloc = rospy.Service(self.pname+"/sound_localization/enable", Empty, self.handleEnableSoundLoca)
        self.disable_soundloc = rospy.Service(self.pname+"/sound_localization/disable", Empty, self.handleDisableSoundLoca)
        rospy.loginfo("naoqi_sound_loc is initialized")

        global sound_located
        sound_located = ALProxy("ALSoundLocalization")
        global memory
        memory = ALProxy("ALMemory")

    def handleEnableSoundLoca(self, req):
        try:
            sound_located.subscribe("Sulla")
            memory.subscribeToMicroEvent("ALSoundLocalization/SoundLocated","SoundGreeter","Message","onSoundDetected")
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res


    def handleDisableSoundLoca(self, req):
        try:
            sound_located.unsubscribe("Sulla")
            memory.unsubscribeToMicroEvent("ALSoundLocalization/SoundLocated","SoundGreeter")
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res



class SoundGreeterModule(ALModule):

    def __init__(self, name):
        ALModule.__init__(self, name)
        print "Inicializacija"
        self.pname = rospy.get_param('/pepper_name')
        self.pub_sound = rospy.Publisher(self.pname+"/sound/located", SoundLocated, queue_size=10)
   
    def onSoundDetected(self, *args, **kwargs):
        #memory.unsubscribeToEvent("ALSoundLocalization/SoundLocated","SoundGreeter")
        #print args
        #memory.subscribeToMicroEvent("ALSoundLocalization/SoundLocated","SoundGreeter","Message","onSoundDetected")

        self.data = SoundLocated()

        self.data.header.stamp.secs = args[1][0][0]
        self.data.header.stamp.nsecs = args[1][0][1]
        self.data.azimuth = args[1][1][0]
        self.data.elevation = args[1][1][1]
        self.data.confidence = args[1][1][2]
        self.data.energy = args[1][1][3]
        
        self.data.head_position_frame_torso.linear.x = args[1][2][0]
        self.data.head_position_frame_torso.linear.y = args[1][2][1]
        self.data.head_position_frame_torso.linear.z = args[1][2][2]
        self.data.head_position_frame_torso.angular.x = args[1][2][3]
        self.data.head_position_frame_torso.angular.y = args[1][2][4]
        self.data.head_position_frame_torso.angular.z = args[1][2][5]

        self.data.head_position_frame_robot.linear.x = args[1][3][0]
        self.data.head_position_frame_robot.linear.y = args[1][3][1]
        self.data.head_position_frame_robot.linear.z = args[1][3][2]
        self.data.head_position_frame_robot.angular.x = args[1][3][3]
        self.data.head_position_frame_robot.angular.y = args[1][3][4]
        self.data.head_position_frame_robot.angular.z = args[1][3][5]

        #print self.data

        rate = rospy.Rate(10) # 10hz
        self.pub_sound.publish(self.data)
        rate.sleep()


if __name__ == "__main__":

    pepper_ip = rospy.get_param('/pepper_ip')
    pepper_port = rospy.get_param('/pepper_port')
    pepper_name = rospy.get_param('/pepper_name')
 
    myBroker = ALBroker("myBroker", "0.0.0.0", 0, pepper_ip, pepper_port)  
     
    global SoundGreeter
    SoundGreeter = SoundGreeterModule("SoundGreeter")
    naoqi_sound = NaoqiSoundModule(SoundGreeter)

    rospy.spin()
