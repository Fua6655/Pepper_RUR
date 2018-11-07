#!/usr/bin/env python
import rospy
import actionlib
from naoqi_bridge_msgs.msg import(AudioBuffer, SoundLocated)
from naoqi_bridge_msgs.srv import(GetFloat)
from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String, Int32
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse


class AudioController(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'audio_server')

        self.pip = rospy.get_param('/pepper_ip')
        self.pport = rospy.get_param('/pepper_port')
        self.pname = rospy.get_param('/pepper_name')

        self.connectNaoQi()



        # advertise topics:
        self.audioBuffer = rospy.Publisher(self.pname+"/audio_buffer", AudioBuffer, queue_size=10)
        #self.soundLocated = rospy.Publisher(self.pname+"/sound/located", SoundLocated, queue_size=10) definirano u drugom serveru

        # start services / actions:
        self.enableStiffnessSrv = rospy.Service(self.pname+"/energy_computation/enable", Empty, self.energy_computation)
        self.disableStiffnessSrv = rospy.Service(self.pname+"/energy_computation/disable", Empty, self.energy_computation_off)
        self.setOutputVolume = rospy.Service(self.pname+"/output_volume/set", GetFloat, self.set_output_volume)

        rospy.loginfo("sound initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.audioDeviceProxy = self.get_proxy("ALAudioDevice")
        if self.audioDeviceProxy is None:
            exit(1)

        self.soundDetectionProxy = self.get_proxy("ALSoundDetection")
        self.soundLocalizationProxy = self.get_proxy("ALSoundLocalization")



    def energy_computation(self, req):
        try:
            self.audioDeviceProxy.enableEnergyComputation()
            rospy.loginfo("Energy Computation enabled")

            self.audio_msg = AudioBuffer()
            self.audio_msg.header.frame_id = "torso" 
            #self.audio_msg.channelMap = ["Front", "Rear", "Left", "Right"]
            self.audio_msg.channelMap = [0, 1, 2, 3]

            self.audio_msg.header.stamp = rospy.Time.now()
            self.frontMicEnergy = self.audioDeviceProxy.getFrontMicEnergy()
            self.rearMicEnergy = self.audioDeviceProxy.getRearMicEnergy()
            self.leftMicEnergy = self.audioDeviceProxy.getLeftMicEnergy()
            self.rightMicEnergy = self.audioDeviceProxy.getRightMicEnergy()

            self.audio_msg.data = [self.frontMicEnergy, self.rearMicEnergy, self.leftMicEnergy, self.rightMicEnergy]

            self.rate = rospy.Rate(10) 
            while not rospy.is_shutdown():
                self.frontMicEnergy = self.audioDeviceProxy.getFrontMicEnergy()
                self.rearMicEnergy = self.audioDeviceProxy.getRearMicEnergy()
                self.leftMicEnergy = self.audioDeviceProxy.getLeftMicEnergy()
                self.rightMicEnergy = self.audioDeviceProxy.getRightMicEnergy()
                
                self.audio_msg.header.stamp = rospy.Time.now()
                self.audio_msg.data = [self.frontMicEnergy, self.rearMicEnergy, self.leftMicEnergy, self.rightMicEnergy]

                print self.audio_msg.data
                self.audioBuffer.publish(self.audio_msg)
                self.rate.sleep()

           
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

 

    def energy_computation_off(self, req):
        try:
            self.audioDeviceProxy.disableEnergyComputation()
            rospy.loginfo("Energy Computation disable")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None



    def set_output_volume(self, req):
        try:
            self.audioDeviceProxy.setOutputVolume(int(req))
            rospy.loginfo("seting output volume")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None


    
if __name__ == '__main__':

    controller = AudioController()
    controller.start()
    rospy.loginfo("nao sound server running...")
    rospy.spin()

    rospy.loginfo("nao sound server stopped.")
    exit(0)
