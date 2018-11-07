#!/usr/bin/env python
import rospy
import actionlib
from naoqi_bridge_msgs.msg import(
    JointTrajectoryResult,
    JointTrajectoryAction,
    JointAnglesWithSpeed,
    JointAnglesWithSpeedResult,
    JointAnglesWithSpeedAction,
    BodyPoseWithSpeedAction,
    BodyPoseWithSpeedGoal,
    BodyPoseWithSpeedResult)

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from sensor_msgs.msg import JointState

class PoseController(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'pose_controller')

        self.pip = rospy.get_param('/pepper_ip')
        self.pport = rospy.get_param('/pepper_port')
        self.pname = rospy.get_param('/pepper_name')

        #self.pip = "192.168.1.111"
        #self.pport = 9559
        self.connectNaoQi()

        self.rate = 10

        # store the number of joints in each motion chain and collection, used for sanity checks
        self.collectionSize = {}
        for collectionName in ['Head', 'LArm', 'LHand', 'RArm', 'RHand', 'Body', 'BodyJoints', 'BodyActuators']:
            try:
                self.collectionSize[collectionName] = len(self.motionProxy.getJointNames(collectionName));
            except RuntimeError:
                # the following is useful for old NAOs with no legs/arms
                rospy.logwarn('Collection %s not found on your robot.' % collectionName)

        # Get poll rate for actionlib (ie. how often to check whether currently running task has been preempted)
        # Defaults to 200ms
        self.poll_rate = int(rospy.get_param("~poll_rate", 0.2)*1000)

        # initial stiffness (defaults to 0 so it doesn't strain the robot when no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)

        # advertise topics:
        self.getLifeStatePub = rospy.Publisher(self.pname+"/life/get_state", String, queue_size=10)

        # start services / actions:
        self.enableStiffnessSrv = rospy.Service(self.pname+"/body_stiffness/enable", Empty, self.handleStiffnessSrv)
        self.disableStiffnessSrv = rospy.Service(self.pname+"/body_stiffness/disable", Empty, self.handleStiffnessOffSrv)
        self.wakeUpSrv = rospy.Service(self.pname+"/wakeup", Empty, self.handleWakeUpSrv)
        self.restSrv = rospy.Service(self.pname+"/rest", Empty, self.handleRestSrv)
        self.enableLifeSrv = rospy.Service(self.pname+"/life/enable", Empty, self.handleLifeSrv)
        self.disableLifeSrv = rospy.Service(self.pname+"/life/disable", Empty, self.handleLifeOffSrv)
        self.getLifeSrv = rospy.Service(self.pname+"/life/get_state", Trigger, self.handleGetLifeSrv)


       
        # only start when ALRobotPosture proxy is available
        if not (self.robotPostureProxy is None):
            self.bodyPoseWithSpeedServer = actionlib.SimpleActionServer(self.pname+"/body_pose", BodyPoseWithSpeedAction,
                                              execute_cb=self.executeBodyPoseWithSpeed,
                                              auto_start=False)
            self.bodyPoseWithSpeedServer.start()
        else:
            rospy.logwarn("Proxy to ALRobotPosture not available, requests to body_pose_naoqi will be ignored.")

      

        rospy.loginfo("nao_controller initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

        # optional, newly introduced in 1.14
        self.robotPostureProxy = self.get_proxy("ALRobotPosture")

        # get a proxy to autonomous life
        self.lifeProxy = self.get_proxy("ALAutonomousLife")



    def handleStiffnessSrv(self, req):
        try:
            self.motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
            rospy.loginfo("Body stiffness enabled")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleStiffnessOffSrv(self, req):
        try:
            self.motionProxy.stiffnessInterpolation("Body", 0.0, 0.5)
            rospy.loginfo("Body stiffness removed")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleWakeUpSrv(self, req):
        try:
            self.motionProxy.wakeUp()
            rospy.loginfo("Wake Up")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleRestSrv(self, req):
        try:
            self.motionProxy.rest()
            rospy.loginfo("Rest")
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleLifeSrv(self, req):
        try:
            self.lifeProxy.setState("solitary")
            rospy.loginfo("set life state to solitary")
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception while setting life state:\n%s", e)
            return None

    def handleLifeOffSrv(self, req):
        try:
            self.lifeProxy.setState("disabled")
            rospy.loginfo("set life state to disabled")
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception while disabling life state:\n%s", e)
            return None

    def handleGetLifeSrv(self, req):
        try:
            res = TriggerResponse()
            res.success = True
            res.message = self.lifeProxy.getState()
            rospy.loginfo("current life state is " + str(res.message))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception while getting life state:\n%s", e)
            return None

 
    def executeBodyPoseWithSpeed(self, goal):

        #~ Sanity checks
        if (goal.speed < 0.0) or (goal.speed > 1.0):
            bodyPoseWithSpeedResult = BodyPoseWithSpeedResult()
            self.bodyPoseWithSpeedServer.set_aborted(bodyPoseWithSpeedResult)
            rospy.logerr("Body pose setter: Not a valid speed value.")
            return

        valid_postures = self.robotPostureProxy.getPostureList()

        if goal.posture_name not in valid_postures:
            bodyPoseWithSpeedResult = BodyPoseWithSpeedResult()
            self.bodyPoseWithSpeedServer.set_aborted(bodyPoseWithSpeedResult)
            rospy.logerr("Body pose setter: Not a valid posture.")
            return

        #~ Must set stiffness on
        try:
            self.motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
            rospy.loginfo("Body stiffness enabled")
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return

        #~ Go to posture. This is blocking
        self.robotPostureProxy.goToPosture(goal.posture_name, goal.speed)
        #~ Return success
        self.bodyPoseWithSpeedServer.set_succeeded()

    def run(self):
        while self.is_looping():
            try:
                if self.getLifeStatePub.get_num_connections() > 0:
                    get_life_state_msg = String()
                    get_life_state_msg.data = self.lifeProxy.getState()
                    self.getLifeStatePub.publish(get_life_state_msg)

            except RuntimeError, e:
                print "Error accessing ALMotion, ALRobotPosture, ALAutonomousLife, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

            #self.rate.sleep()

if __name__ == '__main__':

    controller = PoseController()
    controller.start()
    rospy.loginfo("nao pose_controller running...")
    rospy.spin()

    rospy.loginfo("nao pose_controller stopped.")
    exit(0)
