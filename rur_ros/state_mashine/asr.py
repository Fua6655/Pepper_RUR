#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from smach_ros import *
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PickupActionGoal
import time
from naoqi_bridge_msgs.srv import (
    SetFloatResponse,
    SetFloat,
    SetStringResponse,
    SetString,
    GetFloatResponse,
    GetFloat)
from std_srvs.srv import (
    SetBool,
    SetBoolResponse)

from naoqi_bridge_msgs.msg import (BodyPoseWithSpeedActionGoal, JointTrajectoryActionGoal, SetSpeechVocabularyActionGoal, RunBehaviorActionGoal, WordRecognized)

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['time'])
    def execute(self, userdata):
        time.sleep(userdata.time) 
        return 'succeeded'


class GoToPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['pose', 'time'])
        self.pose_pub = rospy.Publisher('/body_pose_naoqi/goal', BodyPoseWithSpeedActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        pose_goal = BodyPoseWithSpeedActionGoal()
        pose_goal.goal.posture_name = userdata.pose
        pose_goal.goal.speed = 0.5
        self.pose_pub.publish(pose_goal)
        time.sleep(userdata.time) 
        return 'succeeded'

class Fill_Vocabulary(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['vocabulary_data'])
        self.vocabulary_pub = rospy.Publisher('/speech_vocabulary_action/goal', SetSpeechVocabularyActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        vocalubary_goal = SetSpeechVocabularyActionGoal()
        vocalubary_goal.goal.words = userdata.vocabulary_data
        self.vocabulary_pub.publish(vocalubary_goal)
        return 'succeeded'


class Run_Behavior(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['behavior', 'time'])
        self.behavior_pub = rospy.Publisher('/run_behavior/goal', RunBehaviorActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        behavior_goal = RunBehaviorActionGoal()
        behavior_goal.goal.behavior = userdata.behavior
        self.behavior_pub.publish(behavior_goal)
        time.sleep(userdata.time) 
        return 'succeeded'



def main():

    rospy.init_node('Stateovi')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    sm.userdata.time = 3.0
    sm.userdata.body_pose = "Stand"
    sm.userdata.vocabulary_data = 'Hello, Helena'
    sm.userdata.behavior = 'animations/Stand/Gestures/Stretch_2'
    sm.userdata.word_recognized = ""
    sm.userdata.word_conf = 0

    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('helena/wakeup', ServiceState('helena/wakeup', Empty),
                                transitions={'succeeded':'/helena/asr/vocabulary/set/goal1'})


################################ ASR TTS #########################################

        # Fill in the vocabulary
        sm.userdata.vocabulary_data = ['Hello', 'Helena', 'Sulla']
        smach.StateMachine.add('/helena/asr/vocabulary/set/goal1', SimpleActionState('/helena/asr/vocabulary/set/goal', 
                                        SetSpeechVocabularyActionGoal,
                                        goal_slots = ['vocabulary_data']),
                                transitions={'succeeded':'/helena/tts/animation1'})

        # Say word via tts service  and animate while talking                                                   
        smach.StateMachine.add('/helena/tts/animation1', ServiceState('/helena/tts/animation', SetString, 'Hello, human'),
                                transitions={'succeeded':'/helena/asr/enable1'})

        # enable asr                                 
        smach.StateMachine.add('/helena/asr/enable1', ServiceState('/helena/asr/enable', Empty),
                                transitions={'succeeded':'/helena/asr/word_recognized1'})

        # Goal callback for state wait_for_word
        def word_callback(userdata):
            
            sub = rospy.Subscriber('/helena/asr/word_recognized', WordRecognized, word_inspect_callback)
            if sub == succeeded:
                return succeeded
            rospy.spin()
            
            #print userdata.word
            #print userdata.word_conf


        def word_inspect_callback(data):
            print data.words
            for x in sm.userdata.vocabulary_data:
                for y in data.words:
                    if x == y:
                        sm.userdata.word_recognized = x
                        return succeeded
            return preempted

        #Get detected word                        
        smach.StateMachine.add('/helena/asr/word_recognized1', CBState(word_callback), 
                                    transitions={'succeeded':'/helena/asr/word_recognized1'})

        # disable asr
        smach.StateMachine.add('/helena/asr/disable1', ServiceState('/helena/asr/disable', Empty),
                                transitions={'succeeded':'/helena/tts/animation2'})


        # Say word via tts service  and animate while talking                                                   
        smach.StateMachine.add('/helena/tts/animation2', ServiceState('/helena/tts/animation', SetString, sm.userdata.word_recognized),
                                transitions={'succeeded':'/helena/rest'})

############# REST
        # Go to rest pose
        smach.StateMachine.add('/helena/rest', ServiceState('/helena/rest', Empty),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

    #########################################################################
    # Create and start the introspection server
    sis = IntrospectionServer('my_smach_introspection_server', sm, '/Test ASR')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
