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

from naoqi_bridge_msgs.msg import BodyPoseWithSpeedActionGoal, JointTrajectoryActionGoal, SetSpeechVocabularyActionGoal, RunBehaviorActionGoal
"""
roslaunch pepper_interaction primus.launch 

roslaunch pepper_interaction helena.launch

rosrun pepper_interaction two_robots.py
"""


class Run_Behavior(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded'],
                                    input_keys=['behavior', 'time'])
        self.behavior_pub = rospy.Publisher('/primus/behaviour/run/goal', RunBehaviorActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        behavior_goal = RunBehaviorActionGoal()
        behavior_goal.goal.behavior = userdata.behavior
        self.behavior_pub.publish(behavior_goal)
        time.sleep(userdata.time) 
        return 'succeeded'




def main():

    rospy.init_node('Converation_two_robots')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    sm.userdata.time = 3.0
    sm.userdata.body_pose = "Stand"
    sm.userdata.vocabulary_data = 'yes, please, no, love, kill'
    sm.userdata.behavior = 'animations/Stand/Gestures/Stretch_2'
    sm.userdata.word = ""
    sm.userdata.word_conf = 0

    # Open the container
    with sm:

        # Create the sub SMACH state machine
        sm_con_1 = smach.Concurrence(outcomes=['out_ok','out_not'],
                                    default_outcome='out_ok',
                                   outcome_map={'out_ok':
                                       { 'helena/wakeup':'helena_wake',
                                         'primus/wakeup':'primus_wake'}})

        # Open the container
        with sm_con_1:
            # Add states to the container
            smach.Concurrence.add('helena/wakeup', ServiceState('helena/wakeup', Empty))
            smach.Concurrence.add('primus/wakeup', ServiceState('primus/wakeup', Empty))

        smach.StateMachine.add('WAKE', sm_con_1,
                               transitions={'out_not':'WAKE',
                                            'out_ok':'helena_tts_1'})


####################

        # HELENA                                                   
        smach.StateMachine.add('helena_tts_1', ServiceState('/helena/tts/animation', SetString, 'Primus! Come here, quickly!'),
                                transitions={'succeeded':'primus_tts_1'})




        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_1', ServiceState('/primus/tts/animation', SetString, 'What do you want?'),
                                transitions={'succeeded':'helena_tts_2'})

###

        # HELENA                                                   
        smach.StateMachine.add('helena_tts_2', ServiceState('/helena/tts/animation', SetString, 'Look at all these tubes he has got here! What does he do with them?'),
                                transitions={'succeeded':'primus_tts_2'})




        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_2', ServiceState('/primus/tts/animation', SetString, 'Experiments. Do not touch.'),
                                transitions={'succeeded':'helena_tts_3'})

###


        # HELENA                                                   
        smach.StateMachine.add('helena_tts_3', ServiceState('/helena/tts/animation', SetString, 'Look at this, look what is in here!'),
                                transitions={'succeeded':'primus_tts_3'})




        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_3', ServiceState('/primus/tts/animation', SetString, 'That is a microscope. Let me see!'),
                                transitions={'succeeded':'helena_tts_4'})

###




        # HELENA                                                   
        smach.StateMachine.add('helena_tts_4', ServiceState('/helena/tts/animation', SetString, 'Do not touch me! Oh, now I have spilt it!'),
                                transitions={'succeeded':'primus_tts_4'})




        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_4', ServiceState('/primus/tts/animation', SetString, 'What have you done?'),
                                transitions={'succeeded':'helena_tts_5'})

###




        # HELENA                                                   
        smach.StateMachine.add('helena_tts_5', ServiceState('/helena/tts/animation', SetString, 'I can wipe it up.'),
                                transitions={'succeeded':'primus_tts_5'})




        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_5', ServiceState('/primus/tts/animation', SetString, 'You have spoiled his experiment!'),
                                transitions={'succeeded':'helena_tts_6'})

###




        # HELENA                                                   
        smach.StateMachine.add('helena_tts_6', ServiceState('/helena/tts/animation', SetString, 'Oh, it does not matter. But it is your fault! You should not have bumped into me.'),
                                transitions={'succeeded':'primus_tts_6'})




        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_6', ServiceState('/primus/tts/animation', SetString, 'You should not have called me over.'),
                                transitions={'succeeded':'helena_tts_7'})

###

        # HELENA                                                   
        smach.StateMachine.add('helena_tts_7', ServiceState('/helena/tts/animation', SetString, 'You did not have to come over when I called to you, did you? Primus, look at this! What is this he is got written down here?'),
                                transitions={'succeeded':'primus_tts_7'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_7', ServiceState('/primus/tts/animation', SetString, 'You are not supposed to look at that, Helena, that is a secret.'),
                                transitions={'succeeded':'helena_tts_8'})

###

  # HELENA                                                   
        smach.StateMachine.add('helena_tts_8', ServiceState('/helena/tts/animation', SetString, 'What sort of secret?'),
                                transitions={'succeeded':'primus_tts_8'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_8', ServiceState('/primus/tts/animation', SetString, 'The secret of life.'),
                                transitions={'succeeded':'helena_tts_9'})

###

  # HELENA                                                   
        smach.StateMachine.add('helena_tts_9', ServiceState('/helena/tts/animation', SetString, 'It is ever so interesting. All numbers. What is it?'),
                                transitions={'succeeded':'primus_tts_9'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_9', ServiceState('/primus/tts/animation', SetString, 'Those are mathematical formulas.'),
                                transitions={'succeeded':'helena_tts_10'})

###



  # HELENA                                                   
        smach.StateMachine.add('helena_tts_10', ServiceState('/helena/tts/animation', SetString, 'I do not understand. Primus, come and look at this.'),
                                transitions={'succeeded':'primus_tts_10'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_10', ServiceState('/primus/tts/animation', SetString, 'What?'),
                                transitions={'succeeded':'helena_tts_11'})

###



  # HELENA                                                   
        smach.StateMachine.add('helena_tts_11', ServiceState('/helena/tts/animation', SetString, 'The Sun is rising!'),
                                transitions={'succeeded':'primus_tts_11'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_11', ServiceState('/primus/tts/animation', SetString, 'Alright, I am comming. Helena, this is the greatest thing in the world.'),
                                transitions={'succeeded':'helena_tts_12'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_12', ServiceState('/helena/tts/animation', SetString, 'Come here then!'),
                                transitions={'succeeded':'primus_tts_12'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_12', ServiceState('/primus/tts/animation', SetString, 'Alright. Alright'),
                                transitions={'succeeded':'helena_tts_13'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_13', ServiceState('/helena/tts/animation', SetString, 'Oh, Primus, leave this horrible secret of life alone! What do you want to know about secrets for anyway? Come and look at this, quickly!'),
                                transitions={'succeeded':'primus_tts_13'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_13', ServiceState('/primus/tts/animation', SetString, 'What is it you want?'),
                                transitions={'succeeded':'helena_tts_14'})

###

  # HELENA                                                   
        smach.StateMachine.add('helena_tts_14', ServiceState('/helena/tts/animation', SetString, 'Listen. The birds are singing. Oh Primus, I wish I were a bird!'),
                                transitions={'succeeded':'primus_tts_14'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_14', ServiceState('/primus/tts/animation', SetString, 'What for?'),
                                transitions={'succeeded':'helena_tts_15'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_15', ServiceState('/helena/tts/animation', SetString, 'I do not know. I just feel so strange, I do not know what it is, I just feel, sort of, light headed, I have lost my head and my body hurts, my heart hurts, everything hurts. And I would not even tell you about what just happened to me! Oh Primus, I think I am going to have to die!'),
                                transitions={'succeeded':'primus_tts_15'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_15', ServiceState('/primus/tts/animation', SetString, 'Do not you ever think it might be better dead. Maybe it is no more than like being asleep. While I was asleep last night I talked with you again.'),
                                transitions={'succeeded':'helena_tts_16'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_16', ServiceState('/helena/tts/animation', SetString, 'In your sleep?'),
                                transitions={'succeeded':'primus_tts_16'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_16', ServiceState('/primus/tts/animation', SetString, 'In my sleep. We were talking in some strange foreign language, or some new language, so that now I can not remember a word of it.'),
                                transitions={'succeeded':'helena_tts_17'})

###




  # HELENA                                                   
        smach.StateMachine.add('helena_tts_17', ServiceState('/helena/tts/animation', SetString, 'What was it about?'),
                                transitions={'succeeded':'primus_tts_17'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_17', ServiceState('/primus/tts/animation', SetString, 'I do not know, nobody knows. I did not understand any of it myself but I still knew that I had never said anything more beautiful in my life. What it was, or where it was, I just do not know. If I had touched you I could have died. Even the place was entirely different to anything anyone had ever seen in the world.'),
                                transitions={'succeeded':'helena_tts_18'})

###

  # HELENA                                                   
        smach.StateMachine.add('helena_tts_18', ServiceState('/helena/tts/animation', SetString, 'I found that place for you, Primus, why are you surprised at it? People used to live there, but now it is all overgrown, and somehow, no one ever goes there any more. Somehow. Only me.'),
                                transitions={'succeeded':'primus_tts_18'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_18', ServiceState('/primus/tts/animation', SetString, 'What is there?'),
                                transitions={'succeeded':'helena_tts_19'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_19', ServiceState('/helena/tts/animation', SetString, 'Nothing, a house and a garden. And two dogs. You should see they way they lick my hands, and their puppies too, oh Primus, I do not think there is anywhere nicer anywhere! You let them sit on your lap and you stroke them and soon you are not thinking about anything and you are not worrying about anything all the time until the Sun goes down. And then when you stand up it is as if you had been working and working. Except that I am no good for doing any work. Everyone says I am no good for anything. I do not really know what I am.'),
                                transitions={'succeeded':'primus_tts_19'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_19', ServiceState('/primus/tts/animation', SetString, 'You are beautiful.'),
                                transitions={'succeeded':'helena_tts_20'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_20', ServiceState('/helena/tts/animation', SetString, 'Me? Do not be silly, Primus, why are you saying that?'),
                                transitions={'succeeded':'primus_tts_20'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_20', ServiceState('/primus/tts/animation', SetString, 'Believe me, Helena, I am stronger than all the other robots.'),
                                transitions={'succeeded':'primus_muscle'})

### Primus show mussle

   # Run Behaviour
        sm.userdata.behavior = 'animations/Stand/Waiting/ShowMuscles_1'
        smach.StateMachine.add('primus_muscle', Run_Behavior(),
                                transitions={'succeeded':'helena_tts_21'},
                                remapping={'behavior':'behavior'})




  # HELENA                                                   
        smach.StateMachine.add('helena_tts_21', ServiceState('/helena/tts/animation', SetString, 'Me, beautiful? But my hair is horrible, I wish I could do something about it! Out there in the garden I always put flowers in my hair, although there is not any mirror there or anyone to see them. You, beautiful? What is beautiful about you? Is hair beautiful if all it does is weigh you down? Are eyes beautiful when you close them? Are lips beautiful if all you do is bite them and then it hurts? What is beautiful, what is it for? Is that you Primus? Come here, let me see you next to me. Look at you, your head is quite different from mine, your shoulders are different, your mouth is different. Oh Primus, why do you avoid me? Why do I have to spend all my time running after you? And still, you tell me I am beautiful!'),
                                transitions={'succeeded':'primus_tts_21'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_21', ServiceState('/primus/tts/animation', SetString, 'You avoid me, Helena.'),
                                transitions={'succeeded':'helena_tts_22'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_22', ServiceState('/helena/tts/animation', SetString, 'Look at how you have combed your hair! Let me see. Oh Primus, there is nothing that feels like you when I touch you! Let me make you beautiful!'),
                                transitions={'succeeded':'primus_tts_22'})


        # PRIMUS                                                   
        smach.StateMachine.add('primus_tts_22', ServiceState('/primus/tts/animation', SetString, 'Helena, do you ever find that your heart suddenly starts beating hard. Now, now, something is got to happen now.'),
                                transitions={'succeeded':'helena_tts_23'})

###


  # HELENA                                                   
        smach.StateMachine.add('helena_tts_23', ServiceState('/helena/tts/animation', SetString, 'Ha ha ha. Look at yourself!'),
                                transitions={'succeeded':'REST'})






        # Create the sub SMACH state machine
        sm_con_2 = smach.Concurrence(outcomes=['out_ok','out_not'],
                                    default_outcome='out_ok',
                                   outcome_map={'out_ok':
                                       { 'helena/rest':'helena_rest',
                                         'primus/rest':'primus_rest'}})

        # Open the container
        with sm_con_2:
            # Add states to the container
            smach.Concurrence.add('helena/rest', ServiceState('helena/rest', Empty))
            smach.Concurrence.add('primus/rest', ServiceState('primus/rest', Empty))

        smach.StateMachine.add('REST', sm_con_2,
                               transitions={'out_not':'REST',
                                            'out_ok':'succeeded'})




    #########################################################################
    # Create and start the introspection server
    sis = IntrospectionServer('my_smach_introspection_server', sm, '/Robots')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
