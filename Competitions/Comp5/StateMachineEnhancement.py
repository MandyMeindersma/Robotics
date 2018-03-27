#!/usr/bin/env python
import rospy
import smach
import smach_ros
import time

class LearnFaces(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['know']) 

    def execute(self, userdata):
        rospy.loginfo('Executing LearnFaces state')
        # TODO: code to learn faces
        time.sleep(99999999999999999)
        return 'know'

class GoHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived'])

    def execute(self, userdata):
        rospy.loginfo('Executing GoHome state')
        # TODO: code to get to home location FOR ROBOT
        return 'arrived'

class LookingForFace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found','not_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing LookingForFace state')
        see_face = False
        # TODO: code to recognize faces
        if see_face:
            return 'found'
        else:
            return 'not_found'

class MatchToHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found','not_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing MatchToHome state')
        match = False
        # TODO: code to match a face to a human home
        if match:
            return 'found'
        else:
            return 'not_found'

class Guide(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived','human_following','human_lost'])

    def execute(self, userdata):
        rospy.loginfo('Executing Guide state')
        humanFound = True
        arrived = False
        # TODO: code to lead the human to their home
        if not arrived:
            if humanFound:
                return 'human_following'
            else:
                return 'human_lost'
        else:
            return 'arrived'
        
class FindHuman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found','not_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing Guide state')
        found = True
        # TODO: code to make sure a human is following
        if found:
            return 'found'
        else:
            return 'not_found'

def main():
    rospy.init_node('GuideBot')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LearnFaces', LearnFaces(), 
                               transitions={'know':'GoHome'})
        smach.StateMachine.add('GoHome', GoHome(), 
                               transitions={'arrived':'LookingForFace'})
        smach.StateMachine.add('LookingForFace', LookingForFace(), 
                               transitions={'found':'MatchToHome',
                                            'not_found':'LookingForFace'})
        smach.StateMachine.add('MatchToHome', MatchToHome(), 
                               transitions={'found':'Guide',
                                            'not_found':'LookingForFace'})
        smach.StateMachine.add('Guide', Guide(), 
                               transitions={'arrived':'GoHome',
                                            'human_following':'Guide',
                                            'human_lost':'FindHuman'})
        smach.StateMachine.add('FindHuman', FindHuman(), 
                               transitions={'found':'Guide',
                                            'not_found':'FindHuman'})

 
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/GUIDE_BOT_ENHANCEMENT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
   
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
