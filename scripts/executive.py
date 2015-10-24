#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_usecase')
import rospy
import std_srvs.srv
import smach
import smach_ros
import turtlesim.srv
import turtle_actionlib.msg
from smach_ros import ServiceState
from smach_ros import SimpleActionState

def main():
    rospy.init_node('smach_usecase_executive')

    sm_root = smach.StateMachine(outcomes=['succeeded','preempted','aborted']) 
    
    with sm_root:
        #add state to call ROS service by turtlesim to reset sim
        smach.StateMachine.add('RESET',
            ServiceState('reset',
            std_srvs.srv.Empty),
            transitions={'succeeded':'SPAWN'})
        #add state to spawn new turtle at 0,0 called turtle2
        smach.StateMachine.add('SPAWN',
            ServiceState('spawn',
            turtlesim.srv.Spawn,
            request = turtlesim.srv.SpawnRequest(0,0,0,'turtle2')),
            transitions={'succeeded':'TELEPORT1'})

        smach.StateMachine.add('TELEPORT1',
            ServiceState('turtle1/teleport_absolute',
            turtlesim.srv.TeleportAbsolute,
            request = turtlesim.srv.TeleportAbsoluteRequest(5.0,1.0,0)),
            transitions={'succeeded':'TELEPORT2'})
            
        smach.StateMachine.add('TELEPORT2',
            ServiceState('turtle2/teleport_absolute',
            turtlesim.srv.TeleportAbsolute,
            request = turtlesim.srv.TeleportAbsoluteRequest(9.0,5.0,0)),
            transitions={'succeeded':'DRAW_SHAPES'})
        
        # Create concurrence container
        sm_con = smach.Concurrence( outcomes=['succeeded','preempted','aborted'],
            default_outcome = 'aborted',
            outcome_map={'succeeded': {'BIG':'succeeded' , 'SMALL':'succeeded'}})

        smach.StateMachine.add('DRAW_SHAPES', sm_con,
            transitions={'succeeded': 'succeeded'})

        with sm_con:

            smach.Concurrence.add('BIG',
                SimpleActionState('turtle_shape1',
                turtle_actionlib.msg.ShapeAction,
                goal=turtle_actionlib.msg.ShapeGoal(edges = 11, radius = 4.0) ))
            
            smach.Concurrence.add('SMALL',
                SimpleActionState('turtle_shape2',
                turtle_actionlib.msg.ShapeAction,
                goal=turtle_actionlib.msg.ShapeGoal(edges = 6, radius = 0.5) ))

        

     #start introspection server to use smach_viewer.py
    sis = smach_ros.IntrospectionServer('server_name', sm_root, '/SM_ROOT')
    sis.start()
    
    outcome = sm_root.execute()

    rospy.spin()

if __name__=='__main__':
    main()
