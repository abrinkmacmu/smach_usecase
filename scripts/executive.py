#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_usecase')
import rospy
import threading
import std_srvs.srv
import smach
import smach_ros
import turtlesim.srv
import turtlesim.msg
import turtle_actionlib.msg
from smach_ros import ServiceState
from smach_ros import SimpleActionState
from smach_ros import MonitorState
turtle2pose = turtlesim.msg.Pose
def turtle2pose_cb(data):
    global turtle2pose
    turtle2pose = data

def monitor_callback(ud, turtle1pose):
    #rospy.loginfo('turtle1pose x: %f',turtle1pose.x )
    #rospy.loginfo('turtle2pose x: %f',turtle2pose.x )

    global turtle2pose
    dist = ((turtle1pose.x-turtle2pose.x)**2.0 + (turtle1pose.y-turtle2pose.y)**2.0 )**(1/2.0)
    rospy.loginfo('distance between turtles: %s',dist)
    if(dist < 1.0):
        return False
    else:    
        return True

def child_term_cb(outcome_map):
    if outcome_map['MONITOR'] == 'invalid':
        return True
    else:
        return False



def main():
    rospy.init_node('smach_usecase_executive')

    rospy.Subscriber("turtle2/pose", turtlesim.msg.Pose, turtle2pose_cb)

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
            
            #Create Small container to run the DRAW state and MONITOR state
            sm_small_monitor = smach.Concurrence( outcomes=['succeeded','preempted','aborted'],
                default_outcome = 'aborted',
                child_termination_cb = child_term_cb,
                outcome_map={'succeeded': {'DRAW': 'succeeded'},
                             'preempted': {'DRAW': 'preempted', 'MONITOR':'preempted'},
                             'aborted'  : {'MONITOR': 'invalid'} })

            smach.Concurrence.add('SMALL', sm_small_monitor)

            with sm_small_monitor:

                smach.Concurrence.add('DRAW',
                    SimpleActionState('turtle_shape2',
                    turtle_actionlib.msg.ShapeAction,
                    goal=turtle_actionlib.msg.ShapeGoal(edges = 6, radius = 0.5) ))

                smach.Concurrence.add('MONITOR',
                    MonitorState("turtle1/pose",
                    turtlesim.msg.Pose,
                    monitor_callback))

        

     #start introspection server to use smach_viewer.py
    sis = smach_ros.IntrospectionServer('server_name', sm_root, '/SM_ROOT')
    sis.start()



    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm_root.execute)
    smach_thread.start()
    
    #outcome = sm_root.execute()

    rospy.spin()

    sis.stop()

    sm_root.request_preempt()

    

if __name__=='__main__':
    main()
