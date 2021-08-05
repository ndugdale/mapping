#!/usr/bin/env
import rospy
#import roslaunch
import std_msgs.msg

def callback(string_msg):
    rospy.sleep(1)

    # print message showing save command
    fname = 'src/mapping/bt/sonar_dvl/' + string_msg.data + '.bt'
    print('\nTo save the OctoMap as a .bt file:\n')
    print('1) Run the following command in another terminal tab/window:')
    print('>>> rosrun octomap_server octomap_saver ' + fname)
    print('2) The program can be exited now\n')
    



def launch_save():
    # initialise node and subscribter
    rospy.init_node('save_helper')
    rospy.Subscriber('/save',std_msgs.msg.String,callback)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    while not rospy.is_shutdown():

        rospy.spin()
    
    else:
        rate.sleep()


if __name__ == '__main__':
    try:
        launch_save()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass