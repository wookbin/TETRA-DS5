#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    try:
        rospy.init_node("rob_glob_Pose")
        listener = tf.TransformListener()
        gpose_publisher = rospy.Publisher("robot_global_pose", PoseStamped, queue_size=1000)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            try:
                t = rospy.Time.now()
                #listener.waitForTransform('/map', '/imu_frame', t, rospy.Duration(4.0))
                (trans,rot) = listener.lookupTransform('/map', '/imu_frame', rospy.Time(0))
                
                gpose = PoseStamped()

                gpose.header.stamp = t
                gpose.header.frame_id = "map"

                gpose.pose.position.x = trans[0]
                gpose.pose.position.y = trans[1]
                gpose.pose.position.z = trans[2]

                gpose.pose.orientation.x = rot[0]
                gpose.pose.orientation.y = rot[1]
                gpose.pose.orientation.z = rot[2]
                gpose.pose.orientation.w = rot[3]
                gpose_publisher.publish(gpose)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()

    except rospy.ROSInterruptException:
        pass  
    
