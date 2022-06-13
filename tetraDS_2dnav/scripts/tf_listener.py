#!/usr/bin/env python  
import rospy
import tf

from nav_msgs.msg import Odometry

def tf_broad(msg):
    br = tf.TransformBroadcaster()
    # xyz, quaternion?? ?? sendTransform
    # map - base_link
    br.sendTransform((msg.pose.pose.position.x, 
                    msg.pose.pose.position.y, 
                    msg.pose.pose.position.z),
                    [msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w],
                    rospy.Time.now(),"","/map")

if __name__ == '__main__':
    rospy.init_node('tf_listener_py')
    rospy.Subscriber('tf_listener',Odometry,tf_broad)
    rospy.spin()
