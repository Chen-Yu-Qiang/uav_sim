#!/usr/bin/env python
import roslib
# roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    br3 = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.7071068, 0, 0.7071068, 0),
                         rospy.Time.now(),
                         "org2",
                         "world")
        br2.sendTransform((0.0, 0.0, 0.0),
                         (0, 0, -0.3826834, 0.9238795),
                         rospy.Time.now(),
                         "org",
                         "org2")                 
        br3.sendTransform((1.5, 0.27, 0.0),
                         (0,0,0,1),
                         rospy.Time.now(),
                         "uavstart",
                         "org")
        try:
            (trans, rot) = listener.lookupTransform("org", "camera_position", rospy.Time(0))
            print(trans)
        except:
            pass
        rate.sleep()
