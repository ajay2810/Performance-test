#!/usr/bin/env python

import rospy
from performance_tests.msg import SuperAwesome  # Custom created string message
from std_msgs.msg import Int32  # Integer message for desired rate publication
from dynamic_reconfigure.server import Server as DynamicReconfigureServer  # dynamic reconfiguration server
from performance_tests.cfg import DynReconfConfig  # dynamic reconfiguration parameters


# Publisher Class
class Publisher():

    # Dynamic reconfiguration callback
    def callback(self, config, level):
        self.loop_rate = config["loop_rate"]
        return config

    def __init__(self):
        # Custom message publisher definition
        pub = rospy.Publisher('/SuperAwesomeTopic', SuperAwesome, queue_size=1000)
        # Real rate message publisher definition
        pub_ = rospy.Publisher('/DesiredRate', Int32, queue_size=1000)
        rospy.init_node('py_publisher', anonymous=True)
        # Dynamic reconfiguration definition
        self.server = DynamicReconfigureServer(DynReconfConfig, self.callback)

        while not rospy.is_shutdown():
            rate = rospy.Rate(self.loop_rate)  # Modifiable loop rate
            hello_str = "hello world"
            rospy.loginfo('DESIRED RATE %d' % (self.loop_rate))
            pub.publish(hello_str)  # Custom message publication
            pub_.publish(self.loop_rate)  # Real Rate message publication
            rate.sleep()

# Main
if __name__ == '__main__':
    try:
        pu = Publisher()  # Publisher object
    except rospy.ROSInterruptException:
        pass
