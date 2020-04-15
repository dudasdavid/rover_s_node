#! /usr/bin/env python3
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

class ControllerNode:

    def __init__(self):
        self.velocity = 0.0
        self.angularVelocity = 0.0
        self.steeringAngle = 0.0
        self.distance = 50.0
        self.driver_control = Int16MultiArray()

    def main(self):                                        
        self.wheelPub = rospy.Publisher('driver_control',
                                        Int16MultiArray, queue_size=1)
                                        
        rospy.init_node('rover_drive_controller')
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

        self.ticksPerMeter = float(rospy.get_param('~ticks_per_meter'))
        self.wheelSeparation = float(rospy.get_param('~wheel_separation'))
        self.wheelSeparationLength = float(rospy.get_param('~wheel_separation_length'))
        self.maxMotorSpeed = int(rospy.get_param('~max_motor_speed'))
        self.rate = float(rospy.get_param('~rate', 10.0))
        self.timeout = float(rospy.get_param('~timeout', 0.2))

        rate = rospy.Rate(self.rate)
        self.lastTwistTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        if rospy.get_time() - self.lastTwistTime < self.timeout:
            
            self.steeringAngle = self.angularVelocity * 100

            self.driver_control.data = [int(self.velocity*120), int(self.steeringAngle), int(self.distance)]
            self.wheelPub.publish(self.driver_control)
        else:
            self.driver_control.data = [0, 0, 0]
            self.wheelPub.publish(self.driver_control)

    def twistCallback(self, twist):
        self.velocity = twist.linear.x
        self.angularVelocity = twist.angular.z
        # print("%d,%d" % (self.linearVelocity, self.angularVelocity))
        self.lastTwistTime = rospy.get_time()




if __name__ == '__main__':
    try:
        node = ControllerNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
        