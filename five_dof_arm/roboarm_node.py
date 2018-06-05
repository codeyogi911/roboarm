#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math
# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
#pwm = Adafruit_PCA9685.PCA9685()
# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096
# Set frequency to 60hz, good for servos.
#pwm.set_pwm_freq(60)
def callback(msg):
#    (msg.position[o] * 180)/math.pi
#    pwm.set_pwm(0, 0, ((msg.position[o] * 180)/math.pi)*2.5)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", (msg.position[0] * 180)/math.pi)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('roboarm', anonymous=False)

    rospy.Subscriber("/joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
