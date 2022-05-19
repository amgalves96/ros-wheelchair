#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import paho.mqtt.client as mqtt #import the client1

mqtt = mqtt.Client("PC ROS GAZEBO")
#
try:
    mqtt.connect("192.168.0.107", 1883)
except:
    raise ValueError("Couldn't connect to MQTT broker.")

def callbackstate(data):
    rospy.loginfo("Process value is: %f", data.process_value)

def callback(data):
    rospy.loginfo("Data value is: %f", data.data)
    mqtt.publish("/TopicData", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/command/", Float64, callback)
    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/state", JointControllerState, callbackstate)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()