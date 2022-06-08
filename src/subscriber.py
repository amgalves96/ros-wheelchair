#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import paho.mqtt.client as mqtt #import the client1

mqtt = mqtt.Client("PC ROS GAZEBO")

z_pos_data = 0.0
z_process_value = 0.0

try:
    mqtt.connect("192.168.0.107", 1883)
except:
    raise ValueError("Couldn't connect to MQTT broker.")

def callbackstate(data):
    #rospy.loginfo("Process value is: %f", data.process_value)
    global z_process_value 
    z_process_value= data.process_value 
    pass

def callback(data):
    #rospy.loginfo("Data value is: %f", data.data)
    #mqtt.publish("/TopicData", data.data)
    global z_pos_data 
    z_pos_data= data.data
    pass
    
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

def go_up():
    pub = rospy.Publisher('/wheelchair/z_position_upper_chassis_controller/command/', Float64, queue_size=10)
    rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = float(input("Data: "))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    #listener()
  
    try:
        go_up()
    except rospy.ROSInterruptException:
        pass