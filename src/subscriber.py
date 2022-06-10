#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import paho.mqtt.client as mqtt #import the client1
import time

z_pos_data = 0.0
z_process_value = 0.0

MAX_ZPOS = 55


def callbackstate(data):
    #rospy.loginfo("Process value is: %f", data.process_value)
    global z_process_value 
    z_process_value= data.process_value 

def callback(data):
    #rospy.loginfo("Data value is: %f", data.data)
    #mqtt.publish("/TopicData", data.data)
    global z_pos_data 
    z_pos_data= data.data
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('py_go_up', anonymous=True)

    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/command/", Float64, callback)
    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/state", JointControllerState, callbackstate)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

def go_up():
    pub = rospy.Publisher('/wheelchair/z_position_upper_chassis_controller/command/', Float64, queue_size=10)
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(35) # Hz -> SPEED
    while True:
        cmd = input("CMD: ")
        start_time = time.time()
        if cmd == 'u':
            for i in range(20, MAX_ZPOS):
                if z_process_value < 0.027496213:
                    msg = Float64()
                    #msg.data = float(input("Data: "))
                    msg.data = 0.001*i
                    pub.publish(msg)
                    rate.sleep()
                else:
                    break
        elif cmd == 'd':
            for i in range(MAX_ZPOS - 10, 1, -1):
                if z_process_value > 0:
                    msg = Float64()
                    #msg.data = float(input("Data: "))
                    msg.data = 0.001*i
                    pub.publish(msg)
                    rate.sleep()
                else:
                    break
        else:
            exit(1)
        # for i in range(45, 1, -1):
        #     msg = Float64()
        #     #msg.data = float(input("Data: "))
        #     msg.data = 0.01*i
        #     pub.publish(msg)
        #     rate.sleep()
        print("ZPOS: ", z_process_value)
        print("--- %s seconds ---" % (time.time() - start_time))
if __name__ == '__main__':

    mqtt = mqtt.Client("PC ROS GAZEBO")

    try:
        mqtt.connect("192.168.0.107", 1883)
    except:
        raise ValueError("Couldn't connect to MQTT broker.")

    listener()
  
    try:
        go_up()
    except rospy.ROSInterruptException:
        pass