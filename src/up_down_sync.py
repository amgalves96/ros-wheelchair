#!/usr/bin/env python3.8

from http import client
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import paho.mqtt.client as mqtt
import time

z_pos_data = 0.0
z_process_value = 0.0

SPEED_MAX_ZPOS = 45
MAX_ZPOS = 0.04255
MAX_ZPOS_EV3 = -215
SPEED = 40 # Hz
ev3_z_pos = 0
last_decoded_msg = 0


def callback_state(data):
    #rospy.loginfo("Process value is: %f", data.process_value)
    global z_process_value 
    z_process_value= data.process_value 

def callback_data(data):
    print("Data value is: ", data.data)
    #mqtt.publish("/TopicData", data.data)
    global z_pos_data 
    z_pos_data= data.data
    
def subscribe_ros_topics():

    rospy.init_node('up_down_motor', anonymous=True)

    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/command/", Float64, callback_data)
    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/state", JointControllerState, callback_state)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()


def go_down():
    pub = rospy.Publisher('/wheelchair/z_position_upper_chassis_controller/command/', Float64, queue_size=5)
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(SPEED)

    #for i in range(int(z_pos_data*1000), -1, -1):
    msg = Float64()
    #msg.data = float(input("Data: "))
    #msg.data = 0.001*i
    if z_pos_data > 0:
        msg.data = 0.001*(z_pos_data*1000 - 1)
        print(msg.data)
        pub.publish(msg)
        rate.sleep()

def go_up():
    pub = rospy.Publisher('/wheelchair/z_position_upper_chassis_controller/command/', Float64, queue_size=5)
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(SPEED)
   
    #for i in range(int(z_pos_data*1000), SPEED_MAX_ZPOS + 1, 1):
    msg = Float64()
    #msg.data = float(input("Data: "))
    #msg.data = 0.001*i
    if z_pos_data < 0.045:
        msg.data = 0.001*(z_pos_data*1000 + 1)
        print(msg.data)
        #time.sleep(0.001)
        pub.publish(msg)
        rate.sleep()
    
        
    #print("ZPOS: ", z_process_value)
    #print("--- %s seconds ---" % (time.time() - start_time))

def stop():
    pub = rospy.Publisher('/wheelchair/z_position_upper_chassis_controller/command/', Float64, queue_size=5)
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(SPEED)
    msg = Float64()
    #msg.data = float(input("Data: "))
    msg.data = z_pos_data
    print(msg.data)
    #time.sleep(0.001)
    pub.publish(msg)
    rate.sleep()

def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    client.subscribe("/up_down_motor_pos")


def on_message(client, userdata, msg):
    #print(f"Message received [{msg.topic}]: {msg.payload}")
    #print(type(msg.payload))
    decoded_msg = str(msg.payload.decode("utf-8"))
    global last_decoded_msg
    if decoded_msg != last_decoded_msg:
        print(decoded_msg)
        last_decoded_msg = decoded_msg
    if decoded_msg == "up":
        try:
            go_up()
        except rospy.ROSInterruptException:
            pass
    elif decoded_msg == "down":
        try:
            go_down()
        except rospy.ROSInterruptException:
            pass
    # else:
    #     try:
    #         stop()
    #     except rospy.ROSInterruptException:
    #         pass


if __name__ == '__main__':

    client = mqtt.Client("PC ROS GAZEBO")
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect("192.168.0.107", 1883)
    except:
        raise ValueError("Couldn't connect to MQTT broker.")

    subscribe_ros_topics()
    
    client.loop_forever()
        

  