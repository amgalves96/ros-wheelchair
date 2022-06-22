#!/usr/bin/env python3.8

from http import client
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import paho.mqtt.client as mqtt
import time

cmd_data = 0.015
process_value = 0.0

SPEED_MAX_ZPOS = 55
MAX_POS = 0.8
MIN_POS = -0.5773
MAX_POS_EV3 = -95
SPEED = 75 # Hz
ev3_z_pos = 0


def callback_state(data):
    #rospy.loginfo("Process value is: %f", data.process_value)
    global process_value 
    process_value= data.process_value 

def callback_data(data):
    print("Data value is: ", data.data)
    #mqtt.publish("/TopicData", data.data)
    global cmd_data 
    cmd_data= data.data
    
def subscribe_ros_topics():

    rospy.init_node('back_motor', anonymous=True)

    rospy.Subscriber("/wheelchair/back_controller/command/", Float64, callback_data)
    rospy.Subscriber("/wheelchair/back_controller/state/", JointControllerState, callback_state)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()


def go_backwards(ev3_pos):
    pub = rospy.Publisher('/wheelchair/back_controller/command/', Float64, queue_size=5)
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(SPEED)

    # VER QUAIS OS LIMITES DO MOTOR DAS COSTAS E MUDAR
    desired_pos = ev3_pos*MAX_ZPOS/MAX_ZPOS_EV3

    for i in range(int(z_pos_data*1000), 1, -1):
        if z_process_value > desired_pos:
            msg = Float64()
            #msg.data = float(input("Data: "))
            msg.data = 0.001*i
            pub.publish(msg)
            rate.sleep()
        else:
            break

def go_forward(ev3_pos):
    pub = rospy.Publisher('/wheelchair/z_position_upper_chassis_controller/command/', Float64, queue_size=5)
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(SPEED)

    desired_pos = ev3_pos*MAX_ZPOS/MAX_ZPOS_EV3
   
    for i in range(int(z_pos_data*1000), SPEED_MAX_ZPOS, 1):
        if z_process_value < desired_pos:
            msg = Float64()
            #msg.data = float(input("Data: "))
            msg.data = 0.001*i
            print(msg.data)
            #time.sleep(0.001)
            pub.publish(msg)
            rate.sleep()
        else:
            break
        
    #print("ZPOS: ", z_process_value)
    #print("--- %s seconds ---" % (time.time() - start_time))

def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    client.subscribe("/up_down_motor_pos")


def on_message(client, userdata, msg):
    #print(f"Message received [{msg.topic}]: {msg.payload}")
    #print(type(msg.payload))
    decoded_msg = int(msg.payload.decode("utf-8"))
    print(decoded_msg)
    global ev3_z_pos
    if decoded_msg < ev3_z_pos:
        ev3_z_pos = decoded_msg
        try:
            go_up(decoded_msg)
        except rospy.ROSInterruptException:
            pass
    elif decoded_msg > ev3_z_pos:
        ev3_z_pos = decoded_msg
        try:
            go_down(decoded_msg)
        except rospy.ROSInterruptException:
            pass


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
        

  