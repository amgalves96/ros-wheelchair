#!/usr/bin/env python3.8

from http import client
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt

# Initialize variables
up_down_pos_data = 0.0
up_down_process_value = 0.0
back_data = 0.0
back_process_value = 0.0

BACK_MAX_POS = 0.8
BACK_MIN_POS = -0.5773
#MAX_ZPOS = 0.04255
#MAX_ZPOS_EV3 = -215
UP_DOWN_MAX_POS = 0.045
UP_DOWN_MIN_POS = 0
#MAX_ZPOS = 0.04255
#MAX_ZPOS_EV3 = -215
BACK_SPEED = 100 # Hz
UP_DOWN_SPEED = 40 # Hz
#ev3_z_pos = 0

# Initialize MQTT variable messages
last_decoded_msg_top_down = ''
last_decoded_msg_back = ''
last_decoded_msg_steer = ''
cmd_top_down = ''
cmd_back = ''
cmd_steer = ''


def callback_up_down_state(data):
    #rospy.loginfo("Process value is: %f", data.process_value)
    global up_down_process_value 
    up_down_process_value= data.process_value 


def callback_up_down_data(data):
    print("Data value is: ", data.data)
    #mqtt.publish("/TopicData", data.data)
    global up_down_pos_data 
    up_down_pos_data= data.data


def callback_back_state(data):
    #rospy.loginfo("Process value is: %f", data.process_value)
    global back_process_value 
    back_process_value= data.process_value 


def callback_back_data(data):
    print("Data value is: ", data.data)
    #mqtt.publish("/TopicData", data.data)
    global back_data 
    back_data= data.data


def go_down(pub):
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(UP_DOWN_SPEED)

    for i in range(int(up_down_pos_data*1000), UP_DOWN_MIN_POS - 1, -1):
        if cmd_top_down == "s_ud":
            break
        msg = Float64()
    #msg.data = float(input("Data: "))
        msg.data = 0.001*i
        #if up_down_pos_data > UP_DOWN_MIN_POS:
            #msg.data = up_down_pos_data - 0.001
        print(msg.data)
        pub.publish(msg)
        rate.sleep()


def go_up(pub):
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(UP_DOWN_SPEED)
   
    for i in range(int(up_down_pos_data*1000), int((UP_DOWN_MAX_POS*1000)) + 1, 1):
        if cmd_top_down == "s_ud":
            break
        msg = Float64()
    #msg.data = float(input("Data: "))
        msg.data = 0.001*i
    #if up_down_pos_data < UP_DOWN_MAX_POS:
        #msg.data = up_down_pos_data + 0.001
        print(msg.data)
        #time.sleep(0.001)
        pub.publish(msg)
        rate.sleep()
    
        
    #print("ZPOS: ", z_process_value)
    #print("--- %s seconds ---" % (time.time() - start_time))


def forward(pub):
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(BACK_SPEED)
   
    for i in range(int(back_data*100), int(BACK_MAX_POS*100) + 1, 1):
        if cmd_back == "s_b":
            break
        msg = Float64()
    #msg.data = float(input("Data: "))
        msg.data = 0.01*i
    #if back_data < BACK_MAX_POS:
        #msg.data = back_data + 0.035
        print(msg.data)
        #time.sleep(0.001)
        pub.publish(msg)
        rate.sleep()


def backwards(pub):
    #rospy.init_node('py_go_up', anonymous=True)
    rate = rospy.Rate(BACK_SPEED)

    for i in range(int(back_data*100), int(100*BACK_MIN_POS), -1):
        if cmd_back == "s_b":
            break
        msg = Float64()
    #msg.data = float(input("Data: "))
        msg.data = 0.01*i
    #if back_data > BACK_MIN_POS:
        #msg.data = back_data - 0.035
        print(msg.data)
        pub.publish(msg)
        rate.sleep()


def steer_front(pub, velocity_msg: Twist, right_wheel_speed, left_wheel_speed):
    if right_wheel_speed == 0 and left_wheel_speed == 0:
        speed = 0.3
    else:
        speed = 0.5
    velocity_msg.linear.x = speed
    pub.publish(velocity_msg)
    

def steer_back(pub, velocity_msg: Twist, right_wheel_speed, left_wheel_speed):
    if right_wheel_speed == 0 and left_wheel_speed == 0:
        speed = -0.3
    else:
        speed = -0.5
    velocity_msg.linear.x = speed
    pub.publish(velocity_msg)


def steer_left(pub, velocity_msg: Twist, right_wheel_speed, left_wheel_speed):
    if right_wheel_speed == 0 and left_wheel_speed == 0:
        speed = -0.3
    else:
        speed = 3
    velocity_msg.angular.z = speed
    pub.publish(velocity_msg)


def steer_right(pub, velocity_msg: Twist, right_wheel_speed, left_wheel_speed):
    if right_wheel_speed == 0 and left_wheel_speed == 0:
        speed = -0.3
    else:
        speed = -3
    velocity_msg.angular.z = speed
    pub.publish(velocity_msg)


def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    client.subscribe("/up_down_motor_pos")
    client.subscribe("/back_motor_pos")
    client.subscribe("/steering")


def on_disconnect(client, userdata,rc=0):
    print("DisConnected result code "+str(rc))
    client.loop_stop()


def on_message(client, userdata, msg):
    decoded_msg = str(msg.payload.decode("utf-8"))
    global last_decoded_msg_top_down
    global last_decoded_msg_back
    global last_decoded_msg_steer
    
    if str(msg.topic) == "/up_down_motor_pos":
        global cmd_top_down 
        cmd_top_down = decoded_msg
        if decoded_msg != last_decoded_msg_top_down:
            print(decoded_msg)
        last_decoded_msg_top_down = decoded_msg
    
    elif str(msg.topic) == "/back_motor_pos":
        global cmd_back 
        cmd_back = decoded_msg
        if decoded_msg != last_decoded_msg_back:
            print(decoded_msg)
        last_decoded_msg_back = decoded_msg

    
    elif str(msg.topic) == "/steering":
        global cmd_steer
        cmd_steer = decoded_msg
        if decoded_msg != last_decoded_msg_steer:
            print(decoded_msg)
        last_decoded_msg_steer = decoded_msg


if __name__ == '__main__':

    # Connect to MQTT broker
    client = mqtt.Client("PC ROS GAZEBO")
    client.loop_start()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect("192.168.0.107", 1883)
    except:
        raise ValueError("Couldn't connect to MQTT broker.")

    # Initiate ROS node on this script
    rospy.init_node('Python_ROS', anonymous=True)

    # Subscribe ROS topics and define callbacks for each one
    rospy.Subscriber("/wheelchair/back_controller/command/", Float64, callback_back_data)
    rospy.Subscriber("/wheelchair/back_controller/state/", JointControllerState, callback_back_state)
    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/command/", Float64, callback_up_down_data)
    rospy.Subscriber("/wheelchair/z_position_upper_chassis_controller/state", JointControllerState, callback_up_down_state)
    #rospy.Subscriber("/wheelchair/diff_drive_controller/cmd_vel/", Twist, callback_up_down_data)

    # Define publishers for each topic
    pub_drive = rospy.Publisher('/wheelchair/diff_drive_controller/cmd_vel/', Twist, queue_size=1)
    pub_up_down = rospy.Publisher('/wheelchair/z_position_upper_chassis_controller/command/', Float64, queue_size=1)
    pub_back = rospy.Publisher('/wheelchair/back_controller/command/', Float64, queue_size=1)

    # Initialize angular and linear velocity 
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while True:
        if cmd_top_down == "up":
            try:
                go_up(pub_up_down)
            except rospy.ROSInterruptException:
                pass
        elif cmd_top_down == "down":
            try:
                go_down(pub_up_down)
            except rospy.ROSInterruptException:
                pass
        if cmd_back == "forward":
            try:
                forward(pub_back)
            except rospy.ROSInterruptException:
                pass
        elif cmd_back == "back":
            try:
                backwards(pub_back)
            except rospy.ROSInterruptException:
                pass
        if cmd_steer[:11] == "steer_front":
            right_wheel_speed = int(cmd_steer[11:cmd_steer.index('.')])
            left_wheel_speed = int(cmd_steer[cmd_steer.index('.') + 1:len(cmd_steer) + 1])
            print("Right Wheel Speed:", right_wheel_speed)
            print("Left Wheel Speed:", left_wheel_speed)
            try:
                steer_front(pub_drive, vel_msg, right_wheel_speed, left_wheel_speed)
            except rospy.ROSInterruptException:
                pass
        elif cmd_steer[:10] == "steer_back":
            right_wheel_speed = int(cmd_steer[10:cmd_steer.index('.')])
            left_wheel_speed = int(cmd_steer[cmd_steer.index('.') + 1:len(cmd_steer) + 1])
            print("Right Wheel Speed:", right_wheel_speed)
            print("Left Wheel Speed:", left_wheel_speed)
            try:
                steer_back(pub_drive, vel_msg, right_wheel_speed, left_wheel_speed)
            except rospy.ROSInterruptException:
                pass
        elif cmd_steer[:10] == "steer_left":
            right_wheel_speed = int(cmd_steer[10:cmd_steer.index('.')])
            left_wheel_speed = int(cmd_steer[cmd_steer.index('.') + 1:len(cmd_steer) + 1])
            print("Right Wheel Speed:", right_wheel_speed)
            print("Left Wheel Speed:", left_wheel_speed)
            try:
                steer_left(pub_drive, vel_msg, right_wheel_speed, left_wheel_speed)
            except rospy.ROSInterruptException:
                pass
        elif cmd_steer[:11] == "steer_right":
            right_wheel_speed = int(cmd_steer[11:cmd_steer.index('.')])
            left_wheel_speed = int(cmd_steer[cmd_steer.index('.') + 1:len(cmd_steer) + 1])
            print("Right Wheel Speed:", right_wheel_speed)
            print("Left Wheel Speed:", left_wheel_speed)
            try:
                steer_right(pub_drive, vel_msg, right_wheel_speed, left_wheel_speed)
            except rospy.ROSInterruptException:
                pass
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub_drive.publish(vel_msg)

        

  