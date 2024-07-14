'''
Created on Mar 6, 2024
@authors: Ruben Odamo, Tanaya Patel, Lauryn Williams-Lewis, Matthew Pryke
'''


#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time


import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

from datetime import datetime

from pypdf import PdfWriter
import os
from PIL import Image as Img

from geometry_msgs.msg import PoseWithCovarianceStamped
import math



class move_bot:
    '''
    This class contains all the functionality related to moving the turtlebot based on the MQTT data that is being
    received from the subscribers.
    '''
    global mqttc
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    global publisher
    publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    global image
    image = ""
    global output_image1
    output_image1 = ""
    global output_image2
    output_image2 = ""
    global sum_mask
    sum_mask = 0
    global goal
    goal = ""

    global reset, bold, italics, underline
    global blackhighlight, redhighlight, greyhighlight, whitehighlight, grey, yellow, pink
    reset = "\033[0;0m"
    bold = "\033[1m"
    italics = "\x1B[3m"
    underline = "\033[4m"

    blackhighlight = "\033[40m"
    whitehighlight = "\033[1;31;47m"
    redhighlight = "\033[41m"
    greyhighlight = "\033[47m"
    grey = "\033[30m"
    yellow = "\033[93m"
    pink = "\033[35m"


    # initialiser functions
    def __init__(self):
        '''
        Initializes the initial states for the number of people coming into the great hall and the silent alarm.
        Sets the coordinates for the control room, great hall, the vault and where to take the images from when in the
        vault.
        creates the MQTT subscriber and rospy node.
        '''
        self.entry_count = 0

        self.silent_alarm = False

        # setting constants for coordinates
        self.CONTROL_ROOM = "\033[1;92mCONTROL ROOM\033[0;0m"
        self.CONTROL_ROOM_X = 3.988697052001953
        self.CONTROL_ROOM_Y = -2.304656982421875
        self.CONTROL_ROOM_Z = -0.7473182429830794
        self.CONTROL_ROOM_W = 0.6644662848517471

        self.GREAT_HALL = "\033[1;36mGREAT HALL\033[0;0m"
        self.GREAT_HALL_X = 3.1317687034606934
        self.GREAT_HALL_Y = -3.286555528640747
        self.GREAT_HALL_Z = -0.09662138479570102
        self.GREAT_HALL_W = 0.9953212084549193

        self.VAULT = "\033[1;91mVAULT\033[0;0m"
        self.VAULT_X = 2.230473756790161
        self.VAULT_Y = -1.984551191329956
        self.VAULT_Z = -0.7398005836305999
        self.VAULT_W = 0.6728262007828053

        self.VAULT_IMG1_X = 2.1900742053985596
        self.VAULT_IMG1_Y = -2.835242986679077
        self.VAULT_IMG1_Z = 0.6186573435734185
        self.VAULT_IMG1_W = 0.7856609263815283

        self.VAULT_IMG2_X = 2.230473756790161
        self.VAULT_IMG2_Y = -1.984551191329956
        self.VAULT_IMG2_Z = -0.23692447726818996
        self.VAULT_IMG2_W = 0.9715280706552925

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_capture)
        self.image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        self.current_position = None

        rospy.init_node('mqttListener', anonymous=False)
        rate = rospy.Rate(10) 

        mqttc.on_connect = self.on_connect
        mqttc.on_message = self.on_message
        mqttc.on_subscribe = self.on_subscribe
        mqttc.on_unsubscribe = self.on_unsubscribe
        mqttc.username_pw_set("openhabian", "openhabian")

        mqttc.user_data_set([])
        mqttc.connect("10.148.189.65")

        mqttc.loop_start()

        while not rospy.is_shutdown(): 
            rospy.spin()

    # publishes moves for room parameters
    def movement_publisher(self, room, x, y, z, w):
        '''
        Publishes the coordinates of the turtlebot so that it can move towards the coordinates given
        :param room: the room to move towards
        :param x: x coordinate
        :param y: y coordinate
        :param z: z coordinate
        :param w: w coordinate
        '''
        rate = rospy.Rate(10)
        
        global goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z
        goal.pose.orientation.w = w
        
        # Only publish the goal once (no loop, can be overwritten)
        publisher.publish(goal) 
        print('{0}Thumbscrew --> '.format(bold)+room)
        time.sleep(5)	


    def pose_callback(self, msg):
        # Callback function to handle pose messages
        self.current_position = msg.pose.pose

        # # Callback function to handle pose messages
        # position = msg.pose.pose.position
        # orientation = msg.pose.pose.orientation
        # rospy.loginfo("Current Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
        # rospy.loginfo("Current Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))

    def image_capture(self, msg):
        '''
        Creates an image of what is currently being seen by the turtlebot.
        :param msg: feed from the camera on the turtle bot
        '''
        global image
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def image_callback(self, msg):
        '''
        Detects the charging station (red paper) from the image on the camera by filtering out all the colours such that
        red is the only colour being shown.
        :param msg: feed fromt he camera on the turtle bot
        '''
        lightest1 = [140, 60, 0]
        darkest1 = [179, 255, 255]
        lightest2 = [0, 60, 0]
        darkest2 = [5, 255, 255]
		# image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        cv2.namedWindow("original", 1)
        cv2.imshow("original", image)

   	
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lightest1 = np.array(lightest1, dtype = "uint8")
        darkest1 = np.array(darkest1, dtype = "uint8")
        lightest2 = np.array(lightest2, dtype = "uint8")
        darkest2 = np.array(darkest2, dtype = "uint8")

        mask1 = cv2.inRange(hsv, lightest1, darkest1)
        global output_image1
        output_image1 = cv2.bitwise_and(image, image, mask = mask1)

        mask2 = cv2.inRange(hsv, lightest2, darkest2)
        global output_image2
        output_image2 = cv2.bitwise_and(image, image, mask = mask2)

        global sum_mask
        sum_mask = np.sum(mask1)+np.sum(mask2)

        # cv2.imshow("thresholded_1", output_image1)
        # cv2.imshow("thresholded_2", output_image2)

        cv2.waitKey(100)

    def on_publish(client, userdata, mid, reason_code, properties):
        '''
        Checks if the publisher has connected
        :param userdata:
        :param mid:
        :param reason_code:
        :param properties:
        '''
        # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
        try:
            userdata.remove(mid)
        except KeyError:
            print("on_publish() is called with a mid not present in unacked_publish")

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        '''
        Checks if the subscriber has connected
        :param client:
        :param userdata:
        :param mid:
        :param reason_code_list:
        :param properties:
        '''
        # Since we subscribed only for a single channel, reason_code_list contains
        # a single entry
        if reason_code_list[0].is_failure:
            print(f"Broker rejected you subscription: {reason_code_list[0]}")
        else:
            print(f"Broker granted the following QoS: {reason_code_list[0].value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        '''
        Checks if the subscriber has successfully disconnected
        :param client:
        :param userdata:
        :param mid:
        :param reason_code_list:
        :param properties:
        '''
        # Be careful, the reason_code_list is oonly present in MQTTv5.
        # In MQTTv3 it will always be empty
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            print("uImagensubscribe succeed\033[40med (if SUBACK is received in MQTTv3 it success)")
        else:
            print(f"Broker replied with failure: {reason_code_list[0]}")
        client.disconnect()

    def on_message(self, client, userdata, message):
        '''
        This tells all the connected devices what to do when something happens.
        This is essentially the rules from openhab.
        :param client:
        :param userdata:
        :param message:
        '''
        # userdata is the structure we choose to provide, here it's a list()
        userdata.append(message.payload)
        print(blackhighlight+"SENSOR STATUS: "+italics+f"{message.topic} {message.payload}"+reset)   
        
        #if button pressed x1 -> control room
        if('Team_3_Button_Scene_Number' in message.topic and message.payload == b'1.0'):
            self.movement_publisher(self.CONTROL_ROOM, self.CONTROL_ROOM_X, self.CONTROL_ROOM_Y, self.CONTROL_ROOM_Z, self.CONTROL_ROOM_W)
        #if button pressed x2 -> great hall
        if('Team_3_Button_Scene_Number' in message.topic and message.payload == b'1.3'):
            self.movement_publisher(self.GREAT_HALL, self.GREAT_HALL_X, self.GREAT_HALL_Y, self.GREAT_HALL_Z, self.GREAT_HALL_W)
        #if button pressed x3 -> vault
        if('Team_3_Button_Scene_Number' in message.topic and message.payload == b'1.4'):
            self.movement_publisher(self.VAULT, self.VAULT_X, self.VAULT_Y, self.VAULT_Z, self.VAULT_W)
        # If button is press on HOLD, turn plug (silent alarm) off
        if('Team_3_Button_Scene_Number' in message.topic and message.payload == b'1.2' or 'Team_3_Button_Scene_Number' in message.topic and message.payload == b'3823'):
            if(self.silent_alarm):
                self.silent_alarm = False
                client.publish('team3eventbus/in/Team_3_Fibaro_Wall_Plug_Switch_Binary/command', b'OFF')
                print('{0}~SILENT ALARM DISABLED~{1}'.format(whitehighlight, reset))
        if('Team_3_Button_Scene_Number' in message.topic and message.payload == b'1.5'):
            # Go to the nearest charging station i.e. find red
            print('{0}Thumbscrew --> {1}Nearest Charging Station{2}'.format(bold, pink, reset))
            print("{0}Finding Charging Station...{1}".format(italics, reset))
            # While not at charging station i.e. 50million pixels of colour
            while(sum_mask < 50000000):
                if(sum_mask > 5000000):
                    # Colour detected - move forward
                    twist = Twist()
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
                    pub.publish(twist)
                    time.sleep(3)
                else:
                    # Can't find colour - twist
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 1.0
                    pub.publish(twist)
                    time.sleep(3)
            print("{0}...Charging Station Found{1}".format(italics, reset))


        # if tamper alarm triggers, move to vault and take picture of movement
        if('Team_3_Motion_Sensor_Tamper_Alarm' in message.topic and message.payload == b'ON'):# or 'Team_3_Motion_Sensor_Motion_Alarm' in message.topic and message.payload == b'ON'):
            # turn the silent alarm on
            self.silent_alarm = True
            client.publish('team3eventbus/in/Team_3_Fibaro_Wall_Plug_Switch_Binary/command', b'ON')
            print('{0}{1}~SILENT ALARM TRIGGERED~{2}'.format(bold, redhighlight, reset))

            pdf_writer = PdfWriter()
            pdf_writer.append("./src/cps_robot_code/security_message.pdf")
            
            # Move to the vault
            self.movement_publisher(self.VAULT, self.VAULT_IMG1_X, self.VAULT_IMG1_Y, self.VAULT_IMG1_Z, self.VAULT_IMG1_W)
            # Ensure we have reached the first goal before proceeding
            while(not compare_poses(self.current_position, goal.pose)):
                time.sleep(1)

            #take a snapshot
            time.sleep(3)
            global image
            imageTime = datetime.now().strftime('%d-%m-%Y-%H:%M:%S')
            imageName = f"./src/cps_robot_code/images/vault1-{imageTime}.jpg"
            cv2.imwrite(imageName,image)
            
            # Converts image to pdf
            opened = Img.open(imageName)
            opened.convert("RGB").save("1.pdf")
            pdf_writer.append("1.pdf")
            

            # Move to the vault
            self.movement_publisher(self.VAULT, self.VAULT_IMG2_X, self.VAULT_IMG2_Y, self.VAULT_IMG2_Z, self.VAULT_IMG2_W)
            # Ensure we have reached the second goal before proceeding
            while(not compare_poses(self.current_position, goal.pose)):
                time.sleep(1)
            # Take a snapshot
            time.sleep(3)
            imageTime = datetime.now().strftime('%d-%m-%Y-%H:%M:%S')
            imageName = "./src/cps_robot_code/images/vault2-"+imageTime+".jpg"
            cv2.imwrite(imageName,image)

            # Converts image to pdf
            opened = Img.open(imageName)
            opened.convert("RGB").save("2.pdf")
            pdf_writer.append("2.pdf")

            pdf_writer.write(f"./src/cps_robot_code/Security_Threat_{imageTime}.pdf")
            
            os.remove("1.pdf")
            os.remove("2.pdf")
            
            


        #if the door sensor OPENS, increment a counter for entry count
        if('Team_3_Door_Sensor_Sensor_Door' in message.topic and message.payload == b'OPEN'):
            self.entry_count = self.entry_count + 1
            print(yellow+"Entry Count: ",self.entry_count,reset)     
              

    def on_connect(self, client, userdata, flags, reason_code, properties):
        '''
        Error checking for connection to the subscriber
        :param client:
        :param userdata:
        :param flags:
        :param reason_code:
        :param properties:
        :return:
        '''
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            # we should always subscribe from on_connect callback to be sure
            # our subscribed is persisted across reconnections.
            client.subscribe("#")

def compare_poses(current_pose, goal_pose, threshold=0.1):
    """
        Function: compare_poses
        :return: boolean, to state whether current and goal are within 0.1 blocks of each other
    """
    # Calculate the Euclidean distance between positions
    distance = math.sqrt((current_pose.position.x - goal_pose.position.x)**2 +
                         (current_pose.position.y - goal_pose.position.y)**2 +
                         (current_pose.orientation.z - goal_pose.orientation.z)**2 +
                         (current_pose.orientation.w - goal_pose.orientation.w)**2)
    # Check if the distance is less than the threshold
    return distance < threshold



if __name__ == '__main__':
    try:
        mover = move_bot()
    except rospy.ROSInterruptException:
        mqttc.loop_stop()
        pass