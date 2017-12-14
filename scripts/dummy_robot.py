#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('learning_human_interaction')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from learning_human_interaction.msg import Interaction_robot_status
from std_msgs.msg import Float32, Int8
import numpy as np
import math
import read_key

class Environment:
    def __init__(self):
        self.L = 0.3
        self.target_distance = 0.45
        target_angle = math.acos(self.target_distance/(2.0*self.L))
        self.robot_x = 0
        self.human_x = 0.4
        current_angle = math.acos((self.human_x-self.robot_x)/(2.0*self.L))
        self.max_error_x = 0.1
        self.angle_errors = np.array([0, 0, 0])
        self.target_angles = np.array([-target_angle, 2.0*target_angle, -target_angle])
        self.current_angles = np.array([-current_angle, 2.0*current_angle, -current_angle])
        self.WIDTH = 640
        self.HEIGHT = 480
        self.cv_image = np.zeros((self.HEIGHT, self.WIDTH, 3), np.uint8)

    def calculate(self, move_x):
        self.human_x += move_x
        distance = self.human_x - self.robot_x
        error = self.target_distance - distance
        error = min([max([error, -self.max_error_x]), self.max_error_x])
        distance = self.target_distance - error
        self.human_x = self.robot_x + distance
        target_angle = math.acos(self.target_distance / (2.0 * self.L))
        self.target_angles = np.array([-target_angle, 2.0 * target_angle, -target_angle])
        current_angle = math.acos(distance / (2.0 * self.L))
        self.current_angles = np.array([-current_angle, 2.0 * current_angle, -current_angle])
        self.angle_errors = self.target_angles - self.current_angles

    def draw(self):
        CX = self.WIDTH / 2
        CY = self.HEIGHT - 100
        R = 300

        self.cv_image.fill(255)
        cv2.line(self.cv_image, (0, CY), (self.WIDTH, CY), (0, 0, 0), 3)
        RX = self.robot_x * R + CX
        cv2.rectangle(self.cv_image, (RX, CY), (RX - 100, CY - 200), (255, 0, 0), 3)
        HX = int(self.human_x * R + CX)
        cv2.line(self.cv_image, (HX, CY), (HX, CY - 200), (0, 0, 0), 3)
        AY = CY - 150
        KX = int((self.human_x + self.robot_x) / 2.0 * R + CX)
        KY = int(math.sqrt(self.L ** 2 - ((self.human_x - self.robot_x) / 2.0) ** 2) * R + AY)
        cv2.line(self.cv_image, (RX, AY), (KX, KY), (0, 0, 0), 2)
        cv2.line(self.cv_image, (KX, KY), (HX, AY), (0, 0, 0), 2)
        TX = int((self.robot_x + self.target_distance / 2.0) * R + CX)
        TY = int(math.sqrt(self.L ** 2 - (self.target_distance / 2.0) ** 2) * R + AY)
        THX = int((self.robot_x + self.target_distance) * R + CX)
        cv2.line(self.cv_image, (RX, AY), (TX, TY), (0, 0, 255), 2)
        cv2.line(self.cv_image, (TX, TY), (THX, AY), (0, 0, 255), 2)
        cv2.imshow("robot", self.cv_image)
        cv2.waitKey(1)

class dummy_robot:
    def __init__(self):
        rospy.init_node('dummy_robot', anonymous=True)
        self.bridge = CvBridge()
        self.interaction_robot_status_pub = rospy.Publisher("interaction_robot_status", Interaction_robot_status, queue_size=1)
        self.reward_pub = rospy.Publisher("reward", Float32, queue_size=1)
        self.action_sub = rospy.Subscriber("action", Int8, self.callback_action)
        self.action = 0
        self.pan = 0
        self.fb = 100
        self.reward = 0

        self.interaction_robot_status = Interaction_robot_status()
        self.interaction_robot_status.angle_errors = [0, 0, 0]
        self.interaction_robot_status.target_angles = [0, 0, 0]

        self.env = Environment()

        self.publish_status_timer = rospy.Timer(rospy.Duration(0.033), self.callback_publish_status_timer)
        self.reward_timer = rospy.Timer(rospy.Duration(0.2), self.callback_reward_timer)
        self.count = 0
        self.prev_count = -1

    def callback_publish_status_timer(self, data):
        c = ''
        if read_key.key_pressed():
            c = read_key.read_key()
        if c == 'x':
            move_x = 0.01
        elif c == 'z':
            move_x = -0.01
        else:
            move_x = 0
        self.env.calculate(move_x)
        self.env.draw()
        self.interaction_robot_status_pub.publish(self.interaction_robot_status)

    def callback_action(self, data):
        action_list = [[0, 0], [-10, -2], [10, -2], [0, -4]]
        self.action = data.data
        if (self.action < 0 or self.action >= 4):
            return
        self.pan += action_list[self.action][0]
        self.fb += action_list[self.action][1]
        self.count += 1
        if ((self.count % 200) == 0):
            self.pan = int(np.random.rand() * 400 - 200)
            self.fb = int(np.random.rand() * 100) + 100
            print("change pan angle")

    def callback_reward_timer(self, data):
        if (self.prev_count == self.count):
            print("reward timer is too first!")
        self.prev_count = self.count
        reward_pan = min(1.0 - abs(self.pan) / 100.0, 1.0) ** 3
        reward_fb = min(1.0 - abs(self.fb - 100.0) / 30.0, 1.0) ** 3
        self.reward = reward_pan + reward_fb;
#        print("selected_action: " + str(self.action) + ", reward: " + str(self.reward))
        self.reward_pub.publish(self.reward)

if __name__ == '__main__':
    dr = dummy_robot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()
