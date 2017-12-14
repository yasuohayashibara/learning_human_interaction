#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('learning_human_interaction')
import rospy
from learning_human_interaction.msg import Interaction_robot_status
from cv_bridge import CvBridge
from reinforcement_learning import *
from std_msgs.msg import Float32, Int8

class learning_human_interaction_node:
    def __init__(self):
        rospy.init_node('learning_human_interaction_node', anonymous=True)
        self.action_num = rospy.get_param("/learning_human_interaction_node/action_num", 3)
        print("action_num: " + str(self.action_num))
        self.rl = reinforcement_learning(n_action = self.action_num)
        self.bridge = CvBridge()
        self.interaction_robot_status_sub = rospy.Subscriber("/interaction_robot_status", Interaction_robot_status, self.callback)
        self.reward_sub = rospy.Subscriber("/reward", Float32, self.callback_reward)
        self.action_pub = rospy.Publisher("action", Int8, queue_size=1)
        self.action = 0
        self.reward = 0
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.count = 0
        self.learning = True
        self.interaction_robot_status = Interaction_robot_status()

    def callback(self, status):
        self.interaction_robot_status = status

    def callback_reward(self, reward):
        self.reward = reward.data
        obs = self.interaction_robot_status.angle_errors

        self.action = self.rl.act_and_trains(obs, self.reward)
        self.action_pub.publish(self.action)
        self.count += 1
        print("learning = " + str(self.learning) + " count: " + str(self.count) + " action: " + str(self.action) + ", reward: " + str(round(self.reward,5)))

if __name__ == '__main__':
    lhi = learning_human_interaction_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
