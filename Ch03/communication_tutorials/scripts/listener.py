#!/usr/bin/env python
#coding:utf-8
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)  #打印订阅之后的信息
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True) #初始化节点，节点名称为talker,anonymous=True，要求每个节点都有唯一的名称，避免冲突。这样可以运行多个listener.py

    rospy.Subscriber("chatter", String, callback) #订阅chatter话题,当订阅成功之后执行回调函数

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()        #保持节点运行，直到节点关闭

if __name__ == '__main__':
    listener()