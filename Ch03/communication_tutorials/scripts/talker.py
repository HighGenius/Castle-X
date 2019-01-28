#!/usr/bin/env python
#coding:utf-8
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)   #定义发布的主题名称chatter， 消息类型String,实质是std_msgs.msg.String， 设置队列条目个数
    rospy.init_node('talker', anonymous=True)   #初始化节点，节点名称为talker,anonymous=True，要求每个节点都有唯一的名称，避免冲突。这样可以运行多个talker.py
    rate = rospy.Rate(0.5) # 发布频率为10hz
    while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)  #在屏幕输出日志信息，写入到rosout节点，也可以通过rqt_console来查看
        pub.publish(hello_str)      #发布信息到主题
        rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass