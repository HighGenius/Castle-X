#!/usr/bin/env python
#coding:utf-8

import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
	# 初始化节点
    rospy.init_node('turtle_tf_listener')

    # 创建一个tf.TransformListener对象，用于接收TF转换并缓冲10秒（TF包提供了一个tf.transformlistener实现使接收任务将更容易）
    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
        	# 通过listener.lookupTransform函数来实现获取变换
        	# 第一参数：变换从/turtle2坐标系，第二参数：到/turtle1坐标系
        	# rospy.Time(0)，想变换的时间，rospy.Time(0)获取最新的变换
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # 设置参数来控制跟随小海龟（turtle2）的运动
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()







