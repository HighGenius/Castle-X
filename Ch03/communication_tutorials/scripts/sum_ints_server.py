#!/usr/bin/env python
#coding:utf-8
 
from communication_tutorials.srv import *
import rospy

#定义一个累加函数
def handle_sum_ints(req):
	if req.a < req.i:
		x = req.i
		y = req.a
		while x > y:
			req.a = req.a + x
			x = x - 1
		print "sum: %s\n" % req.a
		return req.a
	else:
		print "The number is wrong,please enter again!\n"

#定义ROS下的一个服务端
def sum_ints_server():
    rospy.init_node('sum_ints_server')  #创建名为sum_ints_server的节点
    s = rospy.Service('sum_ints', SumInts, handle_sum_ints) #定义服务节点名称，服务的类型，处理函数
    print "Ready to sum ints."
    rospy.spin()	#回调函数

if __name__ == "__main__":
    sum_ints_server()
