#!/usr/bin/env python
#coding:utf-8
 
import sys
import rospy
from communication_tutorials.srv import *

#定义一个客户端函数
def sum_ints_client(x, y):
    rospy.wait_for_service('sum_ints')  #等待接入服务节点
    try:
        sum_ints = rospy.ServiceProxy('sum_ints', SumInts) #创建服务的处理句柄,add_ints为服务名,AddInts是服务类型
        resp1 = sum_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

#定义函数调用的方式
def usage():
	return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
	#当sys.argv的长度为3时说明有两个参数传进来，将两个参数赋给x,y,否则就退出
	if len(sys.argv) == 3:
		x = int(sys.argv[1])
		y = int(sys.argv[2])
	else:
		print usage()
		sys.exit(1)
	print "Requesting " 
	print "sum = %s" % (sum_ints_client(x, y))
