#! /usr/bin/env python
#coding:utf-8

from __future__ import print_function

import sys
import rospy
# 加载SimpleActionClient
import actionlib

# 加载fibonacci行为消息，包含目标消息和结果消息
import actionlib_tutorials.msg

def fibonacci_client(x):
    # 创建SimpleActionClient，传递行为类型
    # (FibonacciAction) 到构造函数。
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

# 等待，直到行为服务器挂起或运行。
    # 监听目标。
    client.wait_for_server()

    # 创建一个目标发送到行为服务器。
    goal = actionlib_tutorials.msg.FibonacciGoal(order=x)

    # 发送目标到行为服务器。
    client.send_goal(goal)

    # 等待服务器完成处理行为。
    client.wait_for_result()

    # 打印运行行为结果。
    return client.get_result()  # A FibonacciResult

#定义函数调用的方式
def usage():
	return "%s [x y]"%sys.argv[0]


if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('fibonacci_client_py')
	# 从终端接收输入的参数
	if len(sys.argv) == 2:
		x = int(sys.argv[1])
	else:
		print(usage())
		sys.exit(1)

	#返回计算结果并在终端显示
        result = fibonacci_client(x)
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)









