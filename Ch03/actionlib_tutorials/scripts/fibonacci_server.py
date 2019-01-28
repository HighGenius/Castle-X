#! /usr/bin/env python
#coding:utf-8

import rospy
import actionlib
import actionlib_tutorials.msg

class FibonacciAction(object):
    # 创建用于发布反馈(feedback)/结果(result)的消息
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # 附加种子用于fibonacci序列
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # 发布信息到控制台，用于用户查看
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # 开始运行行为
        for i in range(1, goal.order):
            # 检查客户端是否未请求抢占
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            # 计算fibonacci序列并反馈回去
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # 发布反馈(feedback)
            self._as.publish_feedback(self._feedback)
            # 这个步骤不是必要的，用于演示效果所以序列会以1Hz完成。
            r.sleep()
          
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()










