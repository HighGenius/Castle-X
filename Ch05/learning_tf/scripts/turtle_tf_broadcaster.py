#!/usr/bin/env python
#coding:utf-8
import roslib
roslib.load_manifest('learning_tf')
import rospy
import tf
import turtlesim.msg

# 定义handle_turtle_pose函数
def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()

    # 发布从世界坐标系到turtlex坐标系的变换（包括turtle的平移和旋转）
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")
if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')    # 初始化节点
    # 调用get_param()函数来获取参数，该参数指定一个Turtle名称
    turtlename = rospy.get_param('~turtle') 
    # 订阅话题"turtleX/pose"，并对每个传入消息运行handle_turtle_pose函数
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()    # 回调函数