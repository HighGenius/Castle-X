#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import classify_image


class RosTensorFlow():
    def __init__(self):
        rospy.on_shutdown(self.cleanup);

        # 调用API从tensorflow.org下载训练好的Inception-v3模型
        classify_image.maybe_download_and_extract()
        # 创建一个TensorFlow Session()的会话对象
        self._session = tf.Session()
        # 从保存的GraphDef文件创建一个图像，并返回一个句柄
        classify_image.create_graph()

        # 创建cv_bridge话题
        self._cv_bridge = CvBridge()
        # 创建节点的订阅者和发布者
        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('result', String, queue_size=1)

        # 用于识别阈值和顶层预测数的参数设置
        self.score_threshold = rospy.get_param('~score_threshold', 0.1)
        self.use_top_k = rospy.get_param('~use_top_k', 5)

    def callback(self, image_msg):
        # 将ROS的图像数据转换成OpenCV的图像格式        
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # copy from
        # classify_image.py
        image_data = cv2.imencode('.jpg', cv_image)[1].tostring()

        # 从保存的GraphDef中创建softmax张量，其中softmax:0部分包含1000个标签化预测的张量
        softmax_tensor = self._session.graph.get_tensor_by_name('softmax:0')

        # 运行softmax张量，其中DecodeJpeg/contents:0行是一个包含图像JPEG编码字符串的张量
        predictions = self._session.run(
            softmax_tensor, {'DecodeJpeg/contents:0': image_data})
        predictions = np.squeeze(predictions)

        # 寻找匹配对象的字符串及其概率
        node_lookup = classify_image.NodeLookup()
        top_k = predictions.argsort()[-self.use_top_k:][::-1]
        for node_id in top_k:
            human_string = node_lookup.id_to_string(node_id)
            score = predictions[node_id]
            if score > self.score_threshold:                
                rospy.loginfo('%s (score = %.5f)' % (human_string, score))                
                self._pub.publish(human_string)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        classify_image.setup_args()
        # 初始化ros节点
        rospy.init_node('ros_tensorflow')
        tensor = RosTensorFlow()
        tensor.main()
        
    except KeyboardInterrupt:
        print "Shutting down ros_tensorflow node."
        cv2.destroyAllWindows()
