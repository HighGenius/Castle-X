#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

class faceDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup);

        # 创建cv_bridge话题
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        # 获取haar特征的级联表
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")
        cascade_3 = rospy.get_param("~cascade_3", "")

        # 初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)
        self.cascade_3 = cv2.CascadeClassifier(cascade_3)

        # 设置级联表的参数
        self.scaleFactor  = rospy.get_param("scaleFactor", 1.1)
        self.minNeighbors = rospy.get_param("minNeighbors", 5)
        self.minSize      = rospy.get_param("minSize", 40)
        self.maxSize      = rospy.get_param("maxSize", 60)
        self.color = (50, 255, 50)

        # 初始化订阅rgb格式图像数据的订阅者
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # 创建灰度图像
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)

        # 尝试检测人的眼睛
        faces_result = self.detect_face(grey_image)

        if len(faces_result)>0:
        	 for (x, y, w, h) in faces_result:
				cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


    #定义检测人脸的函数
    def detect_face(self, input_image):
        # 匹配人的正脸模型
		if self.cascade_1:
			faces = self.cascade_1.detectMultiScale(input_image, 
					self.scaleFactor, 
					self.minNeighbors, 
					cv2.CASCADE_SCALE_IMAGE, 
					(self.minSize, self.maxSize))

        # 如果匹配人的正脸失败就开始匹配人的侧脸模型
		if len(faces) == 0 and self.cascade_2:
			faces = self.cascade_2.detectMultiScale(input_image, 
					self.scaleFactor, 
					self.minNeighbors, 
					cv2.CASCADE_SCALE_IMAGE, 
				(self.minSize, self.maxSize))
		else:
			pass

        # 如果脸匹配成功，那么就尝试匹配人的眼睛模型
		if len(faces) != 0 and self.cascade_3:
			faces = self.cascade_3.detectMultiScale(input_image, 
					self.scaleFactor, 
                    self.minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.minSize, self.maxSize))  
		return faces

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("astra_cv_close_eye_detect")
        faceDetector()
        #rospy.loginfo("Eyes detector is started..")
        rospy.loginfo("Astra_cv_close_eye_detect is started.. \n Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down astra_cv_close_eye_detect node."
        cv2.destroyAllWindows()