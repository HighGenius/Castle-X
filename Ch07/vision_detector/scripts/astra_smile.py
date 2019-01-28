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
        faceCascade = rospy.get_param("~faceCascade", "")
        smileCascade = rospy.get_param("~smileCascade", "")

        # 初始化haar特征检测器
        self.faceCascade = cv2.CascadeClassifier(faceCascade)
        self.smileCascade = cv2.CascadeClassifier(smileCascade)

        # 设置级联表的参数
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (0, 255, 255)

        # 初始化订阅rgb格式图像数据的订阅者
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):

        # 将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # 创建灰度图像
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)

        # 尝试检测人脸
        faces_result = self.detect_face(grey_image)

        # 在opencv的窗口中用矩形框出所有人脸区域
        if len(faces_result)>0:
            for face in faces_result: 
                x, y, w, h = face
                # 画一个矩形把人脸框出来
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
                roi_gray = cv_image[y:y+h, x:x+w]
                roi_color = cv_image[y:y+h, x:x+w]

                # 对人脸进行笑脸检测
                smile = self.smileCascade.detectMultiScale(
                    roi_gray,
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors,
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))

                # 框出上扬的嘴角并对笑脸打上Smile标签
                for (x2, y2, w2, h2) in smile:
                    cv2.rectangle(roi_color, (x2, y2), (x2+w2, y2+h2), (255, 0, 0), 2)
                    cv2.putText(cv_image,'Smile',(x,y-7), 3, 1.2, (0, 255, 0), 2, cv2.LINE_AA)

        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def detect_face(self, input_image):
        # 匹配人脸模型
        if self.faceCascade:
            faces = self.faceCascade.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
        
        return faces

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("astra_smile")
        faceDetector()
        rospy.loginfo("astra_smile is started.. \n Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down astra_smile node."
        cv2.destroyAllWindows()
