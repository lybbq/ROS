#!/usr/bin/env python3
# coding: UTF-8 

import rospy
import numpy as np
from base_controller import Base  # 底盘运动模块
from Detector import ObjectDetector
import math
from sensor_msgs.msg import Image
import cv2 as cv
# from  tf_transformer import Transformer
from cmoon.srv import SrvPoseStamped,SrvPoseStampedRequest
from geometry_msgs.msg import TransformStamped, PointStamped,PoseStamped

# from transform import transform_pose
class Tester:
    def __init__(self):
        self.base = Base()
        self.yolo = ObjectDetector()
        self.list = []
        self.threshold = 10  # 判断是否舍去的阈值,需要调整
        # carmera parma
        self.server_name = '/trans_nav_pose'
        self.k = np.array([
        [922.304443359375, 0.0, 953.6251220703125],
        [ 0.0, 922.151611328125, 547.738037109375],
        [ 0.0, 0.0, 1.0]],dtype=np.float64)
        self.d = np.array([0.6654037833213806, -2.737891435623169, 0.0006264661205932498, -0.00046948797535151243,
                           1.4830920696258545, 0.5418514013290405, -2.565876007080078, 1.4164302349090576], dtype=np.float64)
        self.ros_image_depth = None
        rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.image_dep_callback, queue_size=1, buff_size=52428800)
    def get_pointStamped(self,point,frame_name='/camera_link_s'):
        point_show = PointStamped()
        point_show.header.frame_id = frame_name
        point_show.point.x = point[0]
        point_show.point.y = point[1]
        point_show.point.z = point[2]
        return point_show

    def run(self):
        """
        resolution是相机分辨率
        rotate是底盘旋转速度
        range是占画面中间区域百分之多少
        返回的self.result是列表,里面存的是类,有name,box,x,y四个属性
        name是检测到的物品名,box是bounding box的xyxy,x和y是中心点坐标
        """
        while not rospy.is_shutdown():
            self.result, image_depth = self.yolo.detect(device='k4a', mode='detect', depth=True, rotate=0,
                                           range=0.5)
            for object in self.result:
                print(object)
                # stop

                src = np.array([[[object.x,object.y]]])
                "camera link"
                cam_point = cv.undistortPoints(src, self.k, self.d)
                center_x = cam_point[0][0][0]
                center_y = cam_point[0][0][1]

                if image_depth is not None and image_depth[int(object.y)][int(object.x)].item() > 0:
                    dep = image_depth[int(object.y)][int(object.x)].item()
                    rospy.wait_for_service(self.server_name)
                    try:
                        pointstamped = self.get_pointStamped([center_x * dep / 1000.0,center_y * dep/1000.0,dep/1000.0])
                        client = rospy.ServiceProxy(self.server_name,SrvPoseStamped)
                        respond = client(pointstamped)
                        base_link_position = [respond.ser.target_pose.pose.orientation.x,respond.ser.target_pose.pose.orientation.y,respond.ser.target_pose.pose.orientation.z]
                        print(base_link_position )
                        self.get_pointStamped(base_link_position)
                    except rospy.ServiceException:
                        print("Transform Service is failed")



                if len(self.list):
                    if self.distance(object, self.list[-1]) >= self.threshold:
                        self.list.append(object)
                        print(f'Object {object.name} ACCEPTED!!!')
                    else:
                        print(f'Object {object.name} REJECTED!!!')
                else:
                    self.list.append(object)
                    print(f'Object {object.name} ACCEPTED!!!')

            if len(self.list) >= 5:
                print(f'final result:{self.list}')
                break
    def image_dep_callback(self, image):
        self.ros_image_depth  = np.frombuffer(image.data, dtype=np.uint16).reshape(image.height, image.width,-1)


    def distance(self, point1, point2):
        x1, y1 = point1.x, point1.y
        x2, y2 = point2.x, point2.y
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


if __name__ == '__main__':
    try:
        rospy.init_node('name', anonymous=True)
        tester = Tester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
