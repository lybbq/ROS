#!/usr/bin/env python2.7
# coding: UTF-8
import rospy
from geometry_msgs.msg import Twist
import tf
import numpy as np
import math
import time
from geometry_msgs.msg import PointStamped,PoseStamped,Quaternion
from move_base_msgs.msg import MoveBaseGoal
from cmoon.srv import SrvPoseStamped,SrvPoseStampedResponse,SrvPoseStampedRequest
class tf_listener():
  """
  current_new
  callback 回调内的map_link_zw都为弧度值。
  pub_point get_pose 可发布消息以便可在地图上查看，验证正确性。
  输入：传入服务PoinStamped点为相机模型下的坐标。
  输出：返回Map地图中导航点。
  调用说明：
    本客户调用服务在Demo1.py——>run函数中。
  """
  def __init__(self, str ,target_frame,frame,distance=0.4):
    rospy.init_node(str)
    rospy.Service('/trans_nav_pose', SrvPoseStamped, self.callback)
    self.show_pose_pub = rospy.Publisher('/map_point', PoseStamped, queue_size=10) # map_fream
    self.show_point_pub = rospy.Publisher('/base_point', PointStamped, queue_size=10)  # map_fream
    self.target_frame = target_frame
    self.frame = frame
    self.offset_distance = distance
  def transform_frame(self,x,y,z,frame,target_frame):
    listener = tf.TransformListener()
    point_show = PointStamped()
    new_point = PointStamped()
    point_show.header.frame_id = frame
    point_show.point.x = x
    point_show.point.y = y
    point_show.point.z = z
    new_point = listener.transformPoint(target_frame,ps=point_show)
    return new_point.point.x,new_point.point.y,new_point.point.z

  def get_movebase(self,x,y,z,rot):
    r = MoveBaseGoal()
    #r.target_pose.header.stamp = rospy.Time.now()
    r.target_pose.header.frame_id = 'map'
    r.target_pose.pose.position.x = x
    r.target_pose.pose.position.y = y
    r.target_pose.pose.position.z = z
    r.target_pose.pose.orientation.x = rot[0]
    r.target_pose.pose.orientation.y = rot[1]
    r.target_pose.pose.orientation.z = rot[2]
    r.target_pose.pose.orientation.w = rot[3]
    return  r

  def get_pose(self,publisher,x,y,z,rot):
    point_show = PoseStamped()
    #point_show.header.stamp = rospy.Time.now()
    point_show.header.frame_id = 'map'
    point_show.pose.position.x = x
    point_show.pose.position.y = y
    point_show.pose.position.z = z
    point_show.pose.orientation.x = rot[0]
    point_show.pose.orientation.y = rot[1]
    point_show.pose.orientation.z = rot[2]
    point_show.pose.orientation.w = rot[3]
    publisher.publish(point_show)

    return point_show

  def pub_point(self,publisher,x,y,z,frame='/base_link'):
    point_show = PointStamped()
    point_show.header.frame_id = frame
    point_show.point.x = x
    point_show.point.y = y
    point_show.point.z = z
    rospy.sleep(1)
    publisher.publish(point_show)

  """
    求机器人与目标角度偏差，使机器人转向目标
  """
  def rectify_angle(self,base_link_x,base_link_y,raw):
    offset_zw = math.atan2(base_link_y, base_link_x) / math.pi * 180
    raw = raw / math.pi * 180
    raw += offset_zw # angle
    if raw >= 360:
      raw -= 360
    elif raw <= -360:
      raw += 360
    raw = raw / 180 * math.pi
    return raw # return  rad
  """
    根据offset_distance即距离目标多少距离，计算出导航点的Position
  """
  def get_map_point(self,base_link_x,base_link_y,map_link_x,map_link_y,map_link_zw,offset_distance): #
    dis = math.sqrt(base_link_x**2 + base_link_y**2)
    dis -= offset_distance
    point = [map_link_x + dis * math.cos(map_link_zw), map_link_y + dis * math.sin(map_link_zw), 0]
    rot = tf.transformations.quaternion_from_euler(0, 0, map_link_zw)
    return  point,rot

  def callback(self,pointstamped):
    """
    callback 回调内的map_link_zw都为弧度值。
    pub_point get_pose 可发布消息以便可在地图上查看，验证正确性。
    """
    # Current Corr
    start_time = time.time()
    listener = tf.TransformListener()
    rospy.loginfo("------------------------callback-------------------")
    listener.waitForTransform("/map","/base_link",rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/map","/base_link",rospy.Time(0))

    _,_,map_link_zw = tf.transformations.euler_from_quaternion(rot) # rad

    point_base_link = self.transform_frame(pointstamped.res.point.x,pointstamped.res.point.y,pointstamped.res.point.z,self.frame,'/base_link')

    map_link_zw = self.rectify_angle(point_base_link[0],point_base_link[1],map_link_zw)

    new_point ,new_rot= self.get_map_point(point_base_link[0],point_base_link[1],trans[0],trans[1],map_link_zw,self.offset_distance)
    """
    验证正确性。
    """
    self.pub_point(self.show_point_pub, point_base_link[0], point_base_link[1], point_base_link[2], "/base_link")
    self.get_pose(self.show_pose_pub,new_point[0],new_point[1],new_point[2],new_rot)

    movepose = self.get_movebase(new_point[0],new_point[1],new_point[2],new_rot)
    end_time = time.time()
    print("frame_time:", end_time - start_time)
    return movepose

if __name__ == "__main__":
  tf_sub = tf_listener("tf_Lisen",'/map','/camera_rgb_optical_frame',distance=0.4)
  rospy.spin()
  #tf_sub.spin1()
