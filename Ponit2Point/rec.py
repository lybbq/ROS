#!/usr/bin/env python2.7
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PointStamped,PoseStamped,Quaternion

class tf_listener():
  def __init__(self, node_name ,target_frame,frame):
    rospy.init_node(node_name)
    rospy.Subscriber('/camera_point', PointStamped, self.callback, queue_size=10)
    self.show_point_pub = rospy.Publisher('/base_link_point', PointStamped, queue_size=10)  # map_fream
    self.target_frame = target_frame
    self.frame = frame

  def transform_point(self,x,y,z,frame,target_frame):
    listener = tf.TransformListener()
    point_show = PointStamped()

    point_show.header.frame_id = frame
    point_show.point.x = x
    point_show.point.y = y
    point_show.point.z = z
    new_point = listener.transformPoint(target_frame,ps=point_show)
    return new_point.point.x,new_point.point.y,new_point.point.z
  """
    算法转换思路：
      假设base_link 存在一个点[x,y,z],已知base_link相对于camera_rgb_optical_frame 转换关系[R|T]。
      根据[R | T] *[x，y,z]' = [x_base,y_base,z_base] 
      坐标[x_base,y_base,z_base] 参考于 camera_rgb_optical_frame坐标系其坐标也是[x,y,z]，即左乘后得到的点相对于camera_rgb_optical_frame是不变的，但相对于base_link坐标系变化(左乘)！
      
      注：[x,y,z] 相对于下base_link的(0,0,0)和 [x_base,y_base,z_base] 相对于 camera_rgb_optical_frame (0,0,0)是一致的！
      若已知camera_rgb_optical_frame下的点[x,y,z]，其坐标就是假设base_link的[x,y,z]点，然后再通过上式即可求出[x_base,y_base,z_base]。 故即将处于相机坐标系下的点转换至机器人坐标系。
  """
  def transform_test(self,x,y,z,frame,target_frame):
    listener = tf.TransformListener()

    listener.waitForTransform(target_frame,frame,rospy.Time(0), rospy.Duration(4.0))

    (trans,rot) = listener.lookupTransform(target_frame,frame,rospy.Time(0)) # camera_ to map matrix_

    raw_matrix = tf.transformations.quaternion_matrix(rot)

    raw_matrix = raw_matrix[0:3,0:3]

    point = np.array([[x, y, z]]).reshape(3,1)

    point_transform = np.mat(raw_matrix) * np.mat(point) + np.mat(trans).reshape(3,1)

    point_transform = point_transform.reshape(1,3)

    return point_transform[0,0],point_transform[0,1],point_transform[0,2]

  def pub_point(self,publisher,x,y,z,frame):
    point_show = PointStamped()
    point_show.header.frame_id = '/base_link'
    point_show.point.x = x
    point_show.point.y = y
    point_show.point.z = z
    rospy.sleep(1)
    publisher.publish(point_show)
  def callback(self,pointstamped):


    rospy.loginfo("------------------------callback-------------------")

    point_base_link = self.transform_test(pointstamped.point.x,pointstamped.point.y,pointstamped.point.z,self.frame,
                                          self.target_frame)
    point_base_link2 = self.transform_point(pointstamped.point.x, pointstamped.point.y, pointstamped.point.z, self.frame,
                                           self.target_frame)

    print("My Transform:",point_base_link)

    print("Transform_ponit:",point_base_link2)

    self.pub_point(self.show_point_pub,point_base_link[0],point_base_link[1],point_base_link[2],"/base_link")


if __name__ == "__main__":
  tf_sub = tf_listener("tf_Lisen",'/base_link','/camera_rgb_optical_frame')
  rospy.spin()
