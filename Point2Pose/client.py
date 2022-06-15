import rospy
import numpy as np
from cmoon.srv import SrvPoseStamped,SrvPoseStampedRequest
from geometry_msgs.msg import TransformStamped, PointStamped,PoseStamped
class Client:
    def __init__(self):
        self.base = Base()
        self.server_name = '/trans_nav_pose'
    def get_pointStamped(self,point,frame_name='/camera_rgb_optical_frame'):
        point_show = PointStamped()
        point_show.header.frame_id = frame_name
        point_show.point.x = point[0]
        point_show.point.y = point[1]
        point_show.point.z = point[2]

        return point_show
    """
        目标相机坐标下坐标点，dep->深度距离mm。 
    """
    def send(self,center_x,center_y,dep):
        try:
            pointstamped = self.get_pointStamped([center_x * dep / 1000.0, center_y * dep / 1000.0, dep / 1000.0])
            """
                服务返回数据。[Position, Orientation]
            """
            client = rospy.ServiceProxy(self.server_name, SrvPoseStamped)

            respond = client(pointstamped)

            base_link_position = [respond.ser.target_pose.pose.orientation.x, respond.ser.target_pose.pose.orientation.y,
                                  respond.ser.target_pose.pose.orientation.z]

        except rospy.ServiceException:
            print("Transform Service is failed")

if __name__ == '__main__':
    client = Client()
    client.send(-0.5,0.3,1000)
    rospy.spin()