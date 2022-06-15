#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import PointStamped
class puber():
    def __init__(self):
        rospy.init_node("twist_t11")
        self.pub = rospy.Publisher('/camera_point',PointStamped ,queue_size=10)

    def send(self,point):
        point_show = PointStamped()
        point_show.header.frame_id = '/camera_rgb_optical_frame'
        point_show.point.x = point[0]
        point_show.point.y = point[1]
        point_show.point.z = point[2]

        rospy.sleep(1)
        self.pub.publish(point_show)

if __name__ == '__main__':
    pub = puber()
    i = 0
    while i < 2  and not rospy.is_shutdown():
        pub.send([-0.25,-0.25,0.2])
        i += 0.2
        rospy.sleep(5)
    rospy.spin()
        
