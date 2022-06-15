仿真运行条件
------------
* 默认环境:ubuntu18.04
* python2.7
* rbx1、arbotix
* 放置pub.py、rec.py文件进入工作空间的软件包处

运行顺序
------------
```
roslaunch rbx1_bringup fake_turtlebot.launch
roslaunch rbx1_nav fake_nav_test.launch 
rosrun your_name pub.py (your_name更换成工作空间下软件包名)
rosrun your_name rec.py
 ```

Publisher:
------------
发布'/camera_rgb_optical_frame'

Subscribe:
------------
订阅'/camera_rgb_optical_frame'


