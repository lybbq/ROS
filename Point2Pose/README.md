

仿真环境条件/说明
------------
* 默认环境:ubuntu18.04
* python2.7
* 安装rbx1、arbotix
* transfrom_server需要运行在python2.7之中
* 加入客户/服务操作可兼容Python3(Yolov5、其他功能)与Python2.7。
* srv文件夹放置工作空间软件包处，修改CmakeList、paceage.xml
* CmakList.txt配置
```
	find_package 加入 geometry_msgs move_base_msgs message_generation
	add_service_files 加入 SrvPoseStamped.srv
	generate_messages 加入 std_msgs  geometry_msgs move_base_msgs
```
* package.xml配置
```
	* <buildtool_depend>catkin</buildtool_depend>
  	* <build_depend>roscpp</build_depend>
  	* <exec_depend>roscpp</exec_depend>
  	* <build_depend>rospy</build_depend>
  	* <exec_depend>rospy</exec_depend>
  	* <build_depend>std_msgs</build_depend>
  	* <exec_depend>std_msgs</exec_depend>
  	* <build_depend>geometry_msgs</build_depend>
  	* <exec_depend>geometry_msgs</exec_depend>
  	* <build_depend>move_base_msgs</build_depend>
  	* <exec_depend>move_base_msgs</exec_depend>
  	* <build_depend>message_generation</build_depend>
  	* <exec_depend>message_runtime</exec_depend>
```
* 示例配置示例
	*当前目录下package.xml、CmakList.txt


运行顺序
------------
* roslaunch rbx1_bringup fake_turtlebot.launch
* roslaunch rbx1_nav fake_nav_test.launch 
* rosrun your_name client.py (your_name更换成工作空间下软件包名)
* rosrun your_name tansfrom_server.py

显示:
------------
* rviz中选择显示'/map_point' Pose（位置、位姿）
* rviz中选择显示''/base_point'' Postion (相机坐标系下目标点)

Client:
------------


Server:
------------






