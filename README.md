ROS简单根据目标点转换导航点
===============
* Linux: ubuntu18.04
* python2.7
* 比赛项目ros代码:https://github.com/Cmoon-cyl/ros-module
* 条件：附近无障碍物，静态有可能也不能使机器人导航至目标点，若有动态障碍物机器人自行根据局部路径规划自行是否判断。
* 视觉位置转换至机器人坐标点（Point,Point）,代码参考**Ponit2Point** (前置知识)
* 视觉转换至导航点（Point,Pose）代码参考**Point2Pose**
* Demo1.py存放2021年中国机器人大赛测试代码，根据比赛项目代码进行拓展。
	* 功能：
		* 根据Yolov5识别出的目标计算出导航点（仿真测试）。此版本作为测试，以及新相机还未进行标定若用则先进行标定，另外尚未能结合机械臂，若后续想结合机械臂，请先进行手眼标定后再抓取！

效果图
===============

<figure class="half">
    <img src="https://img-blog.csdnimg.cn/1315767b557349f4b733293e04568f80.png " width="300">
    <img src="https://img-blog.csdnimg.cn/6d1dbb51ec4249c88d56efea2b2e46be.png" width="310" >
</figure>

