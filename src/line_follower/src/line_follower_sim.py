#!/usr/bin/env python
import rospy
import cv2
import cv_bridge  
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from obstacle_avoid import LeftWallFollower # my own script!
# is_obstacle_near is introduced here!

# 【最新版本】
# 存在的问题：出现障碍物后，在follow-left-wall的过程中：
#  1、转的时候看到了线，但是线比较边缘或者在脚底，于是直接略过去了
#  2、因为总是向右转，所以如果遇到一个倒角度的障碍物，向右转半天后又看到自己来的时候的线了，这个时候就会返回去
# 这两个问题可以通过巧妙摆放障碍物的方式避免，交满分作业没什么问题

# UPDATE TIME：Oct.31

# 本版本特征：增加三个extra functions，并恢复P控制（而非PID控制）

# 重点知识：ROS 的回调机制和主循环是异步的，while 循环中的条件更新有可能没有及时反映在机器人实际运动状态上。


class LineFollowerSim:
    def __init__(self):
        # 将图像转换为OpenCV的格式
        self.bridge = cv_bridge.CvBridge()

        # 不要包括这部分
        # [Modify with the tutorial in Textbook] don't add this line, the version is different
        # cv2.namedWindow("window", 1)
        # in image_callback function => cv2.imshow("window", image_bgr)

        # subscribe摄像头的图像内容，然后call image_callback
        # 注意，再次提醒，callback function是由ROS订阅机制自动触发的
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                          Image, self.image_callback)

        # 发布速度控制消息Twist
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # 速度控制消息Twist
        self.twist = Twist()

        # 预准备callback中要更新的值，写在这里是因为主程序在callback启动前就读取了
        self.cx = -1
        self.cy = -1
        self.middle = -1

        # 初始化P控制的最大限速参数
        self.max_angular_speed = 0.5 # 转的太快了，加一个最大值

        # 这几个是为了增加的3个功能提供的
        self.obstacle = False
        self.rotation_count = 0
        self.rotation_started = False
        self.explore_state = False

        # Object LeftWallFollower for avoiding the obstacle
        self.left_wall_follower = LeftWallFollower()

    def image_callback(self, msg):
        """
        Callback to `self.image_sub`.
        该方法处理接收到的图像数据，并对图像进行处理以识别黄色线条（或者别的颜色的）
        """
        # ==============================================
        # PART A: Receive the image from the camera
        # 该步骤主要是将接收到的图像转换为OpenCV格式的RGB图像

        # Print message type to verify what is being received (hide, but remain for spare)
        # rospy.loginfo(f"Received image with encoding: {msg.encoding}")
                    
        # Attempt to convert the ROS Image message to an OpenCV image
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        
        # OpenCV uses BGR format, so needs change it, if not, the view will recognize yellow as blue
        image_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        
        # Resize the image
        resized_image = cv2.resize(image_bgr, (640, 480), interpolation=cv2.INTER_LINEAR)

        # show normal image (hide, but keep for spare)
        cv2.imshow('normal_image', resized_image)

        # ================================================
        # PART B: HSV MASK
        # 该步骤主要是将图像转换为HSV格式
        # 然后创建一个mask，只保留黄色区域
        # Use HSV images to help recognize yellow line more efficient
        # [Modify with the tutorial in Textbook]Color Range for Yellow in HSV: The lower and upper bounds for yellow in HSV color space might not be correct. 
        # yellow in HSV has a Hue value around 30-60.
        hsv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([20, 100, 100]) # Textbook: [50, 50, 170]
        upper_yellow = numpy.array([30, 255, 255]) # Textbook: [255, 255, 190]

        # mask hsv
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        
        # show mask hsv (hide, but keep for spare)
        cv2.imshow('mask_image', mask)

        # masked image with yellow line
        masked_hsv_img = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        
        # show masked image with yellow line (hide, but keep for spare)
        cv2.imshow('masked_hsv_image', masked_hsv_img)

        # =====================================================
        # PART C: Draw a circle
        # 画出黄色线条的中心点 （Centroid Detect）
        # use mask hsv image to create a circle
        # 这部分来自于textbook，整体逻辑是：
        # 设定一个区域search_top - search_bot，在该区域寻找黄色线条的中心点
        # 使用cv2.moments(mask) 计算centroid质心坐标
        h, w, d = resized_image.shape
        search_top = int(3*h/4) # here cause an error, and casting to int avoid that error successfully
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # M是通过上述计算得到的最终最重要的 "mask image moments", 这些moments包含了最重要的特征
        # 将M设置为instance variable
        M = cv2.moments(mask)

        # update cx, cy, w 
        # 之所以要在init中就定义，是因为程序运行后callback还没发动的时候，需要识别cx的值
        # set (x,y) , 这个坐标代表质心的位置，该位置在图像中点出来，坐标范围取决于图像的分辨率
        # 由于图像被我压缩到了640*480的范围，因此cx的范围是0 - 639，cy的范围是0到479
        # -1代表该质心不在图像中，也就是没有找到可识别的符合要求的线条
        self.cx = -1
        self.cy = -1
        self.middle = w

        # ==============================================
        # detect yellow line and get the centroid circle
        if M['m00'] > 0:
            self.cx = int(M['m10']/M['m00']) # 质心的x坐标，表示黄色线条在图像中的水平位置
            self.cy = int(M['m01']/M['m00']) # 质心的y坐标，用于标记质心位置，不影响机器人的方向
            cv2.circle(resized_image,(self.cx,self.cy), 20, (0,0,255), -1) # .circle 绘制一个红色圆圈，以进行标记
        else:
            self.cx = -1
            self.cy = -1

        # show circle image
        cv2.imshow('circle', resized_image)

        # this is for cv2.imshow
        cv2.waitKey(3)

    def follow_line(self):
        """Follow a yellow line."""

        # extra functions:
        # 1. Robot starts somewhere where the line is not immediately visible
        # 2. Robot is able to double back along the line allowing it to follow the line infinitely
        # 3. Robot uses Lidar (/scan) to detect an obstacle in the way. Place an object in gazebo of your choice to move around. The more “challenging” choice of obstacle(s) will mean more extra points awarded

        """
        robot启动：
        if 发现yellow line：
            Follow
        else if 没有障碍物，没有线条，explore_state为false（情况1的第一种情况、情况2）：
            先原地旋转2圈，如果依然没有，则进入附近没有线的状态（far away， explore_state = True）
            linear = 0
            angular = 0.3
        else if 没有障碍物，没有线条，explore_state为true（情况1的第二种情况）:
            一直直走
            linear = 0.5
            angular = 0
        else if 有障碍物
            沿着左侧墙壁前进
        else 
            理论上没有其他情况了，但也有可能出现其他情况，该情况留作debug用
        """

        # update obstacle status
        # usage: argument is the detecting distance, over this distance will not detect
        self.obstacle = self.left_wall_follower.is_obstacle_near(0.5)

        if self.cx >= 0 and self.cy >= 0:
            print("SITUATION 1: find yellow line")
            # Reset functional state variables
            self.explore_state = False
            self.rotation_started = False

            # Proportional Control (Only P!)
            err = self.cx - self.middle/2
            self.twist.linear.x = 0.2
            angular_velocity = max(min(-float(err) / 100, self.max_angular_speed), -self.max_angular_speed)
            self.twist.angular.z = angular_velocity 

            print("linear speed: ", self.twist.linear.x)
            print("angular speed:", self.twist.angular.z)

        elif self.cx < 0 and self.cy < 0 and self.obstacle == False and self.explore_state == False:
            """
            原地顺时针旋转720度：
                当旋转过程中出现黄色线条时，跳出循环
            """
            print("SITUATION 2: no line, no obstacle, explore_state is False")
            
            if not self.rotation_started:
                # 初始设置旋转参数
                self.rotation_started = True
                self.rotation_count = 0

            # 持续发布旋转命令
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.3  # 顺时针旋转
            self.rotation_count += 1

            # 如果找到线条，停止旋转
            if self.cx >= 0 and self.cy >= 0:
                print("Yellow line found during rotation, stopping rotation.")
                self.twist.angular.z = 0.0
                self.rotation_started = False
                self.explore_state = False

            # 旋转一段时间后仍未找到线条，进入探索模式
            elif self.rotation_count > 300:  
                # 300 次大约相当于旋转1.5圈左右
                # rate = 10， 10Hz的频率，角速度是0.3，根据这些可以粗略估计
                print("Rotation completed, entering exploration mode.")
                self.rotation_started = False
                self.explore_state = True

        elif self.cx < 0 and self.cy < 0 and self.obstacle == False and self.explore_state == True:
            print("SITUATION 3: no line, no obstacle, explore_state is True")
            # 探索模式
            # 不旋转，直线前进
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0
            if self.cx >= 0 and self.cy >= 0:
                self.explore_state = False

        elif self.cx < 0 and self.cy < 0 and self.obstacle == True:
            print("SITUATION 4: no line, but has obstacle. \nLeft_wall_follower Mode")
            self.twist = self.left_wall_follower.follow_left_wall()

        else:
            # situation should not exist, debug situation
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            print("linear speed: ", 0)
            print("angular speed: ", 0)

        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        """
        Run the Program.
        启动ROS node，并在node关闭前持续调用follow_line方法，设置循环频率为10hz
        """
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            print("\n==================")
            self.follow_line()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('line_follower_sim')
    follower = LineFollowerSim()
    follower.run()
    
