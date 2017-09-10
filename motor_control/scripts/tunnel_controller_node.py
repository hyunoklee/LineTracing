#!/usr/bin/env python
import rospy
import numpy as np
import math
from turtlebot3_auto_msgs.msg import  Twist2DStamped, LanePose, ImgSignals
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from time import sleep
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from scipy.spatial import distance

class tunnel_controller(object):
#	def __init__(self):
#		self.node_name = rospy.get_name()	

    def __init__(self):
        self.node_name = rospy.get_name()

        self.con_state ="LINE" # LINE, BAR, PARKING, TUNNEL, LED,
        self.TunnelState = "WAIT"
        #self.TunnelState = "START"
        self.StartOdom = None
        self.CurOdom = None
        self.FirstCycle = True
        self.gostart_distance = 0
        self.SearchStop = None
        self.obstacle = False
        # Publicaiton
        self.pub_cmd_vel_tunnel = rospy.Publisher('/cmd_vel_tunnel', Twist, queue_size=5)
        self.pub_con_state_change = rospy.Publisher ("/con_state_change", String, queue_size=1)
        self.pub_motor_control_start = rospy.Publisher("/motor_control_start", Bool, queue_size=1)

        # Subscriptions
        self.sub_laser_scan = rospy.Subscriber ("/scan", LaserScan, self.cbLaserScan, queue_size=1)
        self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
        self.sub_odom_reading = rospy.Subscriber ("/con_state", String, self.cbConState, queue_size=1)
        self.sub_imu_reading = rospy.Subscriber("/imu", Imu, self.cbImuState, queue_size=1)

        rospy.Timer(rospy.Duration.from_sec(3.0), self.send_motor_start)

        #print("sne pub_motor_control_start")
        ##### Line Detect Side
        #import time
        #self.start_time = rospy.get_time()
        #self.sub_motor_start = rospy.Subscriber("/motor_control_start", Bool, self.cbMotorControlStart, queue_size=1)
        #self.MotorStartTime = rospy.get_time()
    def cbImuState(self,imu_msg):
        imu = [imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w]
        #print("cbImuState : ",imu)

    def send_motor_start(self,_event):
        self.pub_motor_control_start.publish(True)
        print("sne send_motor_start")
    '''
    def cbMotorControlStart(self, start_msg) :
        #starttt = start_msg.data
        self.MotorStart = True
    def dstewtwt
        cur_time = rospy.get_time() 
        print(  "time_gap :", cur_time - self.start_time )        
    '''
    def cbOdometry(self, odom_msg):
        self.CurOdom = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.orientation.w]
        if self.FirstCycle == True :
            print("FirstCycle ...............")
            self.StartOdom = self.CurOdom
            self.StartOdom = [ 0.5710443258285522, -1.4218412637710571]
            self.FirstCycle = False

        P1 = (self.CurOdom[0], self.CurOdom[1])
        P2 = (self.StartOdom[0], self.StartOdom[1])
        dst = distance.euclidean(P1, P2)

        print(dst, self.CurOdom[0], self.CurOdom[1],self.StartOdom[0],self.StartOdom[1] )
        #return

        if self.TunnelState == "START" :
            print("START ...............")
            self.vehicleStop()
            self.TunnelState = "ANG_SEARCH_START"

        if self.TunnelState == "ANG_SEARCH_START" :
            print("ANG_SEARCH_START ...............")
            twist = Twist();twist.linear.x = 0;twist.angular.z = 0.1;
            self.pub_cmd_vel_tunnel.publish(twist)
            sleep(2)
            self.gostart_distance = dst
            self.TunnelState = "LIDAR_CHECK"

        if self.TunnelState == "LIDAR_CHECK":
            print("LIDAR_CHECK ...............")
            self.vehicleStop()
            sleep(0.3)
            if self.obstacle  == False :
                self.vehicleStop()
                self.TunnelState = "GO"
            else :
                self.TunnelState = "ANG_SEARCH_START"

        if self.TunnelState == "GO" :

            twist = Twist(); twist.linear.x = 0.05; twist.angular.z = 0.0;
            self.pub_cmd_vel_tunnel.publish(twist)
            sleep(0.1)
            self.current_distance = dst
            print("GO ...............", self.current_distance, self.gostart_distance, self.obstacle)
            if self.current_distance  >  self.gostart_distance :
                self.TunnelState = "ANG_SEARCH_START"
                self.vehicleStop()
                print("GO ...............1", self.current_distance, self.gostart_distance, self.obstacle)
            if self.obstacle == True :
                self.TunnelState = "ANG_SEARCH_START"
                self.vehicleStop()
                print("GO ...............2", self.current_distance, self.gostart_distance, self.obstacle)

    def vehicleStop(self):
        twist = Twist(); twist.linear.x = 0; twist.angular.z = 0;
        self.pub_cmd_vel_tunnel.publish(twist)

    def cbConState(self, con_state_msg):
        #print("cbConState ...............", self.TunnelState , con_state_msg.data )
        if self.TunnelState == "WAIT" and con_state_msg.data == "TUNNEL":
            self.TunnelState = "START"
            #print("cbConState ...............START")

    def cbLaserScan(self, laser_scan_msg):
        #return
        self.laser_scan = laser_scan_msg.ranges
        self.obstacle = False
        rangg = 20
        for i in range(0,rangg*2) :
            d = i - rangg
            if d < 0 :
               d = 360 + d
            else :
                d = i
            if self.laser_scan[d] != 0 and self.laser_scan[d] < 0.5:
               #rospy.loginfo("_%d , %0.3f" % (d, self.laser_scan[d]))
               self.obstacle = True
        print("obstacle " , self.obstacle )


if __name__ == "__main__":
	rospy.init_node("tunnel_controller",anonymous=False)
	tunnel_control_node = tunnel_controller()
	rospy.spin()
