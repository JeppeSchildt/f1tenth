#!/usr/bin/env python
"""
#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

def callback(msg=None):
    # steer 0.34 -0.34
    if msg is not None:
	#rospy.loginfo('hilo: %.2f',np.mean(msg.ranges[500:580]))
	rospy.loginfo('min: %.2f max %.2f increment %.6f',msg.angle_min,msg.angle_max,msg.angle_increment)
        if np.mean(msg.ranges[500:580]) < 1.5:
	    actuate(0)
        else:
	    actuate(0)
	    #ack_msg.drive.steering_angle = -math.pi/2.0 
        #ack_msg.drive.steering_angle = math.pi/2.0

def actuate(speed, angle=None,frameid=None):
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    if frameid is not None:
        ack_msg.header.frame_id = frameid
    if angle is not None:
        ack_msg.drive.sterring_angle = angle
    ack_msg.drive.speed = speed
    pub.publish(ack_msg)

def limitercheck(speedmsg):
    if (speedmsg.drive.speed > 3):
        rospy.loginfo("Speed lowered to 3 from: %.2f",speedmsg.drive.speed)
        actuate(3)

def body():
    rospy.init_node('test_node')
    rate = rospy.Rate(5)	
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try: 
	pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop',AckermannDriveStamped,queue_size=1)
        sub = rospy.Subscriber('/scan',LaserScan,callback)
        sub2 = rospy.Subscriber('/vesc/low_level/ackermann_cmd_mux/output',AckermannDriveStamped,limitercheck)
        body()
    except rospy.ROSInterruptException:
        pass
"""

from pickle import TRUE
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class Brain():
    def __init__(self):
        
        rospy.init_node('Brain', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz
        #SPEED LIMITER
        self.speed_limit = 2 #SET DOWN LATER, VERY HIGH
        self.current_speed = 0
        self.EMERGENCY_STOP = False
        self.MANUAL_CONTROL = False
        self.ppControl = False
        self.scantime_interval = rospy.Duration(secs=0.2)
        self.last_scan = rospy.Time.now()
        self.ttc_threshold = 0.8 #Not tested!
        #Subscribing to lidar data from scan topic
        self.sub1 = rospy.Subscriber('/scan', LaserScan, self.emergency_brake)

        #Subscribing to desired speed and steering angle data recieved from DT
        self.sub2 = rospy.Subscriber('/DTdata', AckermannDriveStamped, self.digital_twin_control)   

        #Subscribing to buttons data from joy to check if manual control is ON
        self.sub3 = rospy.Subscriber('/vesc/joy', Joy, self.control_switch)

        #Subscribing to desired speed and steering angle data recieved from manual_controller
        self.sub4 = rospy.Subscriber('/MCdata', AckermannDriveStamped, self.manual_control)

        self.sub5 = rospy.Subscriber('/ppdata',AckermannDriveStamped,self.purepursuit)

        #Publishing to Teleop to control the car
        self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop',AckermannDriveStamped,queue_size=1)
        

    def emergency_brake(self, scan):  
        self.ranges = scan.ranges
        # TIME TO COLLISION SHOULD BE IMPLEMENTED HERE
        if self.current_speed > 0 and rospy.Time.now()-self.last_scan > self.scantime_interval: #TTC -> inf if dist deriv = 0
            ttcLHS = []
            ttcRHS = []
            rospy.loginfo("ttc check beginning")
            for index, p in enumerate(self.ranges[360:540]): #45degree LHS
                if not math.isinf(p) and not math.isnan(p):
                    angle = math.radians(45)-scan.angle_increment*index
                    distance_deriv = self.current_speed*math.cos(angle)
                    self.last_scan = rospy.Time.now()
                    if (distance_deriv > 0): #Ensure no division by 0
                        ttcLHS.append(p/distance_deriv)

        #Tests for both lhs and rhs, sign doesnt matter for cosine calculations
            for index, p in enumerate(self.ranges[540:720]): #45degree RHS
                if not math.isinf(p) and not math.isnan(p):
                    angle = math.radians(45)-scan.angle_increment*index
                    distance_deriv = self.current_speed*math.cos(angle)
                    if (distance_deriv > 0): #Ensure no division by 0
                        ttcRHS.append(p/distance_deriv)
            ttc = ttcLHS + ttcRHS
            #rospy.loginfo("min value %.2f",np.min(ttc))
            minTTC = 0 
            try:
                minTTC = np.min(ttc)
            except ValueError:
                pass
            if (minTTC > 0 and np.min(ttc) < self.ttc_threshold):
                rospy.loginfo("AUTOMATIC BREAKING ENGAGED")
                self.drive(0, 0)
                self.EMERGENCY_STOP = True 
            else:
                self.EMERGENCY_STOP = False


    def drive(self, desired_speed, desired_steering_angle):
        # SPEED LIMITER
        if (desired_speed > self.speed_limit):    #if above speed limit
            self.current_speed = self.speed_limit
        elif (desired_speed < -self.speed_limit): #if below negative speed limit - twin will never, but controller might
            self.current_speed = -self.speed_limit
        else:
            self.current_speed = desired_speed

        self.steering_angle = desired_steering_angle
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "frame_id"
        ack_msg.drive.speed = self.current_speed
        ack_msg.drive.steering_angle = self.steering_angle
        self.pub.publish(ack_msg)

    def control_switch(self, data):
        if data.buttons[4] == 1: #L1
            self.MANUAL_CONTROL = True
        if data.buttons[4] == 0:
            self.MANUAL_CONTROL = False
            #self.drive(0,0)
        if data.buttons[7] == 1: #R2
            rospy.loginfo("Disabled emergency brake")
            self.EMERGENCY_STOP = False
        if data.buttons[2] == 1: #circle
            self.ppControl = True
        if data.buttons[2] == 0:
            self.ppControl = False
        if data.buttons[3] == 1: #triangle
            self.EMERGENCY_STOP = True
            self.drive(0,0)
    
    
    def digital_twin_control(self, data):
        if not self.EMERGENCY_STOP and not self.MANUAL_CONTROL and not self.ppControl:
            rospy.loginfo("DT enganged")
            self.drive(data.drive.speed, data.drive.steering_angle)

    def manual_control(self, data):
        if self.MANUAL_CONTROL and not self.EMERGENCY_STOP:
            rospy.loginfo("ManualControl enganged")
            self.drive(data.drive.speed, data.drive.steering_angle)

    def purepursuit(self,data):
        if self.ppControl and not self.EMERGENCY_STOP and not self.MANUAL_CONTROL:
            rospy.loginfo("PurePursuit enganged")
            self.drive(data.drive.speed,data.drive.steering_angle)

if __name__ == '__main__': 
    try: 
        Brain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
