#!/usr/bin/env python
import rospy 
import json
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
class PP:
    def __init__(self):
        rospy.init_node("PureTest",anonymous=True)
        self.rate = rospy.Rate(100)
        self.sub = rospy.Subscriber('/scan',LaserScan,self.callback)
        self.pub = rospy.Publisher('/ppdata',AckermannDriveStamped,queue_size=1)
    
    """
    Publishes ackermann msg with angle - TODO: Change speed depending on distance?
    """
    def publishdrive(self,distance,angle):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id='pp data'
        ack_msg.drive.speed = 1
        ack_msg.drive.steering_angle = angle
        self.pub.publish(ack_msg)
        pass

    """
    Callback function for laserscan.
    Takes LaserScan message, points for average, and number of scans.
    """
    def callback(self,scan,pfa = 50,nos = 300):
        self.scanranges = np.asarray(scan.ranges) #Convert from tuple to array
        half_nos = int(nos / 2)
        
        
        #Angular resolution
        phi = scan.angle_increment
        #Half the averagepoints
        half_pfa = int(pfa / 2)

        #Take the subset from centered around point 540 (lidar FOV centerpoint). 
        ranges = self.scanranges[540-half_nos-half_pfa:540+half_nos+half_pfa]
        #ranges = ranges.tolist()



        avrg_ranges = [0]*nos

        #set the fpa (overflow protection essentially) to same length as first and last scan in fov. 
        for i in range(half_pfa):
            avrg_ranges[i] = ranges[half_pfa]
            ranges[i + nos +
                   half_pfa] = ranges[nos + half_pfa]
        #for the other values in fov, take the average of the pfa neighbours. 
        for i in range(nos):
            avrg_ranges[i] = np.average(ranges[i: i + nos])
        #get the index and length of the largest value (average length)
        idx_furthest = np.argmax(avrg_ranges)
        length = avrg_ranges[idx_furthest]
        #Calculate the angle wrt. car orientation
        angle = phi * idx_furthest + phi*((1080-nos)/2)-np.radians(135)
        #normalize (max value1 / maxvalue2)
        print(length,"__",angle)
        print("max:",max(ranges))
        angle_norm = 0.34 / (np.pi/2)
        self.publishdrive(length,angle*angle_norm)

if __name__ == '__main__': 
    try: 
        PP()
        print("Spinning")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
