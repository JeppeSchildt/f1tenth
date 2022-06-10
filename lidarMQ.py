#!/usr/bin/env python
#for ros
#from requests import request
import rospy
import numpy as np
#for rabbitmq
import pika
import datetime
import json

from sensor_msgs.msg import LaserScan

#import numpy as np

class LidarMQ():
    def __init__(self):
        
        rospy.init_node('LidarMQ', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

        self.ranges = 0   
        self.last_scan = rospy.Time.now()
        self.scantime_interval = rospy.Duration(secs=0.2)
        #Connection to broker
        self.connectionToBroker = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        #self.connectionFromBroker = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))

        #Channel to broker
        self.channelToBroker = self.connectionToBroker.channel()
        self.channelToBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')

        #Subscribing to LaserScan
        self.sub=rospy.Subscriber('scan', LaserScan, self.lidar_callback)       

        
        
        

    def lidar_callback(self, scan):  
        if (rospy.Time.now()-self.last_scan > self.scantime_interval):
            #get time with for rabbit msg
            rt = rospy.get_rostime()
            self.last_scan = rospy.Time.now()
            rostime = rt.secs + rt.nsecs * 1e-09
            rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
            
            #checking if the send channel and connection is open
            if not self.channelToBroker or self.connectionToBroker.is_closed:
                self.channelToBroker = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
                self.channelToBroker = self.connectionToBroker.channel()
                self.channelToBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
            #Getting ranges from LaserScan
            self.ranges = scan.ranges
            print("max:",max(self.ranges))
            print("average:",np.average(self.ranges))
            #print("max3:",max(self.ranges.tolist()))
            # CHANGE ROUTING KEY
            routing_key = "lidar.ranges"
            """
            message = {
                "time": rostimeISO.isoformat(timespec='milliseconds'), 
            
                "seq":scan.header.seq,
                "secs":scan.header.stamp.secs,
                "nsecs":scan.header.stamp.nsecs,
                "frame_id": scan.header.frame_id,
                
                "angle_min": scan.angle_min,
                "angle_max": scan.angle_max,
                "angle_increment": scan.angle_increment,
                
                "time_increment": scan.time_increment,
                
                "scan_time": scan.scan_time,
                
                "range_min": scan.range_min,
                "range_max": scan.range_max,
                
                "ranges": self.ranges,
                "intensities": scan.intensities,
                }
            """
            message = {
                "time": rostimeISO.isoformat(timespec='milliseconds'), 
                "ranges": self.ranges,
                }
            self.channelToBroker.basic_publish(
                exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))

        
if __name__ == '__main__': 
    try: 
        LidarMQ()
        print("Spinning")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
