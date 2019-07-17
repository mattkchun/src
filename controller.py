#!/usr/bin/python2

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Racecar:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    DESIRED_DISTANCE = 1 
    behindforce=0
    fpdconst=.5
    ktheta=1
    def __init__(self):
        self.data = None
        self.angle = 0
        self.cmd = AckermannDriveStamped()
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,LaserScan,self.scan_callback,queue_size=1)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        

    def scan_callback(self, data):
        self.data = data
    def pol2cart(self,r, theta):
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return(x, y)
    def cart2pol(self,x, y):
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return(rho, phi)
        
        
        
    def fieldpot(self):
        points=[]
        for item in self.data.ranges:
            #inverse squer law
            x,y=self.pol2cart(-1*self.fpdconst/(float(item)**4),(np.pi/4-self.data.ranges.index(item))*1.5*np.pi/len(self.data.ranges))
            #type(points)-
            points.append([x,y])
            
        #print("points array is {0}".format(isdr))
        total=np.sum(points,axis=0)
        #print(total)
        total[1]+=self.behindforce*(len(self.data.ranges)/3)*1/self.DESIRED_DISTANCE**2
        tvel, trad =self.cart2pol(total[0],total[1])
        print("total array is {0}. total sum is (r,theta){1},{2}".format(total,tvel,trad))
        self.VELOCITY=1 #tvel
        angle=self.ktheta*trad
        print(tvel)
        self.drive(.5,angle)
        return 0
        
        
        
    def pid(self):
        #wall on right
        pos=min(self.data.ranges)
        if pos>2*self.DESIRED_DISTANCE:
            self.nowall=True
        if self.nowall:
            if pos<1.2*self.DESIRED_DISTANCE:
                self.nowall=False
            return 0
        p=(self.DESIRED_DISTANCE-pos)
        d=np.cos((len(self.data.ranges)-self.data.ranges.index(max(self.data.ranges)))*1.5*np.pi/len(self.data.ranges))#rads/point
        self.lastsum+=p
        i=self.lastsum
        #print ("kp is {0}. ki is {1}. kd is {2}. the pos is {3} the p is{4} the i is{5} the d is{6}.".format(self.kp,self.ki,self.kd,pos, p,i,d))
        self.drive(.5,p*self.kp+d*self.kd+i*self.ki)
	return 0
    def autorun():
        self.feildpot()
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub_drive.publish(msg)

    def stop(self):
        self.drive(0,0)

    def scan(self):
        return self.scan

rospy.init_node('controller')
rate = rospy.Rate(60)
rc = Racecar()

while not rospy.is_shutdown():
    # TODO implement controller logic here
    rc.drive(1,0)
    rate.sleep()
