#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from newZed import Zed_converter
#from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables

AUTONOMOUS_MODE = True


#What should your drive topic be?
DRIVE_TOPIC = "/drive"


class driveStop(object):
    """class that will help the robot drive and stop at certain conditions"""
    def __init__(self):
        """initalize the node"""
        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)
    
        """ initialize the box dimensions"""
        self.box_size=0
        self.box_angle=350
        self.new_angle=0
        
        """driving code"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0
	
        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image = False)

        self.min_value=0
    
    def size_calc(self):
        """ calculate the x and y size of the box in pixels"""
        #rospy.loginfo("box_size: {}".format(self.box_size))
        width = self.flag_box[1][0] - self.flag_box[0][0]
        height = self.flag_box[1][1] - self.flag_box[0][1]
        self.box_size = width*height
        self.box_x = (self.flag_box[0][0]+self.flag_box[1][0])/2
        
        offset = 0.05 + (width/2000) #adjust for left camera offset, adjust more when clsoer to cone
        self.new_angle = offset -(self.box_x-320)/1000.0 #1000.0
        rospy.loginfo("a: {} w: {}".format(self.new_angle, width))

    def driveStop_car_callback(self,data):
        """laser scan callback function"""
        #checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
		
        #applies the current filter to the image and returns a bounding box
        self.flag_box = cd_color_segmentation(self.camera_data.cv_image)
        rospy.loginfo("box: {}".format(self.flag_box))
        #finds the size of the box
        self.size_calc()
		
        if AUTONOMOUS_MODE:
            self.drive()
        else:
            pass
	
    def drive(self):
        """write driving commands here! You don't need to publish the drive command,
        that is being taken care of in the main() function"""
        #box size is 150,220
        #rospy.loginfo("box_size: {}".format(self.box_size))
        self.cmd.drive.steering_angle=self.new_angle
        if self.box_size < (5000):
            self.cmd.drive.speed = 1
        else:
            self.cmd.drive.speed = 0

def main():
    global AUTONOMOUS_MODE
    try:
        ic = driveStop()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if AUTONOMOUS_MODE:
                ic.pub.publish(ic.cmd)
			
    except rospy.ROSInterruptException:
        exit()


if __name__ == "__main__":
    main()
