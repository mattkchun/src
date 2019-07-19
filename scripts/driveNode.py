#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from turnRectStarter import sift_det
from newZed import Zed_converter
#from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables
AUTONOMOUS_MODE = True


#What should your drive topic be?
DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"

class driveStop(object):
    """class that will help the robot drive and stop at certain conditions"""
    def __init__(self):
        """initalize the node"""
        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.sub_scan = rospy.Subscriber(SCAN_TOPIC,LaserScan,self.scan_callback,queue_size=1)
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
        
        self.drive_state = -1
        self.turnDirection = 0
        self.timer=0
        self.last_dir = 0
    
    
    def scan_callback(self, data):
        self.data = data
    
    def size_calc(self):
        """ calculate the x and y size of the box in pixels"""
        #rospy.loginfo("box_size: {}".format(self.box_size))
        width = self.flag_box[1][0] - self.flag_box[0][0]
        height = self.flag_box[1][1] - self.flag_box[0][1]
        self.box_size = width*height
        self.box_x = (self.flag_box[0][0]+self.flag_box[1][0])/2
        self.box_left = self.flag_box[0][0]
        
        offset = 0.05 + (width/2000) #adjust for left camera offset, adjust more when clsoer to cone
        self.new_angle = offset -(self.box_x-320)/1000.0 #1000.0
        #rospy.loginfo("a: {} w: {}".format(self.new_angle, width))

    def driveStop_car_callback(self,data):
        
        """laser scan callback function"""
        #checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
		
        #applies the current filter to the image and returns a bounding box
        self.flag_box = cd_color_segmentation(self.camera_data.cv_image)
        #rospy.loginfo("box: {}".format(self.flag_box))
        #finds the size of the box
        self.size_calc()
        #rospy.loginfo("timer: {}".format(self.timer))
        if self.drive_state == -1:
            scan = data.ranges
            max = len(scan)
            front = scan[(19*max)/40:(21*max)/40]
            if min(front)<2.6:
                self.drive_state=0
            else:
                self.cmd.drive.speed = 1
                self.cmd.drive.steering_angle = 0.045
        elif self.drive_state == 0:
            dir = self.signdir(self.camera_data.cv_image)
            scan = data.ranges
            max = len(scan)
            front = scan[(9*max)/20:(11*max)/20]
            rospy.loginfo("min: {}".format(min(front)))
            if dir != 0 and dir == self.last_dir:
                self.turnDirection = dir
                self.drive_state = 1
                self.timer = 35
            else:
                self.cmd.drive.speed = 0
                self.last_dir = dir
            
                if min(front)<2.1:
                    self.cmd.drive.speed = 0
                    self.cmd.drive.steering_angle = 0
                else:
                    self.cmd.drive.speed = 0.3
                    self.cmd.drive.steering_angle = 0.045
#            if self.find_sign():
#                self.drive_state=1
#                self.timer = 15
#            else:
#                scan = data.ranges
#                max = len(scan)
#                front = scan[(9*max)/20:(11*max)/20]
#                #rospy.loginfo("min: {}".format(min(front)))
#                if min(front)<0.8:
#                    self.cmd.drive.speed = 0
#                    self.cmd.drive.steering_angle = 0
#                else:
#                    self.cmd.drive.speed = 0.4
#                    self.cmd.drive.steering_angle = 0.045
        elif self.drive_state == 1:
            if self.timer<=0:
                self.drive_state=2
                self.timer = 15
            else:
                self.cmd.drive.steering_angle = -self.turnDirection
                self.cmd.drive.speed = 1
                self.timer-=1
        elif self.drive_state == 2:
            if self.timer<=0:
                self.drive_state=3
            else:
                self.cmd.drive.steering_angle = self.turnDirection
                self.cmd.drive.speed = -1
                self.timer-=1
        elif self.drive_state == 3:
            self.drive_to_cone()
#        elif self.drive_state == 4:
#            center_cone()
        else:
            pass
	
    def drive_to_cone(self):
        """write driving commands here! You don't need to publish the drive command,
        that is being taken care of in the main() function"""
        #box size is 150,220
        #rospy.loginfo("box_size: {}".format(self.box_size))
        scan = self.data.ranges
        max = len(scan)
        rospy.loginfo("low: {}, high: {}".format(int(((8-(self.new_angle*3))*max)/20),int(((12-(self.new_angle*3))*max)/20)))
        front = scan[int(((8-(self.new_angle*3))*max)/20):int(((12-(self.new_angle*3))*max)/20)]
        rospy.loginfo("min: {}".format(min(front)))
        if self.box_size<20:
            self.cmd.drive.steering_angle = self.cmd.drive.steering_angle = 0.045 #-self.turnDirection
            self.cmd.drive.speed = 1
        elif min(front)<0.26:
            self.cmd.drive.speed = 0
            self.cmd.drive.steering_angle = 0
        elif self.box_size < (5000):
            self.cmd.drive.steering_angle=self.new_angle
            self.cmd.drive.speed = 1
        else:
            self.cmd.drive.steering_angle=self.new_angle
            self.cmd.drive.speed = 0.3

    def find_sign(self):
        color_box, sign_box = sift_det("images/oneway.jpg",self.camera_data.cv_image)
        rospy.loginfo("cb: {} sb: {}".format(color_box,sign_box))
        if color_box == None or sign_box == None:
            return False
        if abs(color_box[0][0]-sign_box[0][0]) < 100: #, abs(color_box[0][1]-sign_box[0][1]), abs(color_box[1][0]-sign_box[1][0]), abs(color_box[1][1]-sign_box[1][1])]
            x_diff = color_box[0][0] - sign_box[0][0]
            if x_diff<0: #positive = right
                self.turnDirection = -1
            else:
                self.turnDirection = 1
            rospy.loginfo("turn_dir: {}".format(self.turnDirection))
            return True
        else:
            return False

#    def center_cone(self):
#        if timer==0:
#            if self.box_size > 15000:
#                if abs(self.box_x-320)<200:
#                    self.cmd.drive.speed=0
#                    self.cmd.drive.steering=0
#                    self.drive_state = 4
#                else:
#                    self.cmd.drive.speed=-1
#                    self.cmd.drive.steering=-self.new_angle
#                    timer = 10
#            else:
#                self.drive_state = 2
#        elif timer>0:
#            self.cmd.drive.speed=-1
#            self.cmd.drive.steering=-self.new_angle
#            timer = 10
#            timer-=1

    def signdir(self,img,threshold=.6,bestmatch=False):
        img_rgb = img
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        bw = cv2.threshold(img_gray, 90, 255, cv2.THRESH_BINARY)[1]
#        template = cv2.imread("images/LEFT.png",0)
#        lres = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
#        template = cv2.imread("images/RIGHT.png",0)
#        rres = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
        template = cv2.imread("images/LEFTS.png",0)
        lress = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
        template = cv2.imread("images/RIGHTS.png",0)
        rress = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
#        template = cv2.imread("images/LEFTxS.png",0)
#        lresxs = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
#        template = cv2.imread("images/RIGHTxS.png",0)
#        rresxs = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
        output=0
        if(bestmatch):
            output = 0
#            if(np.max(lres)<np.max(rres)):
#                output=1
#            else:
#                output=-1
        else:
#            if(np.max(lres) >= threshold) or (np.max(lress)>= threshold) or (np.max(lresxs)>= threshold):
#                output-=1
#            if(np.max(rres) >= threshold) or (np.max(rress)>= threshold) or (np.max(rresxs)>= threshold):
#                output+=1
            if np.max(lress)>= threshold:
                output-=1
            if np.max(rress)>= threshold:
                output+=1
        rospy.loginfo("output: {}".format(output))
        return output

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
