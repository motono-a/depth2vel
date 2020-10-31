#!/usr/bin/env python
import rospy
import time
import numpy as np

from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callback)
    self.roi_l = 0
    self.roi_r = 424
    self.roi_t = 100
    self.roi_b = 165
    self.pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=1)
    self.kp = 0.5
    self.ki = 0.02
    self.kd = 0.0
    self.angle_i = 0.0
    self.angle_i_limit = 0.3
    self.angle_d = 0.0
    self.cam_w = 424
    self.cam_h = 240.0
    self.cam_angle = 85.2
    self.wall_check = 220.0
    self.wall_roi_l = 40
    self.wall_roi_r = 364
    self.wall_roi_t = 150
    self.wall_roi_b = 160
    self.wall_turn = 80.0
    self.angle_offset = 3.0/180.0*3.14
    #self.angle_offset = 0.19

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

   
    #cut target area roi image
    img_band = cv_image[self.roi_t: self.roi_b, self.roi_l: self.roi_r]
    wall_band_l = cv_image[self.wall_roi_t: self.wall_roi_b, self.wall_roi_l: self.cam_w/2]
    wall_band_r = cv_image[self.wall_roi_t: self.wall_roi_b, self.cam_w/2: self.wall_roi_r]

    #calculate closest point per line
    band_min = np.min(img_band, axis=0)
    wall_l_min = np.min(wall_band_l, axis=0)
    wall_r_min = np.min(wall_band_r, axis=0)

    #calculate farthest line
    band_max_index = np.argmax(band_min);
    band_max = np.max(band_min);
    #print(band_max_index)
    #print(band_max)

    #convert vel
    speed=-0.25
    degree=self.cam_angle*(self.roi_l+band_max_index)/self.cam_w - self.cam_angle/2;
    angle_d = (degree - self.angle_d)*self.kd/180*3.14;
    self.angle_d = degree

    # near wall case
    wall_l_ave = np.average(wall_l_min);
    wall_r_ave = np.average(wall_r_min);

    wall_ave = wall_l_ave
    wall_turn = self.wall_turn

    if wall_l_ave>wall_r_ave :
       wall_ave = wall_r_ave
       wall_turn = -self.wall_turn

    # wall setting reset
    #wall_ave = band_max

    if wall_ave<self.wall_check :
       speed=-0.15
       degree=wall_turn
       self.angle_i = 0.0

    angle=degree/180*3.14;

    angle_kp = angle*self.kp

    if wall_ave>self.wall_check :
       self.angle_i += angle*self.ki

    if self.angle_i > self.angle_i_limit :
       self.angle_i = self.angle_i_limit

    if self.angle_i < -self.angle_i_limit :
       self.angle_i = -self.angle_i_limit


    angle_mix = angle_kp + self.angle_i + angle_d + self.angle_offset

    #print(speed)
    #print(angle)
    print([band_max_index, band_max, angle_mix, wall_ave])

    #publish cmd_vel
    t = Twist()
    t.linear.x = speed
    t.angular.z = angle_mix
    self.pub.publish(t)


# initialization
if __name__ == '__main__':

	ic = image_converter()

	# setup ros node
	rospy.init_node('depth2vel')
	

	# start running
	rospy.spin()

