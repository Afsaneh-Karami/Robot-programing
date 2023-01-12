#!/usr/bin/env python 

import rospy
#from cvlib.object_detection import draw_bbox
#import matplotlib.pyplot as plt
from cv2 import namedWindow, cvtColor, imshow, inRange, imwrite
from cv2 import contourArea , arcLength, minEnclosingCircle,moments,circle, putText , FONT_HERSHEY_SIMPLEX
from cv2 import destroyAllWindows, startWindowThread
from cv2 import GaussianBlur,blur,medianBlur,bilateralFilter
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import resize, INTER_CUBIC
from cv2 import findContours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,CHAIN_APPROX_NONE,drawContours
from numpy import mean
from cv_bridge import CvBridge, CvBridgeError
from math import ceil, isnan,isclose
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped,Point32
import image_geometry
from std_msgs.msg import Int32 

class image_converter():
    
    # aspect ration between color and depth cameras based on info in URDF file h
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file
    # ((84.1*3.14/180)/1920) / ((70.0*3.14/180)/512)
    color2depth_aspect_h = (84.1/1920) / (70.0/512)  
    color2depth_aspect_v = (54/1080) / (61/424) 
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub_color = rospy.Subscriber("/thorvald_001/kinect2_left_camera/hd/image_color_rect", Image, self.image_color_callback)
        self.image_sub_depth=rospy.Subscriber("/thorvald_001/kinect2_left_sensor/sd/image_depth_rect",Image, self.image_depth_callback)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_left_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        self.sub_coor_list=rospy.Subscriber("coor_list",Point32,self.coor_callback)
        self.tf_listener = tf.TransformListener()
        self.pub_count = rospy.Publisher("count", Int32, queue_size=1)  # publish the number of counted grapes bunches
        self.pub_coor_list = rospy.Publisher("coor_list", Point32, queue_size=1)  #publish a list of coordinate of counted grapes bunch-es
        self.contour_count=0
        self.position_to_map_coor_list=[]
        self.f=0
        self.camera_model = None
        self.image_depth = None
        self.save_pic = False
        self.filter_frame=True
        

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)  # it return fx and fy from camera info message
        self.camera_info_sub.unregister() #Only subscribe once  
          
    def image_depth_callback(self, data):
        self.image_depth = data  # it return the data of depth from Image message

    def coor_callback(self,data):
        X=data.x
        Y=data.y
        Z=data.z
        self.position_to_map_coor_list.append((round(X,2),round(Y,2),round(Z,2)))

    def image_color_callback(self, data):
        if self.camera_model is None: # it stop until info from cam-era info message arraived
            return
        if self.image_depth is None: # it stop until info from depth image message arraived
            return
        self.f=self.f+1 # count the numer of frame  
        self.cv_image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")   # change the format of iamge
        self.cv_image_depth = self.bridge.imgmsg_to_cv2(self.image_depth, "32FC1") # change the format of iamge

        # using cvlib library for detecting common objects
        #box,label,c_score=cv.detect_common_objects(self.cv_image_color)
        #output=draw_bbox(self.cv_image_color,box,label,c_score)
        #plt.imshow(output)
        #plt.show()

        HSV_image=cvtColor(self.cv_image_color, COLOR_BGR2HSV)  # change the format of color from bgr to HSV

        # for creating some pictures of grapes while thorvald move for using at inrange function
        if self.save_pic is True:
            time = data.header.stamp
            imwrite(''+str(time)+'.jpeg', self.cv_image_color)
        
        # color filtering to detect grapes 
        mask = inRange(HSV_image, (90, 80, 70), (160, 190, 190))
        #mask = inRange(self.cv_image_color, (13, 52, 54), (18, 70, 46)) #RGB
        imshow("masked", mask)

        # using blure filtering to smooth the bunchs of grapes 
        blur_img= blur(mask, (9,9)) 
        imshow("blur_img", blur_img)
        
        # Using findcontours to detect grapes
        contours,hirarcy=findContours(blur_img, RETR_EXTERNAL, CHAIN_APPROX_NONE) # to detect the bunchs of grapes
        number_of_contour=self.process_contour(contours,self.cv_image_color)
        
        self.pub_count.publish(number_of_contour) 
        print("number of brach of contours is: ",number_of_contour)
        self.cv_image_color = resize(self.cv_image_color, None, fx=0.5, fy=0.5, interpolation = INTER_CUBIC)
        imshow("Image window with contour", self.cv_image_color)
        waitKey(1)

    def cal_contour_center(self,contour): #calculate the center of contour based on center of mass of object(moment)
        M=moments(contour)
        if M['m00']!=0:   # aviod division by zero
            x_contour_center=(M['m10']/M['m00'])
            y_contour_center=(M['m01']/M['m00'])
        else:
            return None
        return x_contour_center,y_contour_center
    
    def depth_value(self,x,y):
        # principal point at center of depth image(cxd,cyd)
        cxd=self.cv_image_depth.shape[1]/2
        cyd=self.cv_image_depth.shape[0]/2
        # principal point at center of color image(cx,cy)
        cx=self.cv_image_color.shape[1]/2
        cy=self.cv_image_color.shape[0]/2
        # calculate the center point of contours at depth image
        depth_coords = (cxd + (x - cx)*self.color2depth_aspect_h , cyd + (y - cy)*self.color2depth_aspect_v) #(xd,yd)
        # find the value of depth which indicate the distance from camera
        try:
            depth_value = self.cv_image_depth[int(depth_coords[1]), int(depth_coords[0])] 
        except:
            depth_value =100
        circle(self.cv_image_depth, (int(depth_coords[0]), int(depth_coords[1])), 0, (0,0,0),8) # for checking in depth image
        imshow("depth_image",self.cv_image_depth)
        return depth_value     
    
    def project_2d_3d_coor(self,x,y,depth_value): # calculate ob-ject's 3d location in camera coords
        camera_coords = self.camera_model.projectPixelTo3dRay((x, y))#project the image coords (x,y) into 3D ray in camera coords 
        camera_coords = [i/camera_coords[2] for i in camera_coords] # adjust the resulting vector so that z = 1
        camera_coords = [j*depth_value for j in camera_coords] # mul-tiply the vector by depth
        return camera_coords

    def tf_camera_map(self,camera_coords):  #define a point in camera frame and transfer it to map frame
        object_location = PoseStamped()
        object_location.header.frame_id = "thorvald_001/kinect2_left_rgb_optical_frame"
        object_location.pose.orientation.w = 1.0
        object_location.pose.position.x = camera_coords[0]
        object_location.pose.position.y = camera_coords[1]
        object_location.pose.position.z = camera_coords[2]
        # the coordinates in the map frame
        self.p_camera = self.tf_listener.transformPose('map', object_location)
        X=self.p_camera.pose.position.x
        Y=self.p_camera.pose.position.y
        Z=self.p_camera.pose.position.z
        return X,Y,Z

    def calculate_Xf(self,u,v):
        depth_value_f=self.depth_value(u , v)
        camera_coords_f=self.project_2d_3d_coor(u , v , depth_value_f)
        Xf,Yf,Zf=self.tf_camera_map(camera_coords_f) 
        return Xf 

    def process_contour(self,contours,image):
        drawContours(image, contours, -1, (255,0,255),2)
        if self.f!=1 and self.filter_frame is True:
                self.Xf2n=self.calculate_Xf(1755,752)
                self.Xf1n=self.calculate_Xf(165,752)
                #print("self.Xf1n self.Xf2n",self.f,self.Xf1n,self.Xf2n)
        for contour in contours:     # process contour 
            area=contourArea(contour)
            if self.cal_contour_center(contour) is None:
                continue

            else:
                x_contour_center , y_contour_center=self.cal_contour_center(contour)
            depth_value=self.depth_value(x_contour_center , y_contour_center)
            if depth_value==100:
                continue
            while (isnan(depth_value)): 
                x_contour_center=x_contour_center
                y_contour_center=y_contour_center+1
                depth_value=self.depth_value(x_contour_center , y_contour_center)
                
            camera_coords=self.project_2d_3d_coor(x_contour_center , y_contour_center , depth_value)
            X,Y,Z=self.tf_camera_map(camera_coords) 
            X,Y,Z=round(X,2),round(Y,2),round(Z,2)

            if self.f!=1 and self.filter_frame is True:
                if self.Xf2-self.Xf2n>0 and X>self.Xf2:
                    continue
                if  self.Xf2-self.Xf2n<0 and X<self.Xf1:
                    continue 
            
            if area>=1300:
                
                circle(image, (int(x_contour_center),int(y_contour_center)), 0, (0,0,0),8)
                ((x,y),radius)=minEnclosingCircle(contour)
                if (len(self.position_to_map_coor_list)==0):
                    # creating a publisher
                    object_coor = Point32()
                    object_coor.x=X
                    object_coor.y=Y
                    object_coor.z=Z
                    self.pub_coor_list.publish(object_coor)
                    self.contour_count=self.contour_count+1
                    putText(image, str(self.contour_count), (int(x_contour_center+5),int(y_contour_center+5)), FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),2)
                    circle(image, (int(x_contour_center),int(y_contour_center)), 0, (255,255,255),3)   # draw a center point forr each contour
                else:
                    common=0   
                    for item in self.position_to_map_coor_list:
                        if isclose(X,item[0],rel_tol=0.15) is True:
                            if isclose(Y,item[1],rel_tol=0.15) is True:
                                if isclose(Z,item[2],rel_tol=0.15) is True:
                                    common=1
                    if common==0:
                        object_coor = Point32()
                        object_coor.x=X
                        object_coor.y=Y
                        object_coor.z=Z
                        self.pub_coor_list.publish(object_coor)
                        self.contour_count=self.contour_count+1
                        putText(image, str(self.contour_count), (int(x_contour_center+5),int(y_contour_center+5)), FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),2)
                        circle(image, (int(x_contour_center),int(y_contour_center)), 0, (255,255,255),3)   # draw a center point forr each contour  
            
        if self.f==1 and self.filter_frame is True:
            self.Xf1=self.calculate_Xf(165,752)
            self.Xf2=self.calculate_Xf(1755,752)
        elif self.filter_frame is True:
             self.Xf1=self.Xf1n
             self.Xf2=self.Xf2n
             #print("self.Xf1,self.Xf2",self.f,self.Xf1,self.Xf2)
        return self.contour_count
     

#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

#destroyAllWindows()

