from cv2 import namedWindow, cvtColor,imread
from cv2 import COLOR_BGR2HSV
read=imread('/home/ubuntu/Desktop/catkin_ws/src/my_package/image/1.png') #BGR
img=cvtColor(read, COLOR_BGR2HSV) #HSV
for i in range(0,img.shape[0]):
    for j in range(0,img.shape[1]):
        print(img[i,j])


