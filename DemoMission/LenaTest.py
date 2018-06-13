import cv2
from PIL import Image


lena = '/home/quad/Desktop/lena512.bmp'
img = cv2.imread(lena, 0)

#cv2.imshow('Realsense' , img)
#key = cv2.waitKey(1)

img2 = Image.fromarray(img, 'RGB')
img2.show()
