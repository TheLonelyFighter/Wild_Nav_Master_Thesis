import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time


#generate HSV image
def extract_rectangle_building(segmented_img):
    """
    Takes a segmented RGB image and returns coordinates of a rectangle that bounds buildings (blue) in the input image. If no 
    building is present, return None,None,None,None.
    """
    x,y,w,h = 0,0,0,0

   

    # the color outputted by the segmentor is always the same
    building_color_lower = np.array([0,0,120]) 
    building_color_upper = np.array([0,17,130]) 
    #generate the mask
    mask = cv.inRange(segmented_img, building_color_lower, building_color_upper)
    

    #extract the masked image from the original image
    segmented_img = cv.bitwise_and(segmented_img, segmented_img, mask=mask)

    plt.imshow(segmented_img)
    plt.show()
    #genrate greyscale version for finding contours
    gray = cv.cvtColor(segmented_img, cv.COLOR_RGB2GRAY)

    #find contours
    contours, hyerarchy = cv.findContours(gray, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        print("Building detected")
        max_area = -1
        max_cnt_index = 0
        for i in range(len(contours)):
            area = cv.contourArea(contours[i])
            if area>max_area:
                max_cnt_index = i
                max_area = area

        cv.drawContours(segmented_img, contours, max_cnt_index, (255, 0, 0), 3)

        

        x,y,w,h = cv.boundingRect(contours[max_cnt_index])
        # template = segmented_img[y:y+h, x:x+w]
        # plt.imshow(template)
        # plt.show()
        cv.rectangle(segmented_img,(x,y),(x+w,y+h),(0,255,0),2)
        plt.imshow(segmented_img)
        plt.show()
        # plt.imshow(segmented_img)
        # plt.show()

    return x,y,w,h

# segmented_img = cv.imread("../photos/segmented/mask_1.png")

# start = time.time()
# segmented_img = cv.cvtColor(segmented_img, cv.COLOR_BGR2RGB)

# rect =  extract_rectangle_building(segmented_img)
# stop = time.time()
# print("Time elapsed: ", stop - start)
# print(rect)